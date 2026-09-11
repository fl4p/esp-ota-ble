#include "bleota.h"
#include "cadence.h"
#include "telemetry.h"
#include "blebus_frame.h"
#include "caps.h"

#include <atomic>
#include "blesess.h"

#if PROTO_BLE_OTA

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <esp_mac.h>
#include <ota_ble.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <mbedtls/sha256.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>
#include <string>
#include "console.h"   /* LAST: shadows Serial with the gate a serial bus session silences */
#include <type_traits>
/* Same guard as main.cpp, and it matters more here: these prints come off the
 * NimBLE host task, on the other core, where no discipline in the app task
 * could stop them landing in the middle of a Modbus reply. */
static_assert(std::is_same<decltype(Serial), ConsoleGate>::value,
              "console.h must be the LAST include: Serial is not the gate");

/* Provided by main.cpp. Parks the SX1262 and flushes the console before a
 * reset. esp-ota-ble reboots from inside otaBleEnd(), so this is the only
 * chance to leave the LoRa PA/LNA switched off rather than resetting with the
 * receiver enabled. */
extern void node_prepare_restart(void);

/* Same UUIDs as smart-shunt-fw and fugu-mppt-firmware, deliberately: the host
 * tools are the same tools, and a second UUID set would mean a second copy of
 * every push script to keep in step. The service is distinguished by the
 * advertised NAME, not by its UUID. */
#define BLE_SVC_UUID       "e8308d3d-c3b4-45ff-ba58-9c0fb99d0ecb"
#define BLE_OTA_CTRL_UUID  "b0e0d1a4-7f52-4a3e-9c61-2d8f5b3ae741"
#define BLE_OTA_DATA_UUID  "b0e0d1a5-7f52-4a3e-9c61-2d8f5b3ae741"
/* Next in the same series. A separate characteristic rather than a command on
 * the control one: these are binary ADUs, the control channel is line-based
 * text, and multiplexing them would mean escaping Modbus frames that already
 * carry every byte value. */
#define BLE_BUS_UUID       "b0e0d1a6-7f52-4a3e-9c61-2d8f5b3ae741"
/* Next again, and NOTIFY-ONLY: no write property at all.
 *
 * The fast telemetry stream (telemetry.h) carries exactly the bytes
 * poll_build_frame() puts on the air over LoRaWAN, length-prefixed and chunked
 * the same way the bus channel's replies are - an ATT notification is not a
 * frame boundary any more than an ATT write is.
 *
 * It is a separate characteristic rather than a `telem` line on the control
 * channel for the reason the bus channel is separate: the frame is binary and
 * contains every byte value, the control channel is line-based text, and
 * multiplexing them would mean escaping a payload that already carries CRCs.
 *
 * It has no write property because it needs none - the ARMING is a verb on the
 * control characteristic, which is authenticated - and because a stream a
 * passer-by can only listen to is a materially smaller thing to have on the
 * air than one they can write to. What that costs is stated plainly in
 * bleota.h: subscribing is not authenticated, so the measurements themselves
 * are readable by anyone in range for as long as the window is open. */
#define BLE_TELEM_UUID     "b0e0d1a7-7f52-4a3e-9c61-2d8f5b3ae741"

static NimBLEServer         *srv       = nullptr;
static NimBLECharacteristic *chrCtrl   = nullptr;
static NimBLECharacteristic *chrData   = nullptr;
static NimBLECharacteristic *chrBus    = nullptr;
static NimBLECharacteristic *chrTelem  = nullptr;

#if PROTO_BLE_BENCH
// Private RAM-only receiver. All fields are owned by the NimBLE host task.
// It never opens an OTA handle, programs flash or changes the boot slot.
static uint8_t *benchRam = nullptr;
static size_t benchRamWant = 0, benchRamGot = 0;
static bool benchRamBad = false;
static int64_t benchRamFirst = 0, benchRamLast = 0;
static uint16_t benchDle[4] = {};
static void benchRamClear() {
    free(benchRam); benchRam = nullptr;
    benchRamWant = benchRamGot = 0; benchRamBad = false;
    benchRamFirst = benchRamLast = 0;
}
static int benchGap(ble_gap_event *event, void *) {
    if (event->type == BLE_GAP_EVENT_DATA_LEN_CHG) {
        const auto &d = event->data_len_chg;
        benchDle[0]=d.max_tx_octets; benchDle[1]=d.max_tx_time;
        benchDle[2]=d.max_rx_octets; benchDle[3]=d.max_rx_time;
    }
    return 0;
}
#endif

/* --- the RS-485 bus, reached over BLE ------------------------------------
 *
 * See the wire format in bleota.h. Everything here is written by the NimBLE
 * host task and read by the app task, or the reverse. The handover is a
 * two-slot queue: `bus_q_tail` publishes a filled slot, `bus_q_head` frees a
 * drained one, and each index is written by exactly one of the two tasks.
 *
 * MUTUAL EXCLUSION IS BY INDEX, PUBLICATION IS BY RELEASE/ACQUIRE, and the
 * second was always the hard part. With fewer than DEPTH queued, the tail's
 * slot cannot be the head's, so no slot is ever touched by both tasks - that
 * is a property of the arithmetic, not of the phases the tasks happen to run
 * in, which matters because a phase argument is what fails first under
 * revision. What it still does NOT give you is visibility, hence the
 * release/acquire pair: see the note on the queue below.
 *
 * ONE REQUEST IS ON THE BUS AT A TIME, unchanged - all Modbus RTU allows,
 * since it has no transaction id and the reply is unambiguous only by
 * construction. The queue is on the BLE SIDE ONLY: it lets the host's next
 * write cross the air while this exchange runs, instead of after it. A third
 * request is refused rather than queued. */
static const size_t BUS_MAX = 256;      /* a full Modbus ADU */

/* THE QUEUE, and why it is exactly two deep.
 *
 * One acked ATT write per Modbus block used to cost a full BLE round trip in
 * series with the exchange, because the host could not send request N+1 until
 * the reply to N came back. Two slots let it: the BLE round trip for N+1 now
 * overlaps the bus exchange for N, and since the bus is far slower than the
 * link the round trip disappears from the critical path entirely.
 *
 * A THIRD SLOT WOULD BUY NOTHING. ATT permits one outstanding transaction per
 * bearer (Core 6.0 ATT 3.3.2), and EATT is disabled in this NimBLE build, so
 * write-with-response writes serialise on the link however many the node will
 * hold. Depth 2 is enough to cover one exchange; depth 3 would only queue
 * writes that cannot overlap each other anyway.
 *
 * The bus itself is UNCHANGED: bus_service() takes one request, exchanges,
 * replies, and only then takes the next. Modbus RTU's "no transaction id, so
 * the reply is unambiguous by construction" property depends on that, not on
 * how the requests arrived. */
static const uint8_t BUS_Q_DEPTH = 2;
static uint8_t  bus_q[BUS_Q_DEPTH][BUS_MAX];
static size_t   bus_q_len[BUS_Q_DEPTH] = {0};
/* Single producer (NimBLE host task) advances the tail, single consumer (app
 * task) advances the head. Each writes ONLY its own index, so no slot is ever
 * touched by both: with fewer than DEPTH queued, tail's slot cannot be head's.
 * That is what makes this safe without a lock - and it is a property of the
 * indices, not of the phases the two tasks happen to run in. The reverted
 * bulk channel got this wrong by sharing one buffer between both tasks. */
static std::atomic<uint8_t> bus_q_head{0};
static std::atomic<uint8_t> bus_q_tail{0};

static inline uint8_t bus_q_count(void)
{
    return (uint8_t)(bus_q_tail.load(std::memory_order_acquire) -
                     bus_q_head.load(std::memory_order_acquire));
}
/* std::atomic, NOT `volatile bool`, and the difference is not pedantry.
 *
 * Review finding 2026-09-07: `volatile` in C++ orders nothing. It stops the
 * compiler caching the load and that is ALL it does - it emits no barrier, so
 * the NimBLE host task's writes to bus_req[] and bus_req_len may become
 * visible to the app task AFTER the flag that announces them. On a dual-core
 * ESP32-S3 with the two tasks pinned to different cores that is a real
 * reordering, not a theoretical one, and the symptom would be a Modbus frame
 * assembled from half of one request - i.e. exactly the splice the framing
 * layer exists to prevent, arriving underneath it.
 *
 * release/acquire is the primitive that actually says "everything I wrote
 * before this store is visible to whoever observes it". The earlier comment
 * here argued no lock was needed because the two tasks touch bus_req in
 * disjoint phases. That argument is still true and still insufficient:
 * mutual exclusion was never the missing piece, PUBLICATION was.
 *
 * The same argument now carries the queue: bus_q_tail is the release store
 * that publishes a slot, bus_q_head the one that frees it. */
/* Reassembly. An ATT write is not a frame boundary. Host task only. */
static uint8_t  bus_rx[BUS_MAX + 2];
static size_t   bus_rx_len = 0;
static uint32_t bus_rx_at = 0;         /* millis() of the last chunk */
static uint32_t bus_last_ms = 0;
/* Raised by onDisconnect(), consumed by the next write. A link that dropped
 * mid-frame guarantees nothing about ordering across it. */
static std::atomic<bool> bus_link_broke{true};
static std::atomic<bool> bus_flush_pending{false};
static ble_bus_exchange_fn bus_exchange = nullptr;
static bool (*bus_is_claimed)(void) = nullptr;

static std::atomic<bool> up{false};
/* THE CONNECTION AND THE AUTHENTICATION, IN ONE WORD. See blesess.h: they used
 * to be two independent atomics and the teardown cleanup was decided from a
 * compound read of both, which two tasks are allowed to be halfway through -
 * review 2026-09-10, finding 3. `sess` replaces them, a teardown is one atomic
 * exchange, and the caller that wins the exchange owns the cleanup. The
 * accessors below keep every reader in this file spelling it the same way. */
static BleSess sess;
static inline bool have_conn(void) { return blesess_conn(&sess); }
static inline bool authed(void)    { return blesess_authed(&sess); }
/* Set for the duration of ble_ota_down()'s teardown. Read by the NimBLE host
 * task, written by the app task, hence volatile.
 *
 * MEASURED CRASH, 2026-09-07, bench node at the 900 s hard deadline:
 *
 *   ble: client disconnected (reason 534)
 *   E NimBLEAdvertising: Error enabling advertising; rc=30
 *   assert failed: heap_caps_free ... "free() target pointer is outside heap areas"
 *
 * NimBLEDevice::deinit(true) terminates the live connection, so the host task
 * runs onDisconnect() from INSIDE deinit. That callback re-armed advertising -
 * correct for a peer that merely went away, fatal here, because the objects
 * advertising touches are already being destroyed. rc=30 is BLE_HS_EDISABLED:
 * the host was told it was disabled and the callback asked anyway.
 *
 * Clearing `up` before deinit would be enough on its own, and it is done too.
 * A named flag is here because the reason is not local: a reader of
 * onDisconnect() cannot otherwise tell that `up` is doing double duty as
 * "a teardown is in progress on another task". */
static volatile bool teardown     = false;
/* millis() when the current peer connected, for BLE_OTA_AUTH_GRACE_MS. */
static uint32_t conn_at       = 0;
static uint16_t conn_handle   = BLE_HS_CONN_HANDLE_NONE;
static uint16_t peer_mtu      = 23;
static bool     flash_busy    = false;
static std::atomic<bool> serial_driving{false};
static std::atomic<unsigned> link_tune_stage{0};
static std::atomic<bool> link_params_seen{false};
static uint32_t link_tune_at = 0;  // app task only

/* RESCUE RESET OVER BLE. Set by the `reset` verb on the NimBLE host task,
 * drained by ble_ota_tick() on the app task - the same latch shape as
 * cadence/telem/claim, and for a stronger reason than theirs: rebooting from
 * inside a GATT callback tears the stack down from within its own task.
 *
 * WHY IT EXISTS. The ESP32-S3 USB-Serial-JTAG endpoint dies while the chip
 * keeps running - five times on 2026-09-10 alone - and when it does, esptool
 * answers on none of default_reset/usb_reset/no_reset and the app console is
 * silent. The chip is fine; only the USB peripheral is wedged, and a reset
 * re-enumerates it. Without this the only way back is a human unplugging the
 * cable, which blocked flashing and erase_region repeatedly through one
 * session. BLE stayed up through every one of those deaths, so BLE is the
 * rescue channel. */
static std::atomic<bool> reset_pending{false};

static uint16_t auth_nonce    = 0;
/* ATOMIC because it is no longer host-task-only: telem_holding_window() reads
 * it from the app task to decide whether the fast path may spend the RS-485 bus
 * (see the note there). A 32-bit-aligned bool would not tear on this core, but
 * the ordering is what matters and the rest of this file's cross-task state is
 * already atomic for the same reason. */
static uint8_t  auth_fails    = 0;

/* millis() of the last thing the peer actually DID on this link, as opposed to
 * merely staying connected. Gates the connection grace in ble_ota_tick(): an
 * idle connection must not be able to hold the radio up, or a client that
 * connects and does nothing costs the battery the whole window. */
static std::atomic<uint32_t> last_act_ms{0};

/* Cadence request handed from the NimBLE host task to the app task. See
 * ble_ota_take_cadence_request() in the header for why this is a latch and not
 * a direct apply.
 *
 * std::atomic WITH EXPLICIT RELEASE/ACQUIRE, for exactly the reason spelled out
 * on the queue above, and this latch has the same shape: the host task writes
 * two payload words and THEN the flag that announces them, and the app task
 * reads the flag and then the payload. `volatile` stops the compiler caching a
 * load and emits no barrier, so on two pinned cores the payload writes may
 * become visible AFTER the flag - and the app task would commit a cadence
 * assembled from a half-published request. Mutual exclusion is not the missing
 * piece here either; PUBLICATION is. */
static std::atomic<bool> cad_pending{false};
static uint16_t cad_req_s     = 0;
static uint16_t cad_req_lease = 0;

/* The fast-telemetry request, latched the same way and for the same reason:
 * onWrite() runs on the NimBLE host task, and the TelemState it arms lives on
 * the app task, which is also the only task allowed to drive the Modbus bus. */
static std::atomic<bool> telem_pending{false};
static uint16_t telem_req_s     = 0;
static uint16_t telem_req_lease = 0;

/* Is a peer subscribed to the telemetry characteristic's notifications?
 *
 * WRITTEN BY THE HOST TASK (onSubscribe, onDisconnect), READ BY THE APP TASK on
 * every service call, so it is atomic for the same publication reason as
 * everything else in this file.
 *
 * The node emits only while somebody is listening, and that is not an
 * optimisation: a notification to an unsubscribed peer is dropped by the stack,
 * so a scheduler that ran anyway would poll the RS-485 bus every few seconds
 * for nobody - contending with the LoRaWAN poll and spending the very budget
 * the lease exists to protect. */
static std::atomic<bool> telem_subscribed{false};

/* Supplied by main.cpp, exactly as the bus predicates are: this module owns no
 * TelemState (it lives on the app task, next to the poller it drives) and must
 * not keep a second copy of the answer. Null in a build that never arms it. */
static bool (*telem_is_armed)(void) = nullptr;

/* THE CLAIM REQUEST CARRIES ITS OWNER. A bare pending flag let an
 * authenticated peer queue `claim`, disconnect, and have the app task install
 * the claim afterwards for a peer that was gone - review 2026-09-10, finding 4.
 * The latch is stamped with the session id and the consumer compares it against
 * the session that is live when it runs. See blesess.h. */
static BleClaimLatch     ble_claim;
static std::atomic<bool> ble_release_pending{false};
static std::atomic<bool> ble_disconnect_disarm{false};

/* millis() deadlines. `hard_deadline` is the one the battery depends on; it is
 * NOT extended by transfer progress, only by an accepted CTRL SESSION_OPEN.
 *
 * `up_since` is the one that cannot be extended by anything at all. Review
 * finding: BLE_OTA_MAX_UP_MS was described as an "absolute ceiling" and was
 * nothing of the kind - a renewal replaces hard_deadline, a fresh nonce
 * arrives with every 300 s uplink, and a stuck host process (or a SessionKeeper
 * nobody is watching) can therefore answer every invitation and hold the radio
 * and Class C up forever. The lease is deliberate and a long push needs it;
 * the missing piece was a bound on the TOTAL, which is what this is. When it
 * expires the radio goes down and stays down until the next bring-up, however
 * many renewals arrive. */
/* std::atomic for the same reason the queue indices are, and this one is written by
 * BOTH tasks: the NimBLE host task moves it from the auth path and from
 * `blehold`, the app task reads it on every ble_ota_tick(). A plain uint32_t
 * shared across two pinned cores is a data race even when each access is
 * word-sized - the compiler is entitled to cache or re-order it, and what it
 * guards here is the radio's lifetime, i.e. the battery. Default (seq_cst)
 * ordering: these are touched a handful of times per window, so the cheapest
 * correct thing is the strongest one. */
static std::atomic<uint32_t> hard_deadline{0};
static uint32_t up_since      = 0;
/* Progress watchdog. esp-ota-ble has no stall timeout of its own - it will sit
 * in an open transfer indefinitely if the host vanishes mid-stream - and an
 * open transfer blocks both sleep and the teardown below. */
static uint32_t staged_total  = 0;
static uint32_t stall_mark    = 0;
static uint32_t stall_at      = 0;
static bool     stall_armed   = false;

/* Status lines queued for notification. Produced on the NimBLE host task
 * (otaBleSubmitCommand reports synchronously) and on the app task; drained
 * only from ble_ota_tick().
 *
 * A FIXED BYTE RING, NOT A std::string UNDER A SPINLOCK. The first version
 * held portENTER_CRITICAL across std::string::append and ::assign, which
 * allocate - so an ordinary 500-byte notification could enter the heap
 * allocator, and take ITS locks, with preemption and interrupts suppressed.
 * That is unbounded critical-section latency on a core that is also running a
 * BLE controller, and a lock-order inversion waiting to happen. A plain buffer
 * under a FreeRTOS mutex has neither problem: nothing inside the lock can
 * block, and nothing inside it can allocate. */
static const size_t TX_CAP = 2048;
static uint8_t  txbuf[TX_CAP];
static size_t   tx_len = 0;
static SemaphoreHandle_t txlock = nullptr;
static uint32_t tx_dropped = 0;

/* Both producers can run before ble_ota_up() has finished, so the lock is
 * created once at first use rather than in ble_ota_up(). A failed create is
 * reported and then treated as "no lock": dropping status lines is worse than
 * a race on a byte buffer that only ever grows under one producer at a time. */
static void txlock_init(void)
{
    if (!txlock) txlock = xSemaphoreCreateMutex();
}
static bool tx_take(void)  { return txlock && xSemaphoreTake(txlock, portMAX_DELAY) == pdTRUE; }
static void tx_give(void)  { if (txlock) xSemaphoreGive(txlock); }

static void tx_append_line(const char *line)
{
    const size_t n = strlen(line);
    txlock_init();
    bool locked = tx_take();
    /* The line and its newline go in together, or neither does. With the cap
     * checked per call a line could be admitted and its newline refused,
     * splicing two protocol lines into one on the host's parser. */
    bool dropped = (tx_len + n + 1 > TX_CAP);
    if (!dropped) {
        memcpy(txbuf + tx_len, line, n);
        tx_len += n;
        txbuf[tx_len++] = '\n';
    } else {
        ++tx_dropped;
    }
    if (locked) tx_give();
    /* Printed OUTSIDE the lock, and never silently: a dropped status line is a
     * host that waits out a timeout instead of learning what went wrong. */
    if (dropped)
        Serial.printf("!! ble: status buffer full, dropped a line (%lu total)\n",
                      (unsigned long)tx_dropped);
}

static void ota_status_hook(OtaBleLevel level, const char *line)
{
    // Protocol replies go to their transport. Mirroring every BLE credit to
    // an unread USB console can block HWCDC for seconds inside the consumer.
    if (serial_driving.load(std::memory_order_acquire))
        Serial.printf("%s %s\n", level == OtaBleLevel::Info ? "ble:" : "!! ble:", line);
    else
        tx_append_line(line);
}

static void ota_quiesce_hook(bool halt) { flash_busy = halt; }

static void drain_tx(void);

static void ota_restart_hook(void)
{
    // Preserve the final erase/skip accounting and digest-success reply.
    if (!serial_driving.load(std::memory_order_acquire)) {
        drain_tx();
        delay(50);
    }
    node_prepare_restart();
    esp_restart();
}

/* END THE SESSION AND PUBLISH ITS CLEANUP, from either task, exactly once.
 *
 * This is the whole of the fix for review finding 3. The exchange inside
 * blesess_end() is a single read-modify-write, so of the two callers that can
 * race here - onDisconnect() on the NimBLE host task, ble_ota_down() on the
 * app task, the second of which invokes the first from inside
 * NimBLEDevice::deinit(true) - exactly one gets the live session word back and
 * exactly one gets zero. There is no interleaving in which both see "already
 * handled", which is precisely what the two-bool version permitted.
 *
 * GATED ON AUTHENTICATION, unchanged and deliberately: an unauthenticated peer
 * cannot write a Modbus register over BLE, so it can have armed nothing and
 * claimed nothing, and a disarm on its drop protects nothing it could have
 * caused. That gating was measured to matter - an unauthenticated client was
 * connecting and being dropped every ~20 s, and every drop disarmed the
 * membrane heater. What it is NOT is a lifetime guarantee for the arm: the USB
 * and LoRaWAN paths reach 0x0060 without ever touching this file. That gap is
 * now covered by the transport-agnostic arm lease in ctrl.h
 * (ctrl_arm_needs_disarm), of which this remains the fast path for the one
 * transport that can see a disconnect happen.
 *
 * MEASURED 2026-09-10, and it is why the gate is not merely defensible but
 * required: an unauthenticated client (fb:1a:b5:3e:70:81) was connecting and
 * being dropped for not authenticating every ~20 s, and every one of those
 * drops fired a disarm. The probe's heater could not stay energised for twenty
 * seconds - not for a bench calibration, and not in a compost pile, where the
 * ring is what keeps the membrane above the dew point. Any unauthenticated
 * radio in range could hold the heater off indefinitely, from outside the
 * authentication boundary entirely.
 *
 * The rest of the net is unchanged: the bench arm at 0x0060 is volatile and
 * dies with a probe reset, and a claim's own expiry still disarms when it
 * lapses. What is new since that note was written is that the arm no longer
 * depends on any of those - ctrl_arm_needs_disarm() bounds it whichever
 * transport armed it, including the two that cannot see a BLE disconnect at
 * all. */
/* SPLIT IN TWO, and only so that the ORDER inside onDisconnect() can be kept:
 * the session must read "down" to the other task from the callback's first
 * line, exactly as `have_conn = false` used to, while the cleanup is published
 * from its last one alongside the other flags the app task consumes. The
 * atomicity is untouched by the split - the exchange is the single decision
 * point, and the word it returns is a local that no other task can reach. */
static uint32_t sess_end(void)
{
    return blesess_end(&sess);
}

static void sess_publish_cleanup(uint32_t was)
{
    if (!blesess_word_conn(was)) return;      /* somebody already did it */
    if (blesess_word_authed(was)) {
        ble_disconnect_disarm.store(true, std::memory_order_release);
        ble_release_pending.store(true, std::memory_order_release);
    }
    /* A queued claim dies with the session that queued it - and ONLY that
     * session's claim. An unconditional clear here can erase a request a newer
     * session published after this one ended: ble_ota_down() can win the
     * session exchange on the app task while the host task accepts a new
     * connection and queues its claim before this line runs. The consumer's id
     * comparison still refuses an orphan; this stops a late cleanup destroying
     * a live request. Review 2026-09-10. */
    blesess_claim_cancel_for(&ble_claim, blesess_id(was));
}

static void sess_end_and_publish(void)
{
    sess_publish_cleanup(sess_end());
}

/* ---- GATT callbacks. Producer side only: copy, latch, never touch flash. -- */

class SrvCb : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *, NimBLEConnInfo &info) override {
        (void)blesess_connect(&sess);
        /* KEEP THE HANDLE. Disconnecting the auth-grace squatter needs the
         * real one - the bench peer came up as conn_handle=1, so the obvious
         * disconnect(0) would have hung up on nothing and left the window
         * squatted exactly as before. */
        conn_handle = info.getConnHandle();
        conn_at = millis();
        /* NAME THE PEER. Without it a squatter is anonymous, and the bench
         * spent two windows guessing which of several machines had connected
         * while the real client was still on the LoRaWAN half. */
        Serial.printf("ble: client connected: %s (handle %u)\n",
                      info.getAddress().toString().c_str(),
                      (unsigned)conn_handle);

        // Negotiate after authentication/initial GATT setup, from the app task.
        link_tune_stage.store(1, std::memory_order_release);
    }

    /* Report what the central ACTUALLY granted - the request above is only a
     * request, and a silent downgrade would otherwise look like the fix
     * working when it had been refused. */
    void onConnParamsUpdate(NimBLEConnInfo &info) override {
        link_params_seen.store(true, std::memory_order_release);
        Serial.printf("ble: conn params interval=%u latency=%u timeout=%u\n",
                      (unsigned)info.getConnInterval(),
                      (unsigned)info.getConnLatency(),
                      (unsigned)info.getConnTimeout());
    }
    void onPhyUpdate(NimBLEConnInfo &, uint8_t tx, uint8_t rx) override {
        Serial.printf("ble: negotiated PHY tx=%u rx=%u\n", (unsigned)tx, (unsigned)rx);
    }
    void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int reason) override {
#if PROTO_BLE_BENCH
        benchRamClear();
        memset(benchDle, 0, sizeof(benchDle));
#endif
        link_tune_stage.store(0, std::memory_order_release);
        /* THE SESSION ENDS ON THE FIRST LINE, as `have_conn = false` did: the
         * app task must not go on believing a peer is connected while this
         * callback is still running (it can spend a Serial.printf in here).
         * The cleanup this returns is published at the bottom. */
        const uint32_t was = sess_end();
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        peer_mtu = 23;
        /* Per esp-ota-ble's contract, and a no-op when nothing is in flight.
         * A client that drops mid-stream must not leave the OTA handle open:
         * the next `begin` would be refused and the node would hold both the
         * radio and the sleep inhibit until the hard deadline. */
        otaBleRequestAbort();
        /* Authentication does not survive a reconnect. The window is bound to
         * a CTRL session, not to a peer, so a second client arriving inside
         * the same window has to quote the nonce too.
         *
         * ENDING THE SESSION AND DECIDING ITS CLEANUP ARE ONE STEP, and they
         * have to be: doing them as two - clear the connection, then read the
         * authentication - IS the defect. sess_end() above did both, in one
         * exchange, and `was` is the answer. */
        /* AND THE BUS CHANNEL'S REASSEMBLY. ATT writes are ordered and
         * reliable within a connection and guarantee nothing across one, so a
         * link that dropped mid-frame leaves bytes that no later write is a
         * continuation of. Leaving them would let the next connection's first
         * chunks complete the dead frame's length - the splice, arriving by a
         * route the framing layer cannot see. Flagged rather than cleared
         * here: bus_rx is the host task's and this IS the host task, but the
         * flag also survives the case where a write is already queued. */
        bus_link_broke.store(true, std::memory_order_release);
        /* Flag queue flush for the app task in bus_service() rather than mutating
         * bus_q_head directly here on the NimBLE host task, preventing underflow
         * races if Core 1 is mid-exchange. */
        bus_flush_pending.store(true, std::memory_order_release);
        /* A CCCD does not survive an unbonded connection, so the next peer
         * starts unsubscribed whatever this one had enabled. Clearing it here
         * is what stops the app task polling the bus every few seconds for a
         * listener that is gone - the arming lease alone would not, since it is
         * measured in minutes. */
        telem_subscribed.store(false, std::memory_order_release);
        /* PUBLISH THE CLEANUP THE SESSION-ENDING EXCHANGE DECIDED.
         *
         * One atomic exchange decided both whether this callback owns the
         * cleanup and what it consists of - see sess_end() and
         * sess_publish_cleanup(), which carry the reasoning, the review
         * history and the measured evidence for the authentication gate. ble_ota_down() ends the session the
         * same way, and exactly one of the two gets the live word. */
        sess_publish_cleanup(was);
        Serial.printf("ble: client disconnected (reason %d)\n", reason);
        /* NOT during teardown - see the note on `teardown`. deinit() drops the
         * link itself, so this callback runs on the way out of the stack. */
        if (up && !teardown) NimBLEDevice::startAdvertising();
    }
    void onMTUChange(uint16_t mtu, NimBLEConnInfo &) override {
        peer_mtu = mtu;
        Serial.printf("ble: MTU %u\n", (unsigned)mtu);
    }
} srv_cb;

class CtrlCb : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) override {
        std::string v = c->getValue();
        while (!v.empty() && (v.back() == '\n' || v.back() == '\r')) v.pop_back();
        if (v.empty()) return;

        if (v.compare(0, 5, "auth ") == 0) {
            /* Compare as a NUMBER, not as text: a host that formats the nonce
             * "0x0a3f" or "A3F" is quoting the right secret and should not be
             * failed for spelling. strtoul with base 16 accepts all of those,
             * and the end-pointer check rejects trailing junk that would
             * otherwise be silently ignored. */
            const char *p = v.c_str() + 5;
            char *end = nullptr;
            unsigned long got = strtoul(p, &end, 16);
            bool ok = (end && end != p && *end == '\0' && got == auth_nonce);
            if (ok && blesess_auth(&sess)) {
                /* blesess_auth() REFUSES if the connection has already gone -
                 * an `auth` write racing its own disconnect must not set AUTH
                 * on a session whose teardown cleanup has already run, or the
                 * cleanup owes a disarm nobody will publish. */
                auth_fails = 0;
                tx_append_line("OTAB AUTH ok");
                Serial.println("ble: authenticated");
            } else if (!ok && ++auth_fails >= BLE_OTA_AUTH_TRIES) {
                /* Take the radio down rather than merely refusing. A 16-bit
                 * nonce is a gate, not a cipher, and the only thing that makes
                 * it one is that guessing costs the attacker the window. The
                 * teardown itself must not run here - this is the host task -
                 * so expire the deadline and let ble_ota_tick() do it. */
                tx_append_line("OTAB FAIL auth attempts exhausted");
                hard_deadline = millis() - 1;
                Serial.println("!! ble: auth attempts exhausted, taking the radio down");
            } else if (!ok) {
                tx_append_line("OTAB FAIL auth bad nonce");
            } else {
                /* Correct nonce, session already gone. Say so rather than
                 * report a bad nonce - the host would retry the wrong thing. */
                tx_append_line("OTAB FAIL auth session gone");
            }
            return;
        }

        /* `abort` is allowed unauthenticated, and that is a TRADE, not a
         * free win. An earlier comment here said it "can only ever make the
         * node safer", which is false at the protocol level: an unauthenticated
         * peer that can reach this can terminate a legitimate operator's
         * transfer, repeatedly. What it cannot do is move the boot slot or
         * write a byte of flash, so the worst case is denial of service inside
         * a window an operator opened, against an operator who is standing
         * there and can see it fail.
         *
         * The other side of the trade is that a host which lost its auth state
         * - any reconnect clears it - could not otherwise clean up after
         * itself, and would leave a transfer open until the stall watchdog
         * fires. Given only one peer is expected on this link at a time, the
         * cleanup is worth more than the DoS costs. Bind auth to the
         * connection handle and this can be tightened. */
        if (!authed() && v != "abort") {
            tx_append_line("OTAB FAIL auth required");
            return;
        }

        /* ---- node verbs, handled here and NOT passed to the OTA receiver ---
         *
         * These are the CTRL commands made reachable over BLE. The point is to
         * remove the round trip: a downlink only lands in the window an uplink
         * opened, so over LoRaWAN every one of these costs up to a full poll
         * interval - and a shorter poll interval is the very thing `cadence`
         * exists to ask for. An operator already connected should not have to
         * wait five minutes to say "go faster".
         *
         * THIS GRANTS NO AUTHORITY THE LoRaWAN PATH DOES NOT HAVE. Reaching
         * here requires the radio to be up, which only an accepted CTRL
         * SESSION_OPEN does, plus `auth <nonce>` against that session's nonce.
         * The clamps are the same clamps, applied in the same place. */
        last_act_ms = millis();

#if PROTO_BLE_BENCH
        if (v == "ramabort") {
            benchRamClear(); tx_append_line("BENCH RAM aborted"); return;
        }
        if (v == "ramend") {
            if (!benchRam || benchRamBad || benchRamGot != benchRamWant) {
                tx_append_line("BENCH RAM FAIL incomplete or overflow");
            } else {
                unsigned char digest[32]; char hex[65], line[160];
                if (mbedtls_sha256(benchRam, benchRamGot, digest, 0) != 0) {
                    tx_append_line("BENCH RAM FAIL hash");
                } else {
                    for (unsigned i=0;i<32;i++) snprintf(hex+2*i,3,"%02x",digest[i]);
                    snprintf(line,sizeof(line),"BENCH RAM bytes=%u us=%lld sha=%s",
                        (unsigned)benchRamGot,(long long)(benchRamLast-benchRamFirst),hex);
                    tx_append_line(line);
                }
            }
            benchRamClear(); return;
        }
        if (benchRam) { tx_append_line("BENCH RAM FAIL busy"); return; }
        if (v.compare(0,4,"ram ")==0) {
            char *end=nullptr; const char *p=v.c_str()+4;
            unsigned long n=strtoul(p,&end,10);
            if (otaBleActive() || end==p || *end || n==0 || n>6*1024*1024) {
                tx_append_line("BENCH RAM FAIL size or OTA busy"); return;
            }
            benchRam=(uint8_t*)heap_caps_malloc(n,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
            if (!benchRam) { tx_append_line("BENCH RAM FAIL allocation"); return; }
            benchRamWant=n; benchRamGot=0; benchRamBad=false;
            benchRamFirst=benchRamLast=0;
            tx_append_line("BENCH RAM ready"); return;
        }
        if (v == "buffers") {
            char line[100];
            snprintf(line,sizeof(line),"BENCH BUFFERS msys1=%u size=%u msys2=%u acl=%u",
                (unsigned)MYNEWT_VAL(MSYS_1_BLOCK_COUNT),
                (unsigned)MYNEWT_VAL(MSYS_1_BLOCK_SIZE),
                (unsigned)MYNEWT_VAL(MSYS_2_BLOCK_COUNT),
                (unsigned)MYNEWT_VAL(BLE_TRANSPORT_ACL_FROM_LL_COUNT));
            tx_append_line(line); return;
        }
        if (v == "dleinfo") {
            char line[100];
            snprintf(line,sizeof(line),"BENCH DLE tx=%u tx_us=%u rx=%u rx_us=%u",
                benchDle[0],benchDle[1],benchDle[2],benchDle[3]);
            tx_append_line(line); return;
        }
        if (v == "dle2") {
            int rc=otaBleActive() ? -1 : ble_gap_set_data_len(conn_handle,251,1060);
            char line[80]; snprintf(line,sizeof(line),"BENCH DLE2 request_rc=%d",rc);
            tx_append_line(line); return;
        }
        // Bench controls: negotiate one parameter at a time before timing OTA.
        if (v == "link") {
            uint8_t tx = 0, rx = 0;
            char line[100];
            const bool known = srv->getPhy(conn_handle, &tx, &rx);
            snprintf(line, sizeof(line), "BENCH LINK interval=%u mtu=%u phy_known=%u tx=%u rx=%u",
                     (unsigned)info.getConnInterval(), (unsigned)info.getMTU(),
                     (unsigned)known, (unsigned)tx, (unsigned)rx);
            tx_append_line(line);
            return;
        }
        if (v.compare(0, 9, "interval ") == 0) {
            char *end = nullptr;
            const unsigned long n = strtoul(v.c_str() + 9, &end, 10);
            if (!end || *end || n < 6 || n > 104 || otaBleActive()) {
                tx_append_line("OTAB FAIL interval needs 6..104 while idle");
            } else {
                srv->updateConnParams(conn_handle, n, n, 0, 72);
                tx_append_line("BENCH interval requested; inspect link after settling");
            }
            return;
        }
        if (v == "phy1" || v == "phy2") {
            const uint8_t mask = v == "phy2" ? BLE_GAP_LE_PHY_2M_MASK : BLE_GAP_LE_PHY_1M_MASK;
            const bool ok = !otaBleActive() && srv->updatePhy(conn_handle, mask, mask, 0);
            tx_append_line(ok ? "BENCH PHY requested; inspect link after settling" : "OTAB FAIL PHY request");
            return;
        }
        if (v == "dle") {
            if (otaBleActive()) tx_append_line("OTAB FAIL dle while active");
            else { srv->setDataLen(conn_handle, 251); tx_append_line("BENCH DLE requested"); }
            return;
        }
#endif

        /* `caps` - src/caps.h. Read-only, but after the auth gate like every
         * node verb: an unauthenticated peer squatting the window is not owed
         * a description of the image. */
        if (v == "caps") {
            char line[160];
            char out[168];
            unsigned n = 0;
            for (unsigned i = 0; caps_line(CAPS_BLE, i, line, sizeof(line)); i++, n++) {
                snprintf(out, sizeof(out), "CAPS %s", line);
                tx_append_line(out);
            }
            snprintf(out, sizeof(out), "CAPS end lines=%u", n + 1u);
            tx_append_line(out);
            return;
        }

        if (v.compare(0, 8, "cadence ") == 0) {
            /* Parsed here, APPLIED on the app task - see the latch. Refusing a
             * second pending request rather than overwriting it means the
             * operator's first command is the one that takes effect and the
             * second is reported refused, instead of one of the two silently
             * vanishing. */
            if (cad_pending.load(std::memory_order_acquire)) {
                tx_append_line("OTAB FAIL cadence request already pending");
                return;
            }
            const char *p = v.c_str() + 8;
            char *end = nullptr;
            unsigned long secs = strtoul(p, &end, 10);
            if (!end || end == p) {
                tx_append_line("OTAB FAIL cadence wants <seconds> [lease_min]");
                return;
            }
            unsigned long lease = 0;
            if (*end != '\0') {
                const char *q = end;
                lease = strtoul(q, &end, 10);
                if (!end || end == q || *end != '\0') {
                    tx_append_line("OTAB FAIL cadence wants <seconds> [lease_min]");
                    return;
                }
            }
            if (lease == 0) lease = CADENCE_MAX_LEASE_MIN;
            if (secs > 0xFFFFUL)  secs = 0xFFFFUL;
            if (lease > 0xFFFFUL) lease = 0xFFFFUL;
            cad_req_s     = (uint16_t)secs;
            cad_req_lease = (uint16_t)lease;
            /* RELEASE: everything written above is visible to whoever observes
             * this store. Must stay last. */
            cad_pending.store(true, std::memory_order_release);
            return;
        }

        if (v == "reset" || v == "reboot") {
            /* Reachable only after `auth` - this sits below the auth gate, so
             * it answers OTAB FAIL auth required like every other verb. A
             * reboot primitive that did not would be a denial-of-service any
             * passer-by could fire at a node holding a BLE window open.
             *
             * The reply goes out BEFORE the latch is set, and ble_ota_tick()
             * drains the TX queue and waits before it calls esp_restart():
             * a client that gets no acknowledgement cannot tell a reset that
             * happened from a command that was never received. */
            tx_append_line("OTAB RESET rebooting");
            reset_pending.store(true, std::memory_order_release);
            Serial.println("ble: reset requested over BLE");
            return;
        }

        if (v.compare(0, 6, "telem ") == 0) {
            /* FAST TELEMETRY OVER BLE. Same latch shape as `cadence` above,
             * and the same refusal rather than overwrite: the operator's first
             * command is the one that takes effect, and the second is reported
             * refused instead of one of the two silently vanishing.
             *
             * `telem 0` disarms and is always granted. `telem <s> [min]` arms
             * at <s> seconds for [min] minutes, defaulting to the maximum
             * lease - the lease is what stops a fast cadence surviving the
             * operator who armed it, and a default of "as long as allowed" is
             * still bounded by the BLE window's own ceilings. */
            if (telem_pending.load(std::memory_order_acquire)) {
                tx_append_line("OTAB FAIL telem request already pending");
                return;
            }
            const char *p = v.c_str() + 6;
            char *end = nullptr;
            unsigned long secs = strtoul(p, &end, 10);
            if (!end || end == p) {
                tx_append_line("OTAB FAIL telem wants <seconds> [lease_min]");
                return;
            }
            unsigned long lease = 0;
            if (*end != '\0') {
                const char *q = end;
                lease = strtoul(q, &end, 10);
                if (!end || end == q || *end != '\0') {
                    tx_append_line("OTAB FAIL telem wants <seconds> [lease_min]");
                    return;
                }
            }
            if (lease == 0) lease = TELEM_MAX_LEASE_MIN;
            if (secs > 0xFFFFUL)  secs = 0xFFFFUL;
            if (lease > 0xFFFFUL) lease = 0xFFFFUL;
            telem_req_s     = (uint16_t)secs;
            telem_req_lease = (uint16_t)lease;
            /* RELEASE: must stay last, see cad_pending. */
            telem_pending.store(true, std::memory_order_release);
            return;
        }

        if (v.compare(0, 8, "blehold ") == 0) {
            const char *p = v.c_str() + 8;
            char *end = nullptr;
            unsigned long mins = strtoul(p, &end, 10);
            if (!end || end == p || *end != '\0' || mins == 0) {
                tx_append_line("OTAB FAIL blehold wants <minutes>");
                return;
            }
            if (mins > 0xFFFFUL) mins = 0xFFFFUL;
            uint16_t got = ble_ota_hold_minutes((uint16_t)mins);
            if (got) {
                char line[48];
                snprintf(line, sizeof(line), "OTAB HOLD %u min", (unsigned)got);
                tx_append_line(line);
            }
            return;
        }

        if (v.compare(0, 6, "claim ") == 0 || v == "claim") {
            if (blesess_claim_pending(&ble_claim)) {
                tx_append_line("BUS FAIL claim request already pending");
                return;
            }
            unsigned long mins = 15;
            if (v.length() > 6) {
                const char *p = v.c_str() + 6;
                char *end = nullptr;
                mins = strtoul(p, &end, 10);
                if (!end || end == p || *end != '\0' || mins == 0) {
                    tx_append_line("BUS FAIL claim wants [minutes]");
                    return;
                }
            }
            if (mins > 0xFFUL) mins = 0xFFUL;
            /* STAMPED WITH THE SESSION THAT ASKED. The consumer runs on the
             * app task an arbitrary time later and compares the two, so a
             * claim outliving its requester is refused rather than installed
             * for nobody - review 2026-09-10, finding 4. */
            if (!blesess_claim_queue(&ble_claim, &sess, (uint16_t)mins))
                tx_append_line("BUS FAIL claim not queued");
            return;
        }

        if (v == "unclaim" || v == "release") {
            ble_release_pending.store(true, std::memory_order_release);
            return;
        }

        if (otaBleActive() && serial_driving.load(std::memory_order_acquire)) {
            tx_append_line("OTAB FAIL serial transfer owns receiver");
            return;
        }
        if (otaBleSubmitCommand(v.c_str()) == OtaBleSubmit::Rejected) {
            /* Announced on the status channel too: the receiver reports
             * synchronous rejections through its return value, not through the
             * status hook, so without this the host sees silence. */
            tx_append_line("OTAB FAIL command rejected");
        } else {
            serial_driving.store(false, std::memory_order_release);
        }
    }
} ctrl_cb;

/* Host task. Accumulate until the length prefix is satisfied, then publish
 * exactly one request to the app task.
 *
 * A malformed or oversized length RESETS the accumulator rather than clamping
 * it. Clamping would splice the tail of one frame onto the head of the next
 * and hand the app task an ADU that passes CRC by accident far less often
 * than it corrupts a register write - and a register write is what this
 * channel exists to carry. */
class BusCb : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &) override {
        if (!authed()) return;          /* silent, as DataCb is */
        std::string v = c->getValue();
        if (v.empty()) return;

        /* THE TWO DISCONTINUITIES, both from outside the byte stream because
         * nothing inside it can tell a resumed frame from a spliced one. */
        uint32_t now = millis();
        bool broke = bus_link_broke.exchange(false, std::memory_order_acq_rel);
        bool stale = (bus_rx_len != 0u) &&
                     ((uint32_t)(now - bus_rx_at) > BLE_BUS_REASM_MS);
        if (stale) {
            tx_append_line("BUS FAIL partial frame timed out - discarded");
        }
        bus_rx_at = now;

        size_t want = 0;
        blebus_step_t st = blebus_accumulate(bus_rx, sizeof(bus_rx), &bus_rx_len,
                                             (const uint8_t *)v.data(), v.size(),
                                             BUS_MAX, broke || stale, &want);
        switch (st) {
        case BLEBUS_NEED_MORE:
            return;
        case BLEBUS_ERR_OVERRUN:
            tx_append_line("BUS FAIL frame overruns the reassembly buffer");
            return;
        case BLEBUS_ERR_LEN:
            tx_append_line("BUS FAIL implausible frame length");
            return;
        case BLEBUS_ERR_TRAILING:
            /* A host that pipelined has lost track of which reply belongs to
             * which request. Say so, rather than serving the first frame and
             * silently dropping the rest. */
            tx_append_line("BUS FAIL trailing bytes - one request at a time");
            return;
        case BLEBUS_FRAME:
            break;
        }

        if (bus_q_count() >= BUS_Q_DEPTH) {
            /* Two are already outstanding and ATT will not overlap a third
             * write anyway - this is the host running ahead of its own
             * replies, not a transport problem. */
            tx_append_line("BUS FAIL queue full");
            return;
        }
        const uint8_t t = bus_q_tail.load(std::memory_order_relaxed);
        const uint8_t slot = (uint8_t)(t % BUS_Q_DEPTH);
        memcpy(bus_q[slot], bus_rx + 2, want);
        bus_q_len[slot] = want;
        bus_last_ms = now;
        /* RELEASE: everything written above is visible to whoever acquires
         * the tail. The memcpy must not be seen after the announcement. */
        bus_q_tail.store((uint8_t)(t + 1), std::memory_order_release);
    }
} bus_cb;

/* Notify a length-prefixed frame, chunked to the negotiated MTU. A zero
 * length is a real answer here and means the bus said nothing - distinct from
 * no notification at all, which means the node never got the request. */
static void bus_notify(const uint8_t *adu, size_t len)
{
    if (!chrBus || !have_conn()) return;
    uint8_t out[BUS_MAX + 2];
    out[0] = (uint8_t)(len >> 8);
    out[1] = (uint8_t)(len & 0xFF);
    if (len) memcpy(out + 2, adu, len);
    size_t total = len + 2;
    size_t chunk = peer_mtu > 3 ? (size_t)(peer_mtu - 3) : 20;
    for (size_t off = 0; off < total; ) {
        size_t n = total - off < chunk ? total - off : chunk;
        /* A failed notify is an empty mbuf pool, not a dead link. Stop: the
         * host's own timeout is the recovery, and looping here would drain
         * the pool and drop the rest of the frame silently. */
        if (!chrBus->notify(out + off, n)) {
            /* A truncated reply is WORSE than none: the host's reassembler is
             * left holding a partial frame whose declared length the NEXT
             * exchange's reply would complete - the same splice, in the reply
             * direction. It cannot be finished (the mbuf pool is empty, which
             * is why this failed), so say so on the channel that is still
             * working and let the host discard rather than wait. */
            Serial.printf("!! ble: bus notify failed at %u/%u - reply "
                          "abandoned\n", (unsigned)off, (unsigned)total);
            tx_append_line("BUS FAIL reply truncated - discard the partial "
                           "frame");
            return;
        }
        off += n;
    }
}

/* App task. One exchange, one reply. */
static void bus_service(void)
{
    if (bus_flush_pending.exchange(false, std::memory_order_acq_rel)) {
        bus_q_head.store(bus_q_tail.load(std::memory_order_acquire),
                         std::memory_order_release);
    }
    if (bus_q_count() == 0) return;

    /* Flash erase disables the CPU cache and stalls the other core, so a
     * Modbus exchange started inside it would time out on a healthy bus.
     * Leave the request pending and come back - the host is waiting on a
     * reply, not on a tick. */
    if (flash_busy) return;

    uint8_t rsp[BUS_MAX];
    size_t  rlen = 0;

    if (bus_exchange == nullptr) {
        tx_append_line("BUS FAIL this build does not drive the RS-485 pads");
    } else if (bus_is_claimed != nullptr && !bus_is_claimed()) {
        /* Refused, not served into a race. See bleota.h: the poller would
         * otherwise insert a transaction between two updater frames, and this
         * firmware cannot see that a run of exchanges is one transaction. */
        tx_append_line("BUS FAIL bus not claimed - open the session with "
                       "CTRL_F_CLAIM_BUS");
    } else {
        const uint8_t h = bus_q_head.load(std::memory_order_relaxed);
        const uint8_t slot = (uint8_t)(h % BUS_Q_DEPTH);
        rlen = bus_exchange(bus_q[slot], bus_q_len[slot], rsp, sizeof(rsp));
        if (rlen == 0) tx_append_line("BUS silence");
    }

    bus_last_ms = millis();
    /* RELEASE again on the way back: the host task may write this slot the
     * instant it observes the head move, and must not do so before rsp is
     * built out of it. */
    bus_q_head.store((uint8_t)(bus_q_head.load(std::memory_order_relaxed) + 1),
                     std::memory_order_release);

    /* A FAILURE DROPS WHATEVER WAS QUEUED BEHIND IT. The host sent those
     * speculatively, before it could know this one failed, and a run of
     * exchanges is one transaction the node cannot see the shape of - serving
     * request N+1 into a probe that has just refused N is the node deciding
     * something only the host can. Say how many were dropped so the host
     * resends rather than guesses.
     *
     * A host that still waits for each reply never has anything queued, so
     * this changes nothing for it. */
    if (rlen == 0) {
        uint8_t dropped = bus_q_count();
        if (dropped) {
            bus_q_head.store(bus_q_tail.load(std::memory_order_acquire),
                             std::memory_order_release);
            char line[48];
            snprintf(line, sizeof(line), "BUS FLUSH %u queued dropped",
                     (unsigned)dropped);
            tx_append_line(line);
        }
    }
    bus_notify(rsp, rlen);
}

void ble_bus_set_exchange(ble_bus_exchange_fn exchange,
                          bool (*bus_claimed)(void))
{
    bus_exchange = exchange;
    bus_is_claimed = bus_claimed;
}

bool ble_bus_active(void)
{
    if (!up) return false;
    if (bus_q_count() != 0) return true;
    if (bus_last_ms == 0) return false;
    return (int32_t)(millis() - bus_last_ms - BLE_BUS_IDLE_MS) < 0;
}

/* --- fast telemetry ------------------------------------------------------ */

class TelemCb : public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic *, NimBLEConnInfo &,
                     uint16_t subValue) override {
        /* Bit 0 is notifications, bit 1 is indications. This characteristic
         * only offers NOTIFY, so bit 0 is the whole answer - but test the bit
         * rather than the word, because a host that enables indications on a
         * characteristic that has none would otherwise read as subscribed and
         * be sent frames it never asked for and cannot receive. */
        bool on = (subValue & 0x0001u) != 0u;
        telem_subscribed.store(on, std::memory_order_release);
        /* NOT counted as activity. Subscribing is one write from a host that
         * may then go silent for the rest of the window, and treating it as
         * activity would let it buy BLE_OTA_CONN_GRACE_MS. What keeps a
         * telemetry window alive is telem_is_armed() plus this flag, both
         * bounded by BLE_TELEM_MAX_TOTAL_MS - see ble_ota_tick(). */
        Serial.printf("ble: telemetry %s\n", on ? "subscribed" : "unsubscribed");
    }
} telem_cb;

/* Notify one telemetry frame, length-prefixed and chunked to the negotiated
 * MTU. Returns true only when the WHOLE frame went out.
 *
 * The return value is load-bearing: telem_sent() is stamped from it, so a
 * frame the controller could not take is retried on the next service call
 * rather than counted as delivered. A truncated frame is worse than a missing
 * one - the host's reassembler would be left holding a partial frame that the
 * NEXT frame's bytes would complete, splicing two measurements into one
 * plausible-looking record - so a partial send says so on the control channel
 * and lets the host discard.
 *
 * MEASURED (esp-ota-ble host notes, farmgw<->node): the negotiated MTU on this
 * link is 517, so a ~90-byte frame is one notification and the chunking below
 * is a guard rather than a normal path. It is still here because the MTU is
 * whatever the peer negotiated, and at the 23-byte default it would be five. */
static bool telem_notify(const uint8_t *frame, size_t len)
{
    if (!chrTelem || !have_conn()) return false;
    if (!telem_subscribed.load(std::memory_order_acquire)) return false;
    if (len > BUS_MAX) return false;          /* cannot happen: caller caps it */

    uint8_t out[BUS_MAX + 2];
    out[0] = (uint8_t)(len >> 8);
    out[1] = (uint8_t)(len & 0xFF);
    if (len) memcpy(out + 2, frame, len);
    size_t total = len + 2;
    size_t chunk = peer_mtu > 3 ? (size_t)(peer_mtu - 3) : 20;
    for (size_t off = 0; off < total; ) {
        size_t n = total - off < chunk ? total - off : chunk;
        if (!chrTelem->notify(out + off, n)) {
            /* An empty mbuf pool, not a dead link - the same failure bus_notify
             * documents. Looping would drain the pool and drop the rest
             * silently. */
            if (off != 0)
                tx_append_line("TELEM FAIL frame truncated - discard the "
                               "partial frame");
            return false;
        }
        off += n;
    }
    return true;
}

bool ble_telem_send(const uint8_t *frame, size_t len)
{
    if (!up || frame == nullptr || len == 0) return false;
    return telem_notify(frame, len);
}

bool ble_telem_subscribed(void)
{
    /* Same authentication gate as telem_holding_window(), and for the same
     * reason: this is the predicate main.cpp's emitter tests before it polls
     * the bus, so leaving `authed` out here would have reopened the hole the
     * other one closes. Two call sites, one rule. */
    return up && have_conn() && authed() &&
           telem_subscribed.load(std::memory_order_acquire);
}

void ble_telem_set_armed_hook(bool (*armed)(void))
{
    telem_is_armed = armed;
}

/* True while the fast path is the reason the radio is still up: armed on the
 * app task AND a peer actually listening. Both halves matter - an armed lease
 * with nobody subscribed is a session whose host has gone, and it must not
 * hold the window open. */
static bool telem_holding_window(void)
{
    if (!up || !have_conn()) return false;
    /* AUTHENTICATED, not merely subscribed. Review 2026-09-09: an arming
     * OUTLIVES the connection that created it, while `authed` and the CCCD do
     * not. So an authenticated operator could arm a stream and disconnect, and
     * the next peer to connect - unauthenticated, because a CCCD write is not
     * gated by `auth` - would be enough to make the node poll the RS-485 bus
     * and notify the result. That contradicted this module's own stated
     * invariant, that a listener "cannot cause the node to transmit; it can
     * only hear what an authenticated operator has already asked for".
     *
     * Requiring `authed` restores it exactly: an unauthenticated peer may still
     * connect and subscribe, and will simply receive nothing. */
    if (!authed()) return false;
    if (telem_is_armed == nullptr || !telem_is_armed()) return false;
    return telem_subscribed.load(std::memory_order_acquire);
}

bool ble_telem_active(void) { return telem_holding_window(); }

bool ble_take_telem_request(uint16_t *seconds, uint16_t *lease_min)
{
    if (!seconds || !lease_min) return false;
    /* ACQUIRE: pairs with the release store in CtrlCb::onWrite(). */
    if (!telem_pending.load(std::memory_order_acquire)) return false;
    *seconds   = telem_req_s;
    *lease_min = telem_req_lease;
    telem_pending.store(false, std::memory_order_release);
    return true;
}

void ble_report_telem(uint16_t seconds, uint16_t lease_min, bool clamped)
{
    char line[72];
    if (seconds == 0)
        snprintf(line, sizeof(line), "OTAB TELEM off");
    else
        snprintf(line, sizeof(line), "OTAB TELEM %u s lease %u min%s",
                 (unsigned)seconds, (unsigned)lease_min,
                 clamped ? " (clamped)" : "");
    tx_append_line(line);
}

class DataCb : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &) override {
        if (!authed()) return;     /* silent: this channel has no reply path */
        std::string v = c->getValue();
#if PROTO_BLE_BENCH
        if (benchRam) {
            last_act_ms=millis();
            if (benchRamBad || v.empty() || v.size()>benchRamWant-benchRamGot) {
                benchRamBad=true; return;
            }
            if (!benchRamGot) benchRamFirst=esp_timer_get_time();
            memcpy(benchRam+benchRamGot,v.data(),v.size());
            benchRamGot+=v.size(); benchRamLast=esp_timer_get_time();
            return;
        }
#endif
        otaBleStageBytes((const uint8_t *)v.data(), v.size());
        staged_total += v.size();
    }
} data_cb;

/* ------------------------------------------------------------------------- */

static void drain_tx(void)
{
    /* Consumer task only, so a function-local static is safe and keeps 517
     * bytes off the stack of a task that also runs the LoRaWAN stack. */
    static uint8_t scratch[517];
    if (!chrCtrl || !have_conn()) return;
    size_t chunk = peer_mtu > 3 ? (size_t)(peer_mtu - 3) : 20;
    if (chunk > sizeof(scratch)) chunk = sizeof(scratch);
    txlock_init();
    for (;;) {
        size_t n;
        bool locked = tx_take();
        n = tx_len < chunk ? tx_len : chunk;
        if (n) memcpy(scratch, txbuf, n);
        if (locked) tx_give();
        if (!n) return;

        /* Notify OUTSIDE the lock - it can block on the controller - and only
         * consume the bytes once they have actually gone. A failed notify
         * means NimBLE's mbuf pool is empty; looping on it exhausts the pool
         * and silently drops packets, so stop and pick up on the next tick
         * with the bytes still queued. Peeking rather than popping is what
         * makes that requeue free, and removes the insert-at-front the string
         * version needed. */
        if (!chrCtrl->notify(scratch, n)) return;

        locked = tx_take();
        /* Re-clamp: a producer may have appended while the notify was in
         * flight, but nothing ever removes bytes except this drain, so the
         * first `n` bytes are still the ones just sent. */
        if (n > tx_len) n = tx_len;
        memmove(txbuf, txbuf + n, tx_len - n);
        tx_len -= n;
        if (locked) tx_give();
    }
}

void ble_ota_up(uint16_t nonce, uint32_t hold_ms)
{
    if (hold_ms > BLE_OTA_MAX_UP_MS) hold_ms = BLE_OTA_MAX_UP_MS;

    auth_nonce = nonce;

    /* The cumulative bound, CHECKED BEFORE THE EXTENSION IS WRITTEN.
     *
     * REVIEW FINDING 2026-09-08, and the comment here used to claim exactly
     * what the code did not do. `hard_deadline = millis() + hold_ms` ran
     * FIRST, and the exhausted branch then returned WITHOUT undoing it - so
     * the refusal still moved the deadline it was refusing to move. Since
     * ble_ota_tick() had no cumulative check of its own and returns while the
     * hard deadline is future, repeated renewals with fresh nonces (one per
     * scheduled uplink, forever) held the radio up indefinitely, and
     * maybe_sleep() refuses to sleep while BLE is up. That is a flat battery
     * in a pile, produced by a bound that read as if it were enforced. */
    if (up && (int32_t)(up_since + BLE_OTA_MAX_TOTAL_MS - millis()) <= 0) {
        tx_append_line("OTAB FAIL window total exhausted");
        Serial.println("!! ble: total window exhausted - refusing to extend");
        return;
    }

    hard_deadline = millis() + hold_ms;

    /* And never past the cumulative bound, so a long grant cannot step over it
     * either. Clamping rather than refusing: the caller asked for a window it
     * is entitled to, it is simply shorter than it hoped. */
    if (up && (int32_t)(hard_deadline - (up_since + BLE_OTA_MAX_TOTAL_MS)) > 0)
        hard_deadline = up_since + BLE_OTA_MAX_TOTAL_MS;

    if (up) {
        /* RENEWAL DOES NOT REVOKE AN EXISTING AUTHENTICATION, and getting this
         * wrong breaks the exact case renewal exists for.
         *
         * The BLE ceiling (BLE_OTA_MAX_UP_MS, 15 min) is deliberately shorter
         * than the CTRL ceiling (120 min), so a long push is kept alive by
         * renewals - which arrive on the node's own 5-minute uplink cadence,
         * i.e. DURING the transfer. An earlier version cleared `authed` here,
         * on the tidy-sounding reasoning that a new invitation deserves a new
         * proof. The effect would have been that the next renewal silently
         * dropped every firmware byte the connected peer sent (DataCb returns
         * early when unauthenticated), and the push would have died with no
         * error anywhere - the host streaming into a node discarding it.
         *
         * The peer has already proved it holds the LoRaWAN application key,
         * and a renewal is the same operator extending the same window, not a
         * new party arriving. A DISCONNECT still clears `authed`, which is
         * what keeps the proof bound to one link.
         *
         * `auth_fails` is deliberately NOT reset either: it counts per
         * radio-up period, so renewing cannot be used to buy five more
         * guesses at a 16-bit nonce. */
        auth_nonce = nonce;
        Serial.printf("ble: window extended, %lu ms, nonce %04X%s\n",
                      (unsigned long)hold_ms, (unsigned)nonce,
                      authed() ? " (peer stays authenticated)" : "");
        return;
    }

    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_BT);
    char name[24];
    snprintf(name, sizeof(name), "farmnode-%02X%02X%02X", mac[3], mac[4], mac[5]);

    /* EVERY PIECE OF SHARED STATE BEFORE THE STACK EXISTS. These used to be
     * assigned after adv->start(), which is after the point a client can
     * connect and write - so a fast peer's `auth` could set `authed` and have
     * it cleared a microsecond later by this function's own initialisation.
     * Nothing here can be reached until NimBLEDevice::init() returns, so doing
     * it first removes the window rather than narrowing it. */
    /* No session, no latched request, before the stack exists. */
    blesess_reset(&sess);
    blesess_claim_cancel(&ble_claim);
    auth_fails = 0;
    staged_total = 0;
    stall_mark = 0;
    stall_at = 0;
    peer_mtu = 23;
    flash_busy = false;
    tx_len = 0;
    /* The bus channel too, and for the same reason as everything else here:
     * assigned before the stack exists, so a fast peer cannot have a partial
     * frame or a stale pending request cleared out from under it. */
    bus_rx_len = 0;
    bus_rx_at = 0;
        bus_q_head.store(0, std::memory_order_release);
    bus_q_tail.store(0, std::memory_order_release);
    bus_link_broke.store(true, std::memory_order_release);
    bus_last_ms = 0;
    /* And the telemetry channel, before the stack exists, for the same reason
     * as everything else here: a subscription flag left over from the previous
     * window would have a peer that is gone counted as listening, and the app
     * task would poll the bus for nobody. The ARMING is deliberately NOT
     * cleared here - main.cpp owns the TelemState and clears it on the paths
     * where it means something, so a renewal (ble_ota_up on an already-up
     * radio returns before this) never lands here at all. */
    telem_subscribed.store(false, std::memory_order_release);
    telem_pending.store(false, std::memory_order_release);
    txlock_init();

    /* F7: the return value is not decoration. init() fails on controller or
     * NVS errors, including running out of internal heap - and this board has
     * NO PSRAM (esp32-s3-devkitc-1, and platformio.ini selects no PSRAM memory
     * type), so NimBLE, the LoRaWAN stack and esp-ota-ble's 8 kB ring all
     * compete for the same ~300 kB. Ignoring it left `up` claiming a radio
     * that does not exist, which then inhibits deep sleep for the whole window
     * and crashes at the first createServer(). */
    if (!NimBLEDevice::init(name)) {
        Serial.println("!! ble: NimBLEDevice::init() failed - radio stays down");
        return;
    }
#if PROTO_BLE_BENCH
    // init initializes the GAP listener registry: register only afterwards.
    if (!NimBLEDevice::setCustomGapHandler(benchGap))
        Serial.println("BENCH DLE event listener unavailable");
#endif
    /* 517 is the largest an ATT MTU can be. Firmware bytes go out at MTU-3 per
     * write-without-response packet, so this is throughput directly: at the
     * default 23 it would be 20 bytes a packet and the push would take longer
     * than the LoRaWAN path it exists to beat. */
    NimBLEDevice::setMTU(517);

    srv = NimBLEDevice::createServer();
    /* `false` = DO NOT let NimBLE delete this object. It is not optional.
     *
     * NimBLEServer::setCallbacks(cb, deleteCallbacks = true) defaults to
     * taking OWNERSHIP, and ~NimBLEServer() then does `delete
     * m_pServerCallbacks` (NimBLEServer.cpp:73). srv_cb is a FILE-SCOPE STATIC
     * (bleota.cpp:457), so that is a free() of a pointer that was never
     * malloc'd, and the heap asserts:
     *
     *   assert failed: heap_caps_free heap_caps_base.c:80
     *     (heap != NULL && "free() target pointer is outside heap areas")
     *
     * ~NimBLEServer() is reached from NimBLEDevice::deinit(true), which is
     * what ble_ota_down() calls - so before this argument, EVERY BLE teardown
     * panicked the node and rebooted it. Diagnosed 2026-09-10 from a backtrace
     * decoded against the flashed ELF (sha256 0ff8f81287527e6c): ble_ota_down
     * -> deinit -> ~NimBLEServer -> operator delete -> SrvCb::~SrvCb.
     *
     * Confirmed on the wire rather than only in the trace: node_f_cnt climbed
     * to 11 and reset to 0 at 06:40:42Z, the instant a 15-minute window
     * expired, while gas_probe_uptime_s kept rising - the node rebooted and
     * rejoined, the probe did not. The bus claim "releasing" at that moment
     * was RAM being wiped, not the release latch firing.
     *
     * NimBLECharacteristic::setCallbacks has no such parameter and never
     * deletes, so ctrl_cb/data_cb/bus_cb/telem_cb are safe as they are. */
    srv->setCallbacks(&srv_cb, false);

    NimBLEService *svc = srv->createService(BLE_SVC_UUID);
    chrCtrl = svc->createCharacteristic(BLE_OTA_CTRL_UUID,
                                        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
    chrCtrl->setCallbacks(&ctrl_cb);
    /* Write-no-response only. An acked write per chunk caps throughput at one
     * packet per connection interval, which is the difference between a
     * one-minute push and a ten-minute one. */
    chrData = svc->createCharacteristic(BLE_OTA_DATA_UUID, NIMBLE_PROPERTY::WRITE_NR);
    chrData->setCallbacks(&data_cb);
    /* WRITE with response, unlike chrData. Firmware bytes are a stream whose
     * loss the receiver detects by hash; a Modbus request is a transaction
     * whose loss looks exactly like a silent probe, and the retry would be a
     * REPEATED FC16 - which for the updater's COMMIT is not idempotent. The
     * per-chunk ack costs one connection interval and buys the distinction
     * between "the node never heard me" and "the bus said nothing". */
    chrBus = svc->createCharacteristic(BLE_BUS_UUID,
                                       NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
    chrBus->setCallbacks(&bus_cb);
    /* NOTIFY ONLY - no write property. The stream is armed over the
     * authenticated control characteristic; there is nothing for a peer to
     * write here, and not offering the property is one less thing on the air.
     * NimBLE creates the CCCD for a NOTIFY characteristic itself, which is what
     * TelemCb::onSubscribe() observes. */
    chrTelem = svc->createCharacteristic(BLE_TELEM_UUID, NIMBLE_PROPERTY::NOTIFY);
    chrTelem->setCallbacks(&telem_cb);
    svc->start();

    OtaBleHooks hooks;
    hooks.status  = &ota_status_hook;
    hooks.quiesce = &ota_quiesce_hook;
    hooks.restart = &ota_restart_hook;
    otaBleInit(hooks);

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();

    /* THE NAME MUST BE IN THE SCAN RESPONSE, NOT THE ADVERTISEMENT, and it has
     * to be set explicitly - NimBLEDevice::init(name) sets only the GAP name,
     * which a central cannot read until after it connects.
     *
     * MEASURED, 2026-09-07: farmgw's scan saw 34:B7:DA:5B:84:E1 with
     * `name=None`, so ota_ble_push.py - which matches `farmnode-` by name -
     * skipped the very node it was pushing to and failed with "no BLE device
     * matching 'farmnode-' found", while the window was open and the radio was
     * advertising perfectly well. The failure names the window, so it reads as
     * an expiry rather than a discovery bug.
     *
     * It cannot go in the advertisement: an ADV payload is 31 bytes, the
     * 128-bit service UUID takes 16 plus 2 of header, and "farmnode-XXXXXX" is
     * another 15 plus 2. 35 > 31, and an over-long field is dropped silently
     * rather than reported. The scan response is a second 31-byte packet an
     * active scanner asks for, which is exactly what it is for. */
    /* SET THE NAME EXPLICITLY. NimBLEDevice::init(name) sets only the GAP
     * name, which a central cannot read without connecting first - so without
     * this the node advertised with no name at all, and every consumer of this
     * receiver (which discovers by name) skipped the very node it was pushing
     * to, reporting "no BLE device found" while the radio advertised perfectly
     * well. Measured 2026-09-07: a scan saw 34:B7:DA:5B:84:E1 with name=None.
     *
     * Let NimBLE pack the payload rather than building NimBLEAdvertisementData
     * by hand. Name and a 128-bit service UUID do not both fit in a 31-byte
     * advertisement (3 flags + 17 name + 18 UUID), and NimBLE spills the
     * overflow into the scan response by itself once enableScanResponse(true)
     * is set - verified: a BlueZ scan reports both the name and the service
     * UUID. A hand-built split worked too, in both orderings, and was dropped
     * for being more code that has to be right.
     */
    /* Put the 128-bit Service UUID in the PRIMARY advertisement packet so
     * macOS / CoreBluetooth packet filtering catches it immediately, and put the
     * name in the scan response.
     */
    NimBLEAdvertisementData advData;
    advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
    advData.addServiceUUID(BLE_SVC_UUID);
    adv->setAdvertisementData(advData);

    NimBLEAdvertisementData scanData;
    scanData.setName(name);
    adv->setScanResponseData(scanData);
    adv->enableScanResponse(true);
    /* 500 ms. The operator is standing next to the node with a laptop, so
     * discovery latency of a second is invisible, while the advertising duty
     * cycle is the whole idle cost of an open window. */
    /* 546.25 ms, and the exact number is not arbitrary: it is one of the
     * advertising intervals Apple's Accessory Design Guidelines list as
     * acceptable (20 ms, then 152.5, 211.25, 318.75, 417.5, 546.25, 760,
     * 852.5, 1022.5, 1285 ms). This was 500 ms, chosen purely for power, and
     * 500 is NOT on that list - iOS and macOS are documented to discover
     * poorly at unlisted intervals, so the old value risked exactly the
     * failure mode below for no benefit. 546.25 ms costs marginally less
     * power, not more.
     *
     * It is NOT a fix for the macOS blindness measured on this bench, and was
     * not adopted as one: with the interval set to 20 ms and VERIFIED on the
     * air (635 adverts in 19.9 s from a BlueZ scanner, a 31 ms mean gap) this
     * Mac still could not see the board at all, while the same scanner saw it
     * at rssi -69. See the note in BLE-OTA-E2E.md. */
    adv->setMinInterval(874);
    adv->setMaxInterval(874);
    if (!adv->start()) {
        /* Up but undiscoverable is the worst of both: it burns the window and
         * inhibits sleep while no host can ever connect. Fail the bring-up. */
        Serial.println("!! ble: advertising failed to start - taking it down");
        NimBLEDevice::deinit(true);
        srv = nullptr; chrCtrl = nullptr; chrData = nullptr; chrBus = nullptr;
        return;
    }

    up = true;
    up_since = millis();

    Serial.printf("ble: UP as %s, %lu ms, nonce %04X\n",
                  name, (unsigned long)hold_ms, (unsigned)nonce);
}

void ble_ota_down(const char *why)
{
    if (!up) return;
    /* Abort BEFORE the stack goes away. otaBleRequestAbort() only latches, so
     * the abort has to be executed while a tick can still run - hence the
     * direct entry point, which is legal here because ble_ota_down() is only
     * ever called from the app task. */
    if (otaBleActive()) {
        Serial.println("!! ble: transfer still open at teardown - aborting it");
        otaBleAbort();
    }
    /* ORDER IS LOAD-BEARING. deinit() terminates the connection and the host
     * task runs onDisconnect() before deinit() returns, so every flag that
     * callback consults must already say "down". Setting them afterwards
     * crashed the node at the hard deadline; see the note on `teardown`. */
    /* END THE SESSION AND PUBLISH ITS CLEANUP IN ONE ATOMIC STEP, BEFORE
     * deinit() - which is also what onDisconnect() does, on the other task.
     *
     * WHAT WAS WRONG, and it took two reviews to state precisely. First,
     * onDisconnect() latches the heater disarm and the claim release only for
     * a peer that WAS authenticated, and this function cleared `authed` before
     * deinit() ran that callback - so a forced teardown of a genuinely
     * authenticated session presented as an unauthenticated one and published
     * neither latch (node-bus-heater, finding 2). Publishing here first fixed
     * that ordering. Then the second review found that the fix was still a
     * COMPOUND read of two independent atomics, which the other task is
     * allowed to be halfway through, so both callers could see "not
     * authenticated" and neither publish (node-panic-resync-heater, finding
     * 3). No ordering of those two loads repairs that.
     *
     * So the two flags are now one word and the transition is one exchange.
     * Whichever task gets the live word owes the cleanup; the other gets zero
     * and does nothing. deinit(true) still runs onDisconnect() synchronously
     * from inside itself, and that call is now a no-op rather than a second
     * opinion.
     *
     * THE ORDER IS STILL LOAD-BEARING and is unchanged: every flag the
     * callback consults says "down" before deinit() is entered, because
     * setting them afterwards crashed the node at the hard deadline - see the
     * note on `teardown`. blesess_end() is one store to the same word that
     * `have_conn`/`authed` used to be two stores to, so it satisfies that
     * requirement by the same argument, only sooner. */
    sess_end_and_publish();
    teardown = true;
    up = false; peer_mtu = 23;
    NimBLEDevice::deinit(true);
    teardown = false;
    srv = nullptr; chrCtrl = nullptr; chrData = nullptr; chrBus = nullptr;
    chrTelem = nullptr;
    flash_busy = false; stall_armed = false;
    /* The listener is gone with the stack. The ARMING is main.cpp's to clear -
     * it does so when it observes the radio is down, so the two states cannot
     * disagree about whether the fast path is running. */
    telem_subscribed.store(false, std::memory_order_release);
    telem_pending.store(false, std::memory_order_release);
    /* A pending request outlives nothing: the peer that would receive its
     * reply is gone, and leaving it set would make ble_bus_active() hold the
     * node awake against a link that no longer exists. */
    bus_q_head.store(0, std::memory_order_release);
    bus_q_tail.store(0, std::memory_order_release);
    bus_link_broke.store(true, std::memory_order_release);
    bus_rx_len = 0; bus_rx_at = 0; bus_last_ms = 0;
    if (tx_take()) { tx_len = 0; tx_give(); } else { tx_len = 0; }
    Serial.printf("ble: DOWN (%s)\n", why ? why : "");
}

bool ble_ota_is_up(void)      { return up; }
bool ble_ota_peer_connected(void) { return up && have_conn(); }

bool ble_ota_in_conn_grace(void)
{
    /* THE SINGLE ANSWER TO "may the radio outlive the CTRL session right now".
     *
     * REVIEW FINDING 2026-09-08: this used to be a condition living only
     * inside ble_ota_tick(), and main.cpp's ctrl_expire() path tore the radio
     * down anyway - it consults ble_ota_busy()/ble_bus_active(), neither of
     * which is true for an operator typing CTRL commands. The grace was
     * granted and then overridden later in the same loop iteration, so the
     * feature it was written for never worked. Exporting the predicate is
     * what stops the two call sites disagreeing again.
     *
     * Still bounded and still activity-gated: an idle or unauthenticated peer
     * gets nothing, and the cumulative ceiling above outranks this. */
    if (!up || !have_conn() || !authed()) return false;
    if ((int32_t)(millis() - last_act_ms - BLE_OTA_CONN_IDLE_MS) >= 0) return false;
    if ((int32_t)(millis() - (up_since + BLE_OTA_MAX_TOTAL_MS)) >= 0) return false;
    return (int32_t)(hard_deadline + BLE_OTA_CONN_GRACE_MS - millis()) > 0;
}

uint16_t ble_ota_hold_minutes(uint16_t minutes)
{
    if (!up) return 0;

    uint32_t ms = (uint32_t)minutes * 60000UL;
    if (ms > BLE_OTA_MAX_UP_MS) {
        ms = BLE_OTA_MAX_UP_MS;
        minutes = (uint16_t)(BLE_OTA_MAX_UP_MS / 60000UL);
    }

    /* THE CUMULATIVE CEILING BOUNDS THIS TOO, and an earlier comment here
     * asserted it was "not reachable from here" - which was false twice over.
     * Nothing clamped the resulting deadline TO the ceiling, so a hold placed
     * just under the hour ran ~15 minutes past it, and the connection grace
     * could add ~5 more: ~80 minutes from a nominal 60. Refuse once exhausted,
     * and clamp to the bound before then. */
    if ((int32_t)(up_since + BLE_OTA_MAX_TOTAL_MS - millis()) <= 0) {
        tx_append_line("OTAB FAIL window total exhausted");
        return 0;
    }

    uint32_t want = millis() + ms;
    uint32_t ceiling = up_since + BLE_OTA_MAX_TOTAL_MS;
    if ((int32_t)(want - ceiling) > 0) {
        want = ceiling;
        uint32_t left = ceiling - millis();
        minutes = (uint16_t)(left / 60000UL);   /* what the operator really got */
    }
    /* Never SHORTEN an existing window: a `blehold 1` typed during a longer
     * CTRL session would otherwise cut the operator off early, which is the
     * opposite of what the verb is for. */
    if ((int32_t)(want - hard_deadline) > 0) hard_deadline = want;
    last_act_ms = millis();
    return minutes;
}

bool ble_ota_take_cadence_request(uint16_t *seconds, uint16_t *lease_min)
{
    if (!seconds || !lease_min) return false;
    /* ACQUIRE: pairs with the release store in CtrlCb::onWrite(), so the
     * payload words below are guaranteed visible once the flag reads true. */
    if (!cad_pending.load(std::memory_order_acquire)) return false;
    *seconds   = cad_req_s;
    *lease_min = cad_req_lease;
    /* Cleared last, and with release, so the host task cannot observe the slot
     * free while this task is still reading it. */
    cad_pending.store(false, std::memory_order_release);
    return true;
}

void ble_ota_report_cadence(uint16_t seconds, uint16_t lease_min, bool clamped)
{
    char line[64];
    if (seconds == 0)
        snprintf(line, sizeof(line), "OTAB CADENCE default");
    else
        snprintf(line, sizeof(line), "OTAB CADENCE %u s lease %u min%s",
                 (unsigned)seconds, (unsigned)lease_min,
                 clamped ? " (clamped)" : "");
    tx_append_line(line);
}

bool ble_ota_take_claim_request(uint16_t *minutes)
{
    /* THE CONSUMER RECHECKS THE OWNER, and that is the half of the finding-4
     * fix that cannot be lost to a race. Cancelling the latch at disconnect is
     * done too (sess_end_and_publish), but a cancel that has not run yet is
     * exactly the window the review described: an authenticated peer queues
     * `claim`, disconnects, the app task processes the release first and this
     * request second, and a claim is installed for a peer that is gone - which
     * inhibits the scheduled poll, and therefore deletes the whole gas block
     * from every uplink, for up to CTRL_MAX_SESSION_MIN.
     *
     * An orphan is CONSUMED rather than left pending: leaving it would only
     * move the grant to the next pass, or hand it to the next peer to
     * connect. */
    uint16_t asked = 0;
    const int r = blesess_claim_take(&ble_claim, &sess, &asked);
    if (r == BLESESS_TAKE_ORPHAN) {
        Serial.println("ble: queued bus claim DROPPED - the session that "
                       "asked for it is gone");
        return false;
    }
    if (r != BLESESS_TAKE_OK) return false;
    if (minutes) *minutes = asked;
    return true;
}

bool ble_ota_take_release_request(void)
{
    return ble_release_pending.exchange(false, std::memory_order_acq_rel);
}

bool ble_ota_take_disconnect_disarm(void)
{
    return ble_disconnect_disarm.exchange(false, std::memory_order_acq_rel);
}

void ble_ota_report_claim(uint16_t minutes, bool ok)
{
    if (ok) {
        char line[48];
        snprintf(line, sizeof(line), "BUS CLAIM ok %u min", (unsigned)minutes);
        tx_append_line(line);
    } else {
        tx_append_line("BUS FAIL claim refused");
    }
}
bool ble_ota_busy(void)       { return up && otaBleActive(); }
bool ble_ota_flash_busy(void) { return flash_busy; }

/* ---- SERIAL OTA: the same receiver, a different transport --------------
 *
 * esp-ota-ble "knows nothing about BLE" (ota_ble.h): otaBleSubmitCommand() is
 * safe from any task and otaBleStageBytes() only asks not to be the consumer.
 * The BLE-only part was never the receiver, it was the transport - a CTRL
 * characteristic for control lines and a DATA characteristic for firmware
 * bytes. Over USB that is one port: newline-terminated commands, then raw
 * bytes. The status hook already mirrors every OTAB line to Serial
 * (ota_status_hook), so the host's line parser works unchanged.
 *
 * WHY THESE WRAP RATHER THAN main.cpp CALLING THROUGH: main.cpp includes
 * bleota.h, not <ota_ble.h>. This file already owns the hooks, the session and
 * the only otaBleTick() call, so the receiver stays behind one door.
 *
 * SELF-PACING IS MANDATORY, and it is why stage() ticks. otaBleStageBytes()
 * does NOT block and does NOT drop when the ring is full - it latches
 * `failed`, and the consumer reports OTAB FAIL credit-overrun and aborts. On
 * BLE the credit window paces the host; a serial host has no such brake, so
 * the drain has to happen between writes. Producer and consumer being the same
 * task is fine here precisely because it serialises them: while the tick
 * blocks on flash nothing is read, the CDC FIFO backs up, and the host's
 * write() blocks. That is the flow control.
 *
 * ble_ota_tick()'s `if (!up)` guard is BLE-window state, so these must call
 * otaBleTick() directly - going through ble_ota_tick() with the radio down
 * would stage bytes that are never drained, straight into credit-overrun. */
/* SERIAL IS A CLIENT THE `no client` GUARD BELOW CANNOT SEE, and that is what
 * killed the first serial delta push. MEASURED 2026-09-10: `begin` was
 * accepted, `OTAB READY part=app0 size=30804 xform=delta out=718592` and
 * `OTAB CRED 30804` both came back - and then the very next ble_ota_tick()
 * printed "transfer open with no client - aborting" and tore the session down,
 * because have_conn() is blesess_conn(&sess), a BLE-session predicate that a
 * USB host can never satisfy.
 *
 * A FLAG ALONE WOULD BE A REGRESSION, so this is a flag AND a deadline. The
 * guard's comment is protecting a real hazard: esp_ota_begin() has already
 * fired the quiesce hook, so a transfer whose driver vanished leaves the poller
 * parked indefinitely. A serial driver dies exactly the same way - kill the
 * host process mid-push and nothing on the wire ever says so. Exempting serial
 * unconditionally would hand that hazard back for the one transport with no
 * link-layer disconnect to detect. So the exemption expires: `serial_seen_ms`
 * is refreshed by every accepted command and every staged block, and once it
 * goes stale the guard fires as it always did.
 *
 * BLE_OTA_STALL_MS, and the same signed comparison as the stall watchdog at the
 * bottom of ble_ota_tick(), deliberately - one idle-driver policy, not two.
 *
 * THE FLAG IS NOT REDUNDANT WITH THE TIMESTAMP. A zero-initialised
 * `serial_seen_ms` cannot mean "never": millis() is small right after boot, so
 * `now - 0 < BLE_OTA_STALL_MS` reads as LIVE for the first 30 seconds of
 * uptime, which would suppress the guard for every BLE transfer that started in
 * that window. The flag is what distinguishes "no serial driver has ever
 * spoken" from "one spoke at millisecond 12". */
static std::atomic<uint32_t> serial_seen_ms{0};

static void serial_driver_mark(void)
{
    serial_seen_ms.store(millis(), std::memory_order_release);
    serial_driving.store(true, std::memory_order_release);
}

static bool serial_driver_live(uint32_t now)
{
    if (!serial_driving.load(std::memory_order_acquire)) return false;
    const uint32_t seen = serial_seen_ms.load(std::memory_order_acquire);
    return (int32_t)(now - seen - BLE_OTA_STALL_MS) < 0;
}

bool ble_ota_serial_submit(const char *line)
{
    if (otaBleActive() && !serial_driving.load(std::memory_order_acquire)) return false;
    const bool ok = otaBleSubmitCommand(line) == OtaBleSubmit::Accepted;
    /* ON ACCEPTANCE ONLY. A rejected line is not evidence of a live driver, and
     * counting it would let a host that only ever sends garbage hold the
     * exemption open. */
    if (ok) serial_driver_mark();
    return ok;
}

void ble_ota_serial_stage(const uint8_t *d, size_t n)
{
    serial_driver_mark();
    otaBleStageBytes(d, n);
    otaBleTick(millis());
}

void ble_ota_serial_pump(void) { otaBleTick(millis()); }

bool ble_ota_serial_active(void) { return otaBleActive(); }

static void ble_tune_link(void)
{
    if (!have_conn() || !authed() || otaBleActive()) return;
    const uint32_t now = millis();
    const unsigned stage = link_tune_stage.load(std::memory_order_acquire);
    if (stage == 1 && (uint32_t)(now - conn_at) >= 600u) {
        // Equal endpoints matter: macOS accepted 15/15 ms, but chose 30 ms
        // from 15..30 ms. Keep 720 ms supervision so reboot detection does
        // not add the 5 s delay measured with the previous request.
        link_params_seen.store(false, std::memory_order_release);
        link_tune_at = now;
        link_tune_stage.store(2, std::memory_order_release);
#ifndef PROTO_BLE_BENCH_INTERVAL
#define PROTO_BLE_BENCH_INTERVAL 12
#endif
        srv->updateConnParams(conn_handle, PROTO_BLE_BENCH_INTERVAL,
                              PROTO_BLE_BENCH_INTERVAL, 0, 72);
    } else if (stage == 2 &&
               (link_params_seen.load(std::memory_order_acquire) ||
                (uint32_t)(now - link_tune_at) >= 1000u)) {
        // One request after interval negotiation; unsupported peers retain
        // their PHY. Never initiate a control procedure during flash OTA.
        link_tune_stage.store(3, std::memory_order_release);
        if (!srv->updatePhy(conn_handle, BLE_GAP_LE_PHY_2M_MASK,
                            BLE_GAP_LE_PHY_2M_MASK, 0))
            Serial.println("ble: 2M request refused; retaining negotiated PHY");
    }
}

void ble_ota_tick(void)
{
    if (!up) return;
    ble_tune_link();

    /* RESCUE RESET, drained here because this is the app task. Ordered so the
     * acknowledgement actually leaves: drain the TX queue, give the link a
     * moment to put it on air, and only then reboot. Any transfer in flight is
     * abandoned deliberately - the passive slot is half-written, which is
     * harmless, and the operator asking for a reset wants the board back more
     * than they want the push. */
    if (reset_pending.load(std::memory_order_acquire)) {
        reset_pending.store(false, std::memory_order_release);
        drain_tx();
        delay(300);
        Serial.println("ble: rebooting on request");
        Serial.flush();
        node_prepare_restart();
        esp_restart();
    }

    otaBleTick(millis());
    /* BEFORE drain_tx(), so a refusal this exchange produced goes out on the
     * same tick as the reply that accompanies it rather than one later. */
    bus_service();
    drain_tx();

    /* AN OPEN TRANSFER WITH NO PEER CAN NEVER COMPLETE, so end it now rather
     * than waiting out the stall timer.
     *
     * This is not merely an optimisation, it closes a race in the upstream
     * receiver's contract. `begin` is LATCHED by otaBleSubmitCommand() on the
     * host task and only executed on the next tick, while otaBleRequestAbort()
     * early-returns when the transfer is not yet active - so a client that
     * writes `begin` and disconnects inside that window leaves a transfer that
     * nothing asked for and nothing will ever abort. The consequences are
     * worse than a wasted slot: esp_ota_begin() has already fired the quiesce
     * hook, so the node would sit with its poller parked indefinitely. Testing
     * the peer rather than the request removes the race entirely, because
     * `have_conn` cannot be true for a peer that is gone. */
    /* ...OR NO SERIAL DRIVER. See serial_driver_live() above: a USB host is a
     * client this predicate cannot see, and its exemption expires rather than
     * being unconditional, so a serial driver that dies mid-push still lands
     * here once BLE_OTA_STALL_MS has passed with nothing staged. */
    if (otaBleActive() && !have_conn() && !serial_driver_live(millis())) {
        Serial.println("!! ble: transfer open with no client - aborting");
        otaBleAbort();
        stall_armed = false;
        /* CLEAR THE FLAG WITH THE TRANSFER. Left set, a stale timestamp from a
         * dead serial push would be re-evaluated against every future
         * transfer's clock instead of being forgotten with the session. */
        serial_driving.store(false, std::memory_order_release);
        return;
    }

    /* HANG UP ON A PEER THAT NEVER AUTHENTICATES. A connected peripheral stops
     * advertising, so one silent client denies the window to the real one -
     * measured on the bench, see BLE_OTA_AUTH_GRACE_MS. Disconnecting rather
     * than taking the whole radio down is deliberate: the CTRL session paid
     * for this window and the operator should still get to use it, so
     * onDisconnect() re-arms advertising and the next client gets a clean try.
     *
     * `authed` and not `otaBleActive()` is the test. A peer that authenticated
     * but has not yet sent `begin` is a host that is still hashing the image,
     * and it has already proven it holds the CTRL nonce. */
    if (have_conn() && !authed() &&
        (int32_t)(millis() - conn_at - BLE_OTA_AUTH_GRACE_MS) >= 0) {
        Serial.printf("!! ble: client connected %lu ms without authenticating "
                      "- disconnecting so the window is not squatted\n",
                      (unsigned long)BLE_OTA_AUTH_GRACE_MS);
        if (srv && conn_handle != BLE_HS_CONN_HANDLE_NONE)
            srv->disconnect(conn_handle);
        conn_at = millis();     /* do not re-fire before onDisconnect lands */
    }

    /* Stall watchdog. ARMED ON THE TRANSITION into an open transfer, not on the
     * first byte: `stall_at` used to be left at 0 until something arrived, so a
     * transfer that received NOTHING - the exact case this is meant to catch -
     * never armed the timer at all and the `stall_at &&` guard silently
     * disabled the whole watchdog. Rearmed on every byte after that, so a slow
     * link is never mistaken for a dead one. */
    if (otaBleActive()) {
        uint32_t now = millis();
        if (!stall_armed || staged_total != stall_mark) {
            stall_mark = staged_total;
            stall_at = now;
            stall_armed = true;
        } else if ((int32_t)(now - stall_at - BLE_OTA_STALL_MS) >= 0) {
            /* SIGNED, and a separate `stall_armed` rather than `stall_at != 0`
             * as the sentinel. The first version stored 1 when millis() was 0,
             * so a rearm inside millisecond zero made `now - stall_at`
             * underflow to UINT32_MAX and tripped the watchdog instantly. Deep
             * sleep normally keeps millis() far from a wrap, but indefinite
             * renewal is exactly the state that makes it reachable. */
            Serial.println("!! ble: transfer stalled - aborting");
            otaBleAbort();
            tx_append_line("OTAB FAIL stalled");
            stall_armed = false;
        }
    } else {
        stall_armed = false;
    }

    /* THE CUMULATIVE BOUND, ENFORCED HERE AND NOT ONLY AT BRING-UP.
     *
     * A ceiling checked only where the deadline is written is a ceiling that
     * any future writer can forget - and one already had. Testing it on every
     * tick makes it true of the radio's actual lifetime rather than of one
     * code path, so no grace, hold or renewal below can outlive it. This is
     * the last line of defence for the battery, so it comes FIRST and takes
     * no exceptions - not even a transfer in flight. */
    /* THE ONE EXCEPTION, AND IT IS A BUILD-TIME ONE. A window that is actually
     * carrying fast telemetry - armed on the app task AND with a peer
     * subscribed - is bounded by BLE_TELEM_MAX_TOTAL_MS instead, which
     * DEFAULTS TO THE SAME VALUE. So out of the box this line changes nothing
     * whatever, and the only way to hold a window open longer than an hour is
     * for somebody to have raised that constant on the build line, having read
     * what config.h says it costs. Deliberately not reachable at runtime: a
     * bound the operator standing next to the node can lift is not a bound. */
    uint32_t total_ms = telem_holding_window() ? BLE_TELEM_MAX_TOTAL_MS
                                               : BLE_OTA_MAX_TOTAL_MS;
    if (up && (int32_t)(millis() - (up_since + total_ms)) >= 0) {
        ble_ota_down("cumulative window exhausted");
        return;
    }

    if ((int32_t)(hard_deadline - millis()) > 0) return;

    /* A LIVE TELEMETRY STREAM BUYS THE WINDOW, for the same reason a transfer
     * in flight does and with a stricter test than either grace above: the
     * peer must be subscribed AND the lease must still be running, so this
     * cannot be held by an idle connection or by an arming whose host left.
     * The lease is bounded by TELEM_MAX_LEASE_MIN and the cumulative ceiling
     * checked above outranks it, so this cannot run away.
     *
     * NO ACTIVITY GATE, unlike ble_ota_in_conn_grace(): the traffic here is the
     * NODE talking, not the peer, and a host that is only listening is doing
     * exactly what this feature is for. What replaces it is that a listener
     * which disappears clears telem_subscribed on the disconnect. */
    if (telem_holding_window()) return;

    /* Past the deadline. A transfer in flight buys a bounded grace and nothing
     * more: cutting the radio at the deadline would throw away a push that is
     * seconds from finishing, but an unbounded grace would hand any client a
     * way to hold the radio up forever by keeping a transfer nominally open.
     * The stall watchdog above is what makes the grace terminate. */
    if (otaBleActive() &&
        (int32_t)(hard_deadline + BLE_OTA_GRACE_MS - millis()) > 0)
        return;

    /* AN OPERATOR MID-CONVERSATION BUYS THE SAME BOUNDED GRACE A TRANSFER DOES.
     *
     * The grace above only covers otaBleActive() - a firmware transfer. It
     * does not cover someone working the CTRL characteristic (`cadence`,
     * `blehold`, `status`), because none of those open a transfer. Without
     * this, the link is cut mid-command at the session deadline, which is
     * precisely when a bench operator is most likely to be using it.
     *
     * Gated on ACTIVITY and bounded, for the reason stated on the transfer
     * grace: an idle connection must not be able to hold the radio up, or a
     * client that connects and walks away costs the battery the whole window
     * at ~31 mW. A link quiet for BLE_OTA_CONN_IDLE_MS stops extending
     * anything. The cumulative BLE_OTA_MAX_TOTAL_MS ceiling still applies
     * above this and is not reachable from here. */
    if (ble_ota_in_conn_grace()) return;

    ble_ota_down(otaBleActive() ? "deadline, grace exhausted" : "window expired");
}

#endif /* PROTO_BLE_OTA */
