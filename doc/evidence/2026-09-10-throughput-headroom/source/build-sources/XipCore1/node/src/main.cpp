/* Prototype LoRaWAN <-> UART relay.
 *
 * This node parses NOTHING. A downlink's bytes go out of the UART verbatim;
 * whatever comes back within the inter-frame gap goes up as an uplink. All
 * Modbus knowledge - CRC, FC03/FC16 shapes, block sequencing, the
 * UPD_NEXT_SEQ resync - stays in tools/o2p_update.py on the laptop, which is
 * where the diagnostics already are. The bridge board it talks to is
 * deliberately not a Modbus master either, for the same reason.
 *
 * Keeping the relay dumb is the point: every bug then reproduces on a laptop
 * with a serial lead, without a radio in the path.
 */
#include <Arduino.h>
#include <driver/gpio.h>      /* gpio_hold_en/dis - see park_for_sleep() */
#include <driver/uart.h>      /* uart_get_baudrate/uart_get_parity: the `bus`
                               * command reads the live line settings back OUT
                               * of the peripheral rather than reporting a
                               * variable that only records what was asked
                               * for - see bus_line_read(). */
#include "bleota.h"
#include <RadioLib.h>
#include "config.h"
#include "selfupdate.h"
#include "lwstore.h"
#include "poll.h"
#include "mbresync.h"
#include "pollcache.h"
#include <driver/rtc_io.h>
#ifdef PROTO_MOCK_PROBE
#include "mockprobe.h"
#endif
#include "dutycharge.h"
#include "rtcstate.h"
#include "cadence.h"
#include "cadstore.h"
#include "busstore.h"
#include "telemetry.h"
#include "ctrl.h"
#include "serbus.h"
#include "caps.h"
/* LAST, ALWAYS. console.h ends with `#define Serial nodecon`, which reroutes
 * every Serial.printf/println/flush/begin below it through the gate that a
 * serial bus session silences. A header included AFTER it would have its own
 * declaration of `Serial` renamed instead of its uses. */
#include "console.h"
#include <type_traits>

/* THE SHADOW IS LOAD-BEARING, SO IT IS CHECKED.
 *
 * If console.h ever ends up included before a header that redefines `Serial`
 * - or is dropped from this file altogether - every printf below silently
 * goes back to the raw CDC object, the console keeps talking through a
 * firmware push, and the symptom is a Modbus CRC error on the laptop that
 * looks like a bad cable. Nothing else in the build would fail. This turns
 * that into a compile error. */
static_assert(std::is_same<decltype(Serial), ConsoleGate>::value,
              "console.h must be the LAST include: Serial is not the gate");

SPIClass spi(FSPI);

/* THE RADIO COUNTS ITS OWN AIRTIME.
 *
 * Every previous attempt at duty accounting inferred what went on the air from
 * what RadioLib reported afterwards, and every one of them was wrong in a
 * different way: getLastToA() is zeroed by a TX_TIMEOUT, replaced by the
 * recursive MAC-only uplink, and cannot distinguish two frames that happen to
 * have the SAME quantized airtime - a 96-byte MAC response is the same 109-byte
 * PHY frame, and therefore the same 184 ms, as our payload. No amount of
 * arithmetic on a single reported total can separate those.
 *
 * So stop inferring. stageMode() and launchMode() are virtual, and every
 * transmission - ours, a repeat, or one RadioLib starts on its own with its own
 * duty guard disabled - goes through them. Counting there makes the accounting
 * a MEASUREMENT of what the radio was actually told to send:
 *
 *   - stageMode(TX) carries the exact PHY length in cfg->transmit.len
 *   - launchMode() is the moment RF starts; if it fails, nothing flew
 *   - getTimeOnAir(len) prices it at the modulation the radio is configured
 *     with RIGHT THEN, so ADR, a rate change, or a different frame size are
 *     all accounted for without being predicted
 *
 * This also removes three separate defects for free: a repeat sequence that
 * exits early (nbTrans is a maximum, not a count), a PACKET_TOO_LONG that never
 * reached the radio, and the hidden MAC-only uplink. What is not launched is
 * not charged; what is launched is charged exactly once. */
class CountingSX1262 : public SX1262 {
public:
    using SX1262::SX1262;

    int16_t stageMode(RadioModeType_t mode, RadioModeConfig_t *cfg) override {
        staged_tx_ = (mode == RADIOLIB_RADIO_MODE_TX);
        staged_len_ = (staged_tx_ && cfg) ? cfg->transmit.len : 0;
        return SX1262::stageMode(mode, cfg);
    }

    int16_t launchMode() override {
        int16_t st = SX1262::launchMode();
        if (staged_tx_ && st == RADIOLIB_ERR_NONE) {
            /* Priced here, at the instant RF starts, from the radio's live
             * configuration - not from a guess made before or a report made
             * after. */
            air_us_ += (uint64_t)SX1262::getTimeOnAir(staged_len_);
            tx_count_++;
        }
        return st;
    }

    /* Read and clear. The caller owns one sendReceive()'s worth of airtime. */
    uint32_t take_air_ms(uint32_t *count = nullptr) {
        uint64_t us = air_us_; uint32_t n = tx_count_;
        air_us_ = 0; tx_count_ = 0;
        if (count) *count = n;
        /* Round UP: a partial millisecond of air is still air. */
        return (uint32_t)((us + 999ULL) / 1000ULL);
    }
    uint32_t peek_air_ms() const { return (uint32_t)((air_us_ + 999ULL) / 1000ULL); }

private:
    bool     staged_tx_  = false;
    size_t   staged_len_ = 0;
    uint64_t air_us_     = 0;
    uint32_t tx_count_   = 0;
};

CountingSX1262 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO1, PIN_LORA_RST,
                                  PIN_LORA_BUSY, spi);
/* EU868 with RX2 overridden.
 *
 * RadioLib's band table hard-codes the EU868 regional default
 * `.rx2 = { .freq = 8695250, .dr = 0 }` -- 869.525 MHz, SF12, in 100 Hz steps.
 * Nothing on the air changes that for us: a JoinAccept's DLSettings carries
 * the RX2 DATA RATE but NOT the RX2 FREQUENCY, which moves only on an
 * RXParamSetupReq MAC command, and ChirpStack issued none.
 *
 * MEASURED 2026-09-06: with the gateway moved to band 56a the packet forwarder
 * transmitted 12 downlinks, 0 rejected, at freq 869.85 / SF7BW125 / 12 dBm
 * imme:true -- textbook Class C -- and the node heard none of them, because it
 * was still listening on 869.525 at SF12. The server was certain the device
 * knew; the device was never told. Both sides looked healthy.
 *
 * So state RX2 explicitly on this side too. This is a private arrangement
 * between this gateway and this node -- 869.85 is outside the EU868 channel
 * plan -- so it belongs in the node's own configuration, not in a MAC command
 * we would have to wait for. Keep in step with rx2_frequency and rx2_dr in
 * chirpstack/configuration/chirpstack/region_eu868.toml. */
static LoRaWANBand_t band56a;

/* WHY A SUBCLASS. The duty gate needs to know what our own uplink cost, and
 * getLastToA() cannot answer that in two cases RadioLib creates itself:
 *
 *   TX_TIMEOUT - sendReceive() zeroes lastToA before transmitting
 *   (LoRaWAN.cpp:173) and transmitUplink() returns TX_TIMEOUT
 *   (LoRaWAN.cpp:1535) BEFORE reaching `lastToA += toa` (LoRaWAN.cpp:1551).
 *   RadioLib itself treats that outcome as possibly-transmitted and advances
 *   fCntUp, so the frame may well be on the air - and reports zero airtime for
 *   it. Zero reads as "nothing was sent", which skips the gate entirely.
 *
 *   MAC-only uplinks - when the MAC responses exceed 15 bytes RadioLib lifts
 *   its own duty guard and RECURSIVELY calls sendReceive() (LoRaWAN.cpp:2271-
 *   2277). That inner call zeroes lastToA again, so what we read afterwards is
 *   the inner frame's airtime and OUR frame's airtime is simply gone.
 *
 * So compute the airtime ourselves. These accessors reach the protected
 * members that LoRaWAN.cpp:1478-1483 uses, and call RadioLib's OWN
 * calculateTimeOnAir with RadioLib's own band table - it is not a
 * reimplementation of the LoRa airtime formula, which is why the result can be
 * compared to getLastToA() for exact equality rather than with a tolerance.
 * If a future RadioLib moves these members this fails to COMPILE, which is the
 * failure mode we want from a legal limit. */
class DutyLoRaWANNode : public LoRaWANNode {
public:
    using LoRaWANNode::LoRaWANNode;

    /* Airtime of ONE transmission of a `phy_len`-byte PHY frame at the live
     * uplink data rate, in ms. */
    uint32_t uplinkToaMs(uint8_t phy_len) {
        const uint8_t dr = this->channels[RADIOLIB_LORAWAN_UPLINK].dr;
        if (!this->band) return 0;
        return (uint32_t)(this->phyLayer->calculateTimeOnAir(
                              this->band->dataRates[dr].modem,
                              this->band->dataRates[dr].dr,
                              this->band->dataRates[dr].pc,
                              phy_len) / 1000);
    }
    /* ADR may ask for the same frame several times; all of them are airtime. */
    uint8_t uplinkRepeats() { return this->nbTrans ? this->nbTrans : 1; }
};

DutyLoRaWANNode node(&radio, &band56a);

/* LoRaWAN PHY overhead over the application payload: MHDR 1 + DevAddr 4 +
 * FCtrl 1 + FCnt 2 + FPort 1 + MIC 4. RadioLib uses the same constant when it
 * sizes a maximum frame (LoRaWAN.cpp:1586), so it is taken from there rather
 * than counted out again here. */
#define LORAWAN_PHY_OVERHEAD 13

/* The worst-case EU868 uplink this node can produce: a full 109-byte PHY frame
 * at SF12/125 kHz is ~2.79 s on air. Used only where the real airtime cannot
 * be computed, because "unknown" must never be charged as zero. */
#define DUTY_UNKNOWN_TOA_MS 2790UL

/* Modbus t3.5 at 9600 8E1 is 39 bit times ~= 4.06 ms (the firmware rounds up
 * to a whole bit time, which is the safe direction - a longer gap can never
 * split a frame). Allow generous margin: the bridge is store-and-forward, so
 * its reply arrives as one burst well after the request, and an idle gap that
 * is too short truncates it into two "frames". */
static const uint32_t FRAME_GAP_MS   = 20;
static const uint32_t REPLY_WAIT_MS  = 1500;
static const size_t   MAX_FRAME      = 256;   /* a full Modbus ADU */

/* REPLY QUEUE.
 *
 * Under Class A exactly one reply can ever be outstanding, because a downlink
 * only arrives in the window opened by our own uplink -- so a single buffer
 * happened to be safe. It is NOT safe under Class C, where a downlink can land
 * at any instant, including while a previous reply still waits for its uplink
 * slot. A single buffer silently drops the older one, and both the relay and
 * the self-updater write into it. Hence a real queue with a drop counter, so a
 * loss is visible rather than silent. */
struct Reply {
    uint8_t  port;
    uint16_t len;
    uint8_t  buf[MAX_FRAME];
};
static const uint8_t REPLY_QUEUE_DEPTH = 4;
static Reply    rq[REPLY_QUEUE_DEPTH];
static uint8_t  rq_head = 0, rq_tail = 0, rq_count = 0;
static uint32_t rq_dropped = 0;

static bool reply_push(const uint8_t *p, size_t n, uint8_t port) {
    if (n == 0 || n > MAX_FRAME) return false;
    if (rq_count == REPLY_QUEUE_DEPTH) {
        rq_dropped++;
        Serial.printf("!! reply queue full, dropped (total %lu)\n",
                      (unsigned long)rq_dropped);
        return false;
    }
    Reply *r = &rq[rq_tail];
    r->port = port; r->len = (uint16_t)n;
    memcpy(r->buf, p, n);
    rq_tail = (uint8_t)((rq_tail + 1) % REPLY_QUEUE_DEPTH);
    rq_count++;
    return true;
}

/* PUT BACK a reply that was popped but never transmitted.
 *
 * reply_pop() removes the head BEFORE the send is attempted, and the CTRL and
 * cadence state it answers for is committed as soon as the ACK is merely
 * queued. So a send that does not happen loses the ACK while its effects
 * stand: a SESSION_OPEN claims the bus and opens Class C, and the host never
 * learns it was granted. Enabling the duty cycle turned that from a rare
 * radio failure into an ORDINARY one - a CTRL downlink arrives right after
 * the invitation uplink, while the band still owes ~18/33/59 s of silence at
 * DR5/DR4/DR3 and the post-uplink window is only 12 s.
 *
 * Restoring at the HEAD, not the tail: these are answers to specific
 * requests, and reordering them behind a later reply is its own bug. */
static void reply_unpop(const Reply *r) {
    if (rq_count == REPLY_QUEUE_DEPTH) {
        /* Nowhere to put it. Say so - this is a lost answer, not a skip. */
        rq_dropped++;
        Serial.printf("!! reply queue full, could not restore an unsent reply "
                      "(total dropped %lu)\n", (unsigned long)rq_dropped);
        return;
    }
    rq_head = (uint8_t)((rq_head + REPLY_QUEUE_DEPTH - 1) % REPLY_QUEUE_DEPTH);
    rq[rq_head] = *r;
    rq_count++;
}

static bool reply_pop(Reply *out) {
    if (rq_count == 0) return false;
    *out = rq[rq_head];
    rq_head = (uint8_t)((rq_head + 1) % REPLY_QUEUE_DEPTH);
    rq_count--;
    return true;
}

static uint32_t last_uplink = 0;
/* SEPARATE from last_uplink, and it has to be.
 *
 * Both used to be one variable, which made the measurement cadence "300 s
 * since ANY uplink" rather than a schedule. A queued relay or self-update
 * reply bypasses the interval and then resets the clock, so a host talking to
 * the node more often than every five minutes would have silently stopped the
 * measurements for as long as it kept talking - and the series would show a
 * gap exactly when someone was working on the node, which is the worst
 * possible time to lose the data and the easiest to explain away. */
static uint32_t last_poll = 0;
/* When the node may next sleep. Pushed forward by every downlink and by every
 * uplink's listening window; a session request pushes it minutes ahead. Sleep
 * is only ever entered when this has passed, so no single mechanism has to
 * know about all the others. */
static uint32_t awake_until = 0;
/* When su_reboot_armed() was first observed; 0 = not armed. */
static uint32_t reboot_armed_at = 0;

/* The CTRL session, and the nonce that invites one. */
static CtrlState ctrl = {};

/* THE CUMULATIVE WINDOW HAS TO BE CARRIED ACROSS SLEEP BY HAND.
 *
 * `ctrl` is an ordinary static, so every deep-sleep wake zeroes it. The RTC
 * accessors were written for exactly this and then never called - the ceiling
 * looked persistent, described itself as persistent in its own comments, and
 * still reset at every sleep. A mechanism nobody invokes is a mute button, not
 * a fix, so these two are called at the load point and after EVERY mutation of
 * the window rather than at a single convenient place. */
static void ctrl_window_restore(void) {
    bool open = false, session_live = false;
    uint32_t start_s = 0, close_s = 0;
    rtc_ctrl_window_load(&open, &start_s, &close_s, &session_live);
    ctrl.window_open    = open;
    ctrl.window_start_s = start_s;
    ctrl.last_close_s   = close_s;

    /* A REBOOT ENDS A SESSION, AND THE QUIET PERIOD STARTS THERE.
     *
     * The live session flags - ctrl.open and ctrl.serial_claim - are plain
     * statics and are false on every boot, while last_close_s still holds
     * whatever the PREVIOUS session left behind. So the quiet-period test in
     * ctrl_handle() ("session-free for 30 minutes -> start a fresh run") saw a
     * node with no session open and an ancient last_close_s, and cleared
     * window_open. The run's age was carried faithfully across the restart by
     * the layer below and then discarded by the layer above: reboot inside a
     * session, and the four-hour ceiling was refunded anyway.
     *
     * If a session was live when the state was last written, the reboot is
     * when it ended. Recording that here makes the quiet period run from the
     * reboot, so the run continues - and 30 genuinely session-free minutes
     * after it still start a new one, which is what the rule is for. */
    if (session_live) {
        ctrl.last_close_s = rtc_uptime_s();
        rtc_ctrl_window_store(ctrl.window_open, ctrl.window_start_s,
                              ctrl.last_close_s, false);
        Serial.println("CTRL: a session was live at the reset - the run "
                       "continues, and its quiet period starts now");
    }
}
static void ctrl_window_persist(void) {
    /* `session_live` is the LIVE session, not the window: either transport
     * holding the node counts, because either one ending in a reset is what
     * the restore above has to reason about. */
    rtc_ctrl_window_store(ctrl.window_open, ctrl.window_start_s,
                          ctrl.last_close_s,
                          ctrl.open || ctrl.serial_claim);
}

/* THE INTERVAL THE NODE IS ACTUALLY ON. Three layers, most specific first:
 *
 *     RTC lease         temporary, supervised, expires by itself (cadence.h)
 *     NVS stored value  persistent, survives reset/power/OTA    (cadstore.h)
 *     POLL_INTERVAL_MS  the compiled default
 *
 * The live value can be overridden at runtime by CTRL_SET_CADENCE or the BLE
 * `cadence` verb. Everything that schedules a measurement goes through here, so
 * there is one place where the override can be forgotten and it is this one.
 *
 * THE FALLBACK IS cad_default_ms(), NOT POLL_INTERVAL_MS. A lapsed lease must
 * return to the STORED cadence, not to the compiled one - otherwise storing a
 * value and then taking a short bench lease would silently discard the stored
 * value the moment the lease ran out, which is the opposite of what "put it
 * back" means to whoever set it. */
/* THE DUTY-CYCLE FLOOR, derived from the last frame's MEASURED airtime.
 *
 * cadence.h:55-63 says this is the fix and says why it was missing: "a fast
 * cadence held at DR3 is a regulatory problem, not merely an energy one, and
 * nothing in this firmware currently checks the live DR before granting one.
 * Written down rather than silently assumed; making the clamp DR-aware is the
 * fix if this ever runs unattended." This is that clamp.
 *
 * WHY IT IS NOT A CONSTANT. Airtime is a function of the live data rate and of
 * how many MAC bytes RadioLib is carrying, and both move underneath us: ADR
 * walks the DR down in poor coverage, and one LinkADRReq adds FOpts to the
 * next frame. Measured on this node: the 96-byte diagnostic frame owes exactly
 * 60 s at DR3 against a 60 s cadence - zero margin - while the 31-byte frame
 * owes 35 s and is always admitted. At DR5 the same frame owes 18-20 s and
 * nothing happens at all. So a fixed cadence cannot be both legal at DR3 and
 * unwasteful at DR5, and a blanket increase throws away samples at every rate
 * to fix a problem that exists at one.
 *
 * WHAT IT PREVENTS, which is worse than a missed sample: last_uplink and
 * last_poll are stamped BEFORE the duty gate, so a frame the gate refuses is
 * DELETED, not deferred - the measurement is gone and the next attempt is a
 * full interval away. Measured: 66/66 of the 120 s gaps followed a
 * diag-carrying frame and 161/161 of the 60 s gaps did not, i.e. 11.4 deleted
 * samples an hour, 20.9 %. Raising the interval to what the band actually owes
 * removes the refusal at its source rather than handling it.
 *
 * A PLAIN STATIC, deliberately not RTC-backed: a floor carried across a wake
 * would describe a data rate that may no longer be in force. It re-derives on
 * the first uplink after every boot, which is admitted anyway because the duty
 * state itself IS in RTC (rtc_duty_load). Capped at CADENCE_MAX_S so a
 * pathological airtime cannot stall the node indefinitely. */
static uint32_t duty_floor_ms = 0;
/* Measurements destroyed by a duty-cycle refusal since boot. */
static uint32_t duty_deleted_polls = 0;
/* Milliseconds of duty debt the poll is currently deferred for, 0 when none.
 *
 * Published so maybe_sleep() can sleep THROUGH the debt instead of spinning
 * awake for it. Without this the deferral traded a deleted measurement for a
 * battery cost: poll_due is cleared, but maybe_sleep() independently sees the
 * cadence as elapsed and refuses to sleep, so the loop turns over every 50 ms
 * - every 5 ms in Class C - for the whole remaining debt. Fifteen 595 ms
 * frames owe 893 s, which at a 300 s cadence is about ten minutes awake.
 * Found by review 2026-09-10. */
static uint32_t duty_defer_ms = 0;

static void duty_floor_note(uint32_t toa_ms)
{
    uint32_t need_s = duty_need_s(toa_ms);
    if (need_s > CADENCE_MAX_S) need_s = CADENCE_MAX_S;
    const uint32_t was = duty_floor_ms;
    duty_floor_ms = need_s * 1000UL;
    /* Announce a CHANGE only, and only when it actually binds - a cadence
     * silently stretched is the same class of surprise as a sample silently
     * deleted. */
    if (duty_floor_ms != was) {
        const uint32_t asked = cadence_interval_ms(rtc_cadence(), rtc_uptime_s(),
                                                   cad_default_ms());
        if (duty_floor_ms > asked)
            Serial.printf("cadence: the band owes %lu s after a %lu ms frame, "
                          "so the effective interval is %lu s and not the %lu s "
                          "configured\n",
                          (unsigned long)need_s, (unsigned long)toa_ms,
                          (unsigned long)need_s,
                          (unsigned long)(asked / 1000UL));
        else if (was > asked)
            Serial.printf("cadence: back to the configured %lu s - the band now "
                          "owes only %lu s\n",
                          (unsigned long)(asked / 1000UL), (unsigned long)need_s);
    }
}

static uint32_t poll_interval_ms(void)
{
    const uint32_t asked = cadence_interval_ms(rtc_cadence(), rtc_uptime_s(),
                                               cad_default_ms());
    return (duty_floor_ms > asked) ? duty_floor_ms : asked;
}
static uint16_t  uplink_nonce = 0;

/* AWAKE TIME BOUGHT BY TRAFFIC WE DID NOT ACCEPT.
 *
 * Three paths handed out AWAKE_HOLD_MS to any frame that arrived, before or
 * regardless of whether the request was honoured: a refused CTRL frame (an
 * ACK is still produced for malformed ops and stale nonces, and a queued ACK
 * was read as acceptance), any fPort-11 self-update frame, and any fPort-10
 * relay frame. Repeating any of them just inside the window keeps the node
 * awake and Class C RX live indefinitely. That is ~15-33 mW against a ~60 Wh
 * usable AGM: battery life drops from ~240 days to ~76-167.
 *
 * The rule now is that AUTHENTICATED work is unbounded and UNAUTHENTICATED
 * work is not. Inside an open CTRL session the node stays up as long as the
 * session says - that is the legitimate long job. Outside one, unaccepted
 * traffic may buy the node awake a few times, and then stops being able to
 * until something is actually accepted. Bounded, not forbidden: the first
 * frames of a legitimate exchange arrive before any session exists, and a
 * host retrying a genuinely lost frame must still be able to get through. */
#ifndef UNACCEPTED_HOLD_BUDGET
#define UNACCEPTED_HOLD_BUDGET  3
#endif
static uint8_t unaccepted_holds_left = UNACCEPTED_HOLD_BUDGET;

static void hold_awake(uint32_t ms);

/* Something was genuinely accepted: refill the budget. */
static void note_accepted_traffic(void) {
    unaccepted_holds_left = UNACCEPTED_HOLD_BUDGET;
}

/* Awake time for traffic that was NOT accepted. Returns false once spent. */
static bool hold_awake_unaccepted(const char *what) {
    if (unaccepted_holds_left == 0) {
        Serial.printf("%s: not accepted, and the unaccepted-traffic awake "
                      "budget is spent - not holding the radio up\n", what);
        return false;
    }
    unaccepted_holds_left--;
    hold_awake(AWAKE_HOLD_MS);
    return true;
}

/* THE FAST BLE TELEMETRY SCHEDULE, and it is a SEPARATE interval from the one
 * above on purpose.
 *
 * poll_interval_ms() is what the LoRaWAN path is on, and that path is duty
 * limited: the healthy frame is 96 bytes (poll.h:152) and 60 s is 0.9916 % at
 * DR3 without FOpts but 1.0940 % with 15 of them, so 60 s is unconditionally
 * legal only at DR4 and DR5 - see cadence.h, which carries the corrected
 * table and the reason the 60 s floor is an energy bound rather than the legal
 * one. ADR can make it tighter at any time. The BLE stream has no
 * duty-cycle limit at all, so tying the two together would either hold BLE
 * back to a limit that does not apply to it or drag the radio past one that
 * does. They are two schedules over two transports carrying the same frame.
 *
 * NOT IN RTC MEMORY, unlike the cadence lease, and the difference is real: a
 * cadence override has to survive deep sleep because it changes what happens
 * across wakes. A telemetry arming cannot outlive an awake period at all - the
 * BLE link dies with the first deep sleep - so persisting it would only
 * resurrect a stream with no listener. See telemetry.h for why the node stays
 * awake instead of trying to sleep between fast samples. */
static TelemState telem = {};

/* Defined below, beside the loop() that normally drives them. Declared here
 * because the PROTO_BLE_BENCH window in setup() never returns to loop() and so
 * has to call them itself - see the call site for why that is not optional. */
static void telemetry_control(void);
static void telemetry_emit(void);

/* Registered with bleota.cpp so the BLE teardown and this file agree about
 * whether the fast path is running. One predicate, two call sites - the shape
 * ble_ota_in_conn_grace() had to be refactored into after a grace that only
 * one site honoured turned out never to have worked. */
static bool telem_armed_now(void)
{
    return telem_armed(&telem, rtc_uptime_s());
}

static void hold_awake(uint32_t ms) {
    uint32_t until = millis() + ms;
    if ((int32_t)(until - awake_until) > 0) awake_until = until;
}

/* `quiet on` is a lease in RTC_NOINIT memory, not a RAM flag: config.h
 * CONSOLE_QUIET_LEASE_S says why (and which reason turned out false), and
 * rtcstate.h what clears it. */
static bool console_quiet_leased(void) { return rtc_console_quiet_until() != 0u; }
/* The port for the PENDING REPLY only - never a sticky mode.
 *
 * MEASURED 2026-09-06: when this was sticky, the node kept uplinking its
 * heartbeat on fPort 11 after a self-update exchange. The host's framing
 * accepts any payload on the port it is listening to, so a 1-byte heartbeat
 * was indistinguishable from a 9-byte status frame and the next exchange
 * matched garbage. A heartbeat is not a reply and must never wear a reply's
 * port. */
static bool     class_c_active = false;

/* WITHOUT THIS ONE LINE, EVERYTHING BELOW IS DEAD CODE.
 *
 * The Arduino core confirms a freshly installed image ITSELF, in
 * initArduino(), before setup() runs and before a single line of this file
 * executes. cores/esp32/esp32-hal-misc.c, under CONFIG_APP_ROLLBACK_ENABLE
 * (=y in the esp32s3 sdkconfig): if the running image is PENDING_VERIFY and
 * verifyRollbackLater() is false, it calls verifyOta() - a weak hook whose
 * default returns true - and then esp_ota_mark_app_valid_cancel_rollback().
 * cores/esp32/main.cpp calls initArduino() at line 112; setup() runs later,
 * inside the task created at 113.
 *
 * So su_pending_verify() would ALWAYS return false, `pending_verify` would
 * always be false, and su_confirm(), su_reject_and_reboot(), the bench
 * build's "cannot prove the bus" refusal and the 30-minute deadline would all
 * be unreachable. The rollback safety net the whole field-update design rests
 * on would not exist, and nothing would say so: an image that joins but
 * cannot reach the bus would be confirmed automatically and permanently.
 *
 * MEASURED 2026-09-07, and it is what sent us looking: after a rescue push
 * into app0, otadata read ota_state=2 (ESP_OTA_IMG_VALID) on a PROTO_BUS_OFF
 * build that is supposed to decline to confirm. Nothing in this file had
 * confirmed it. initArduino() had, half a second earlier.
 *
 * Returning true tells the core to keep its hands off and leaves the decision
 * here, which is the only place that knows whether the bus answered. */
extern "C" bool verifyRollbackLater(void) { return true; }

/* First boot of a freshly installed image: the bootloader will revert to the
 * previous slot unless we confirm. Review finding F11 - the node is the
 * probe's ONLY field-update master, so an image that joins but cannot reach
 * the bus is a brick for every probe downstream, and must NOT be confirmed. */
static bool     pending_verify = false;
/* The OTA state could not be read at all. Distinct from pending_verify on
 * purpose: it authorises NOTHING. It must not roll the image back (that
 * destroys a working image on the say-so of a failed query) and it must not
 * suppress sleep (that flattens the battery for the same reason). Confirming
 * is still attempted, because marking an already-valid image valid is a
 * no-op, while failing to confirm a genuinely pending one is not. */
static bool     verify_unknown  = false;
static uint32_t verify_deadline = 0;

/* Extra time an unverified image may spend in the join loop when every join
 * failure says the GATEWAY is absent rather than that the image is bad.
 *
 * Bounded on purpose. Without a bound this is just "never roll back", and the
 * safety net stops existing for an image that genuinely cannot join. Six
 * hours comfortably covers the outages actually seen on this bench (over an
 * hour on 2026-09-08) while still giving up inside a day. */
#ifndef JOIN_EXTERNAL_GRACE_S
#define JOIN_EXTERNAL_GRACE_S  (6UL * 60UL * 60UL)
#endif
/* Seconds between join attempts. Named because the grace accounting has to
 * charge it: it is part of what one attempt costs. */
#ifndef JOIN_RETRY_S
#define JOIN_RETRY_S  20UL
#endif
static uint32_t join_grace_left_s   = JOIN_EXTERNAL_GRACE_S;
static bool     join_grace_exhausted = false;

/* Did the bus answer at boot? Set from the ONE probe_answers() call every boot
 * makes, so a clear flag always means "checked, and it answered" rather than
 * the two different things it would mean if the ping were conditional.
 *
 * IT NO LONGER SEPARATES THE TWO FAILURES IT WAS WRITTEN TO SEPARATE. The
 * comment here used to say that a node whose bus was dead from the moment it
 * started otherwise looks exactly like a node whose probe has failed, and that
 * those need opposite people sent to the pile. That was true while the bench
 * bridge answered for itself at address 250. With the bridge MCU retired and a
 * plain transceiver in its place there is nothing on the node's side of the
 * cable that can be asked, so this flag now means "slave 3 did not answer" and
 * covers both cases. It is kept because "the bus was already silent at boot"
 * is still the fact the OTA gate needs; it is no longer a claim about which
 * end failed. See PROBE_PING_ATTEMPTS in config.h. */
static bool     bus_silent_at_boot = false;

/* Did the RS-485 direction pin come up? False until setup() proves
 * it, and false in a PROTO_BUS_OFF build, which never starts the port.
 *
 * Without it the click's R/T stays at its on-board pull-down and the driver is
 * never keyed: every write leaves the node silently, reaches nothing, and the
 * bus looks dead from a fault entirely on this side of the transceiver. That
 * is a configuration failure, not a bus observation, so it is recorded
 * separately and printed as its own cause rather than being folded into
 * "SILENT" with no explanation. */
static bool     bus_direction_ok = false;

/* Give up on an unverified image that cannot prove itself in time.
 *
 * Called from every unbounded retry loop in setup(). Each of those loops can
 * legitimately spin forever on working hardware - a radio that never answers,
 * a network that never accepts a join - and a freshly installed image sitting
 * in one of them has neither confirmed nor rejected itself, so the bootloader
 * is never given a boot at which to roll it back. */
static void rollback_if_overdue(void) {
    if (!pending_verify) return;
    if ((int32_t)(rtc_uptime_s() - verify_deadline) <= 0) return;
    Serial.println("unverified image has not proven itself in 30 min "
                   "- rolling back to the previous slot");
    Serial.flush();
    su_reject_and_reboot();              /* does not return */
}

/* FC03 to the o2-probe at slave 3, registers 0x0000..0x0007 - the primary
 * block. Precomputed, CRC included, so the health check needs no Modbus
 * implementation here: the relay still parses nothing.
 *
 * These are the same eight bytes as POLL_CMDS[0] in poll.h, deliberately
 * duplicated rather than referenced: the poll table is a list that gets
 * reordered and widened as the bus grows, and a health check that silently
 * followed its first entry would one day be pinging an SMT100. The duplication
 * is regression-tested in test/poll_host_test.cpp, which reproduces this CRC
 * from the generator.
 *
 * A CRC-valid 21-byte reply from slave 3 proves the UART, the baud, the
 * parity, the click's direction timing and the probe, all at once - which is
 * strictly more than the bridge's identity register proved, and strictly less
 * useful when it fails, because it cannot say which of them broke. */
static const uint8_t PROBE_PING[] = {0x03, 0x03, 0x00, 0x00, 0x00, 0x08,
                                     0x45, 0xEE};
static const uint8_t PROBE_PING_SLAVE = 0x03;
static const size_t  PROBE_REPLY_LEN = 21;
static const uint8_t PROBE_REPLY_BYTECOUNT = 16;

/* Read one frame: bytes until FRAME_GAP_MS of silence, or REPLY_WAIT_MS with
 * nothing at all. Returns length (0 = the far end said nothing).
 *
 * Silence is reported as zero, never as a partial buffer: a truncated ADU
 * would fail its CRC on the laptop and be blamed on the radio, when the real
 * cause is that nothing answered. */
/* Set whenever an exchange gives up without a reply. Until it passes, the bus
 * may still deliver that reply.
 *
 * MODBUS RTU HAS NO TRANSACTION ID. A device that answers after we stopped
 * waiting produces a frame that is correctly addressed, correctly shaped and
 * correctly CRC'd - an answer to the PREVIOUS question, indistinguishable at
 * both ends from an answer to this one. The concrete case is a relay request
 * timing out at REPLY_WAIT_MS followed immediately by a scheduled poll of the
 * same register block: the late reply lands after the drain and is published
 * as the current oxygen reading.
 *
 * Draining cannot fix that - it removes bytes already buffered, not bytes
 * still in flight. Waiting can: after a timeout, hold off the next request
 * until the whole window in which a late reply could still arrive has passed,
 * so it lands in the quarantine and is drained rather than read as data.
 * Costs REPLY_WAIT_MS, and only after a device has already failed to answer. */
static uint32_t bus_quarantine_until = 0;

static void bus_wait_clear(void) {
    while ((int32_t)(bus_quarantine_until - millis()) > 0) {
        while (Serial1.available()) (void)Serial1.read();
        delay(5);
    }
    while (Serial1.available()) (void)Serial1.read();
}

#if PROTO_BUS
/* THE NODE'S SIDE OF THE PROBATION, hooked into read_frame() below because
 * read_frame() is the ONE place a reply from this pair is assembled - the
 * scheduled poller, relay(), ble_bus_exchange() and the serial CTRL path all
 * funnel through it. Hooking each caller instead would mean four places to
 * remember, and the fifth caller is the one that would be forgotten.
 *
 * Defined with the rest of the `bus` command; declared here because it is
 * needed a hundred lines before that. */
static void bus_prob_note_reply(const uint8_t *frame, size_t n);
#endif

/* Defined with the probe ping below; needed here for read_frame()'s resync,
 * which is only allowed to trim a frame the CRC has authorised. */
static uint16_t modbus_crc16(const uint8_t *b, size_t n);

static size_t read_frame(uint8_t *buf, size_t cap) {
    /* SUBTRACT, DO NOT COMPARE. `millis() < deadline` is wrong across the
     * 49.7-day wrap: entered in the last REPLY_WAIT_MS before it, `deadline`
     * has already wrapped to a small number, the comparison is false on the
     * first pass, and the probe is reported silent without one byte being
     * waited for. The quiet-gap loop just below always used the subtraction
     * form; this one did not. Unsigned subtraction is wrap-correct. */
    const uint32_t started = millis();
    size_t n = 0;
    while (millis() - started < REPLY_WAIT_MS) {
        if (Serial1.available()) {
            uint32_t quiet = millis();
            /* THE DEADLINE BOUNDS THIS LOOP TOO. It did not, and the claim
             * that read_frame() respects REPLY_WAIT_MS was false: only the
             * outer loop tested `started`, so a device dribbling one byte
             * every 19 ms refreshed `quiet` forever and the function ran to
             * about 4865 ms at cap 256 - and everything in the app loop (BLE
             * window expiry, the arm lease check, USB SOF sampling) inherits
             * that delay. Found by review 2026-09-10. */
            while (millis() - quiet < FRAME_GAP_MS &&
                   millis() - started < REPLY_WAIT_MS) {
                while (Serial1.available() && n < cap &&
                       millis() - started < REPLY_WAIT_MS) {
                    buf[n++] = (uint8_t)Serial1.read();
                    quiet = millis();
                }
            }
            /* A FRAGMENT DOES NOT END THE WAIT, and this is 43 % of the
             * exchanges on this bus.
             *
             * The bug this fixes: the loop above commits to FRAME_GAP_MS (20)
             * the instant ANY byte arrives, and there was no path back to the
             * REPLY_WAIT_MS (1500) budget. The turnaround 0x00 lands about
             * 0.5 ms after our own last stop bit on essentially every
             * exchange, so the effective reply window was 20 ms - while the
             * probe's own answer latency reaches ~85 ms, dominated by an
             * ADS1119 conversion it waits out inside its own 400 ms deadline.
             * REPLY_WAIT_MS was therefore DEAD CODE on this bus: the node
             * gave up at 20 ms, returned a 1-byte "reply", and the probe was
             * blamed. Measured 2026-09-10 from the ADU length byte already in
             * every uplink: 4 failures in 16 records, ALL of them alen=1
             * carrying 0x00 - not one CRC failure, not one real silence.
             *
             * So keep the bytes and keep listening on the ORIGINAL deadline.
             * The fragment stays at the front of the buffer, which is exactly
             * what mb_resync() strips when the real reply arrives behind it -
             * the measured capture is one leading 0x00 followed by the
             * 21-byte frame, and that is now assembled across two gap windows
             * instead of being lost between them. */
            /* JUDGED ON WHAT A RESYNC WOULD LEAVE, not on the raw buffer.
             *
             * This tested mb_frame_plausible(buf, n) directly, which is wrong
             * in a way that LOSES replies - the opposite of this branch's
             * purpose. Four leading zeros plus one address byte are five
             * nonzero-containing bytes and passed, mb_resync() then reduced
             * them to a single byte, and the function returned and quarantined
             * the bus - possibly before the real answer had arrived at all. At
             * the far tail, cap-1 leading zeros plus one byte both passed and
             * exhausted capacity. Found by review 2026-09-10.
             *
             * So skip the leading zeros first and ask the question about the
             * remainder, which is the thing a caller will actually be handed. */
            size_t lead_now = 0;
            while (lead_now < n && buf[lead_now] == 0x00u) lead_now++;
            const bool deadline_hit = (millis() - started) >= REPLY_WAIT_MS;
            if (!mb_frame_plausible(buf + lead_now, n - lead_now) &&
                !deadline_hit) {
                /* COMPACT, do not merely reset the all-zero case. Dropping the
                 * leading zeros here is what RECOVERS CAPACITY: 255 zeros
                 * followed by one address byte left lead_now = 255 against
                 * n = 256, so the old `if (lead_now == n) n = 0;` did nothing,
                 * n stayed at cap, every later store was refused, and the real
                 * reply could not be assembled even though it arrived well
                 * inside the deadline. That only delayed the failure to the
                 * timeout. Found by review 2026-09-10.
                 *
                 * Safe because address 0 is the Modbus broadcast address:
                 * request-only, never answered, so a leading zero cannot be
                 * the first byte of any reply. mb_resync() would strip exactly
                 * these bytes later, so doing it now is idempotent. */
                if (lead_now > 0u) {
                    if (lead_now < n) memmove(buf, buf + lead_now, n - lead_now);
                    n -= lead_now;
                }
                delay(1);
                continue;
            }
            /* A DEADLINE EXIT IS AN INCOMPLETE FRAME, and it must stand the bus
             * off even when what arrived happens to look plausible.
             *
             * The three deadline tests added above bound execution, but they
             * can also stop collection MID-FRAME: a continuous 21-byte reply
             * beginning near 1488 ms at 9600 8E1 runs past 1500. Five or more
             * bytes then satisfy mb_frame_plausible(), so the quarantine was
             * skipped - and bus_wait_clear() only drains what is already
             * available, so the next request could transmit into the tail of
             * the reply still arriving. That is a bus collision, which is
             * worse than the late measurement being lost. Found by review
             * 2026-09-10. */
            if (deadline_hit) {
                bus_quarantine_until = millis() + REPLY_WAIT_MS;
                Serial.printf("bus: reply window expired with %u byte(s) - "
                              "incomplete, standing off\n", (unsigned)n);
            }
            /* RESYNC. A REPLY WITH RUBBISH IN FRONT OF IT IS STILL A REPLY.
             *
             * MEASURED 2026-09-10 on this node, at 9600 8E1, three boots in a
             * row, byte for byte identical:
             *
             *   00 03 03 10 FF FF 25 7E 03 48 03 3F 1C E5 00 00 90 00 00 00
             *   A8 86 00
             *
             * Strip the leading 00 and bytes 1..21 are a CRC-VALID FC03 reply
             * from slave 3 - the probe, answering, with a spurious zero byte
             * in front of it and another behind. Every field is shifted by
             * one, so probe_answers() read addr=00 fc=03 count=03, rejected
             * it, and the node reported "bus at boot: SILENT - slave 3 did not
             * answer" while holding the probe's answer in its buffer.
             *
             * THAT LEADING BYTE IS THE TURNAROUND GLITCH 0ac0733 DISMISSED.
             * The pinMode(RX, INPUT_PULLUP) removed there was added to hold
             * the pad at mark while the click's receiver is Hi-Z during
             * transmit; removing it was right (it detaches the UART) but the
             * comment went on to call the concern itself unnecessary because
             * _rxPullEnabled defaults true. The concern was real. The pull-up
             * is not preventing the framing error, and this is what it looks
             * like on the wire.
             *
             * WHY 0x00 IS PROVABLY NOT A REPLY. Unit address 0 is the Modbus
             * broadcast address: it is request-only and nothing ever answers
             * it. serbus.h refuses b[0]==0 on the same grounds. So dropping
             * leading zero bytes cannot discard a real response - there is no
             * response they could be the start of.
             *
             * AND THEN TRIM TO THE DECLARED LENGTH, because the trailing zero
             * matters just as much: poll.h ships each reply UP VERBATIM,
             * "including its own CRC", for the server to check. A 22- or
             * 23-byte ADU carrying a 21-byte frame fails that check at the
             * server exactly as it failed here - so this has been discarding
             * the oxygen data, not merely the boot ping.
             *
             * CONSERVATIVE BY CONSTRUCTION. The trim happens only when the
             * declared length is present AND its CRC is valid; otherwise the
             * buffer is handed on exactly as it arrived. An unparseable frame
             * therefore behaves as it did before this change, and nothing is
             * ever invented: the CRC is what authorises the trim. */
            /* THE RULE LIVES IN mbresync.h, and so does the reasoning.
             *
             * It was inline here until 2026-09-10. It moved because the same
             * fault needed fixing in ee's tools/o2p_update.py and would have
             * needed a third copy in farm-node-decoder.js, and three hand
             * copies of one rule drift. test/mbresync_host_test.cpp runs THIS
             * function against the measured captures in
             * test/fixtures/modbus_captures.json, so the code the test proves
             * is the code that ships. */
            const size_t before = n;
            size_t lead = 0;
            n = mb_resync(buf, n, modbus_crc16, &lead);
            /* Report what was ACTUALLY removed, at both ends. The inline
             * version computed `lead` before deciding not to strip an all-zero
             * buffer, so it could announce a strip that never happened -
             * measured in boot-20260910-010815.log:46, which claimed a dropped
             * byte ahead of a 1-byte reply when nothing had been dropped. */
            if (before != n || lead > 0u)
                Serial.printf("bus: recovered a %u-byte reply (%u leading, %u "
                              "trailing zero byte(s) of turnaround noise)\n",
                              (unsigned)n, (unsigned)lead,
                              (unsigned)(before - lead - n));
#if PROTO_BUS
            /* Evidence, not intention: this is the only path on which the
             * cable can prove a line setting. See busstore.h. */
            bus_prob_note_reply(buf, n);
#endif
            /* QUARANTINE ON A FRAGMENT TOO, not only on silence.
             *
             * bus_quarantine_until was armed on the `return 0` path alone, so
             * a return carrying nothing but the turnaround byte skipped it and
             * the next request went out into a reply that was still arriving.
             * Measured consequence: an uplink showing a 29-register REQUEST
             * answered with the 8-register block - a reply one exchange behind,
             * published as current. The diagnostic block failed 48.8 % of
             * cycles when the primary block succeeded and 95.5 % when it did
             * not, which is that coupling.
             *
             * A fragment means the device is mid-answer, which is exactly the
             * condition the quarantine exists for - MORE so than silence,
             * where nothing may be coming at all. Reached only after the full
             * REPLY_WAIT_MS budget, because the branch above keeps listening
             * until then. */
            if (!mb_frame_plausible(buf, n))
                bus_quarantine_until = millis() + REPLY_WAIT_MS;
            return n;
        }
        delay(1);
    }
    /* Silence, or a fragment that never grew into a frame. Either way the
     * device may still be about to answer, so quarantine the bus for one more
     * full reply window before anything else asks it something. `n` is
     * returned rather than 0 so a caller can tell "nothing at all" from "the
     * turnaround byte and nothing behind it" - probe_answers() already prints
     * the difference, and poll.h's record carries the length either way. */
    bus_quarantine_until = millis() + REPLY_WAIT_MS;
    return n;
}

/* ---- RS-485 DIRECTION, IN SOFTWARE ---------------------------------------
 *
 * The click ties RE to DE and brings the pair out as one R/T pin, so exactly
 * one of driver and receiver is ever live and something must choose.
 *
 * THIS WAS UART_MODE_RS485_HALF_DUPLEX, and on paper that is the better
 * mechanism: the UART raises RTS before the first start bit and drops it after
 * the last stop bit, so it cannot race the transmit FIFO the way software can.
 * MEASURED 2026-09-09 on ESP32-S3 + MIKROE-989 + o2-probe at 9600 8E1: with the
 * hardware mode the port returned one 0x00 per cycle and never a frame, at 9600
 * and at 115200. A control on the same wiring minutes later - plain UART, pin
 * toggled here - returned a CRC-valid 21-byte reply every cycle. Both arms had
 * the RX-detaching pinMode() removed first, so that defect is not what
 * separates them.
 *
 * THE MECHANISM IS NOT UNDERSTOOD, and this comment is not pretending
 * otherwise. uart_ll_set_mode_rs485_half_duplex() writes only sw_rts,
 * rs485tx_rx_en=0, rs485rxby_tx_en=1, irda_en=0 and rs485_en=1, and touches
 * neither the RX pad nor its routing - so "the mode disconnects RX" is not
 * supported by the source. rs485rxby_tx_en, "transmit only while the receiver
 * is idle", is the first bit to test. Until someone does, this file uses what
 * is measured to work rather than what ought to.
 * See ~/dev/kb/esp32/rs485-half-duplex-mode-received-nothing-on-s3.md.
 *
 * THE TIMING RISK THE HARDWARE MODE AVOIDED IS REAL, AND IS BOUNDED HERE.
 * Releasing the driver too early truncates our own last byte on the wire; too
 * late keys it into the first bytes coming back. flush() waits for TX_DONE, not
 * merely for the FIFO, and the margin below is two character times computed
 * from the ACTIVE baud rather than a constant - 2.29 ms at 9600, 190 us at
 * 115200 (integer division; and 11 bits assumes 8E1 - at 8N1 a character is
 * 10 bits, so the constant is 2.2 character times - which is 10 % MORE late
 * release, not 10 % safer; late is the direction that collides, and the cap in
 * bus_write() is what bounds it). Modbus RTU requires a slave to wait 3.5
 * character times before
 * answering, so two is inside the window at every rate this bus supports. */
static uint32_t g_bus_baud = O2P_UART_BAUD;

/* Assert, transmit, drain, release. EVERY write to the bus goes through here.
 * One that bypasses it transmits with the driver disabled and vanishes with no
 * error at all, which on this bus is indistinguishable from a dead cable. */
/* Every write to the pair names WHO asked for it - see bus_write_from(). The
 * node's own writes default to CTRL_ARM_OWNER_NODE, which is also the safe
 * default: node traffic is never presence for an operator's heater arm. */
static void bus_write_from(const uint8_t *buf, size_t len, uint8_t origin);
static void bus_write(const uint8_t *buf, size_t len) {
    bus_write_from(buf, len, CTRL_ARM_OWNER_NODE);
}

static void bus_write_from(const uint8_t *buf, size_t len, uint8_t origin) {
    /* REFUSE IF THE DIRECTION PIN NEVER CAME UP, HERE AND NOT AT THE CALLERS.
     *
     * Only probe_answers() used to check bus_direction_ok. relay(),
     * ble_bus_exchange() and poll_exchange() did not, so a node whose R/T is
     * latched or shorted went on transmitting into a disabled driver every
     * cadence, heard nothing, and uplinked POLL_FLAG_BUS_SILENT - which poll.h
     * defines as a statement about the CABLE. An unevaluable check reported as
     * an evaluated one, on the only channel a buried node has. Gating the one
     * function every write goes through fixes all four at once, and cannot be
     * forgotten by a fifth caller. */
    if (!bus_direction_ok) return;

    /* THE BENCH-OVERRIDE ARM LEASE IS HOOKED HERE, and for the same reason
     * the direction check above is: this is the ONE function every write to
     * the pair goes through, whatever transport asked for it. The reviews of
     * 2026-09-10 both landed on the same sentence - the heater arm's lifetime
     * was tied to the single transport that can see a peer disappear, while
     * the USB bridge and the LoRaWAN relay reach HEATER_TEST_ARM (0x0060)
     * with no BLE authentication and no register allowlist at all. Hooking
     * the four callers instead would mean four places to remember and a fifth
     * to forget.
     *
     * THE OWNER IS PLUMBED, NOT DERIVED - see ctrl_arm_observe() in ctrl.h.
     * It used to be derived from whoever held the pair at the moment of the
     * write, and that made the node's own scheduled polls, during an open
     * CTRL session with no bus claim, indistinguishable from the LoRaWAN
     * operator: every poll refreshed the operator's presence and an absent
     * operator never reached the lease timeout. Review 2026-09-10. The
     * reasoning, the lease and the tri-state are in ctrl.h; this is only the
     * wiring. */
    /* COST: in the common case - no arm held, and a frame that is not one -
     * this is ctrl_arm_scan() alone: a length check, one load and two
     * compares against the function code, which for the poller's FC03 reads
     * falls straight through. rtc_uptime_s() folds a 64-bit accumulator and
     * divides, so it is deliberately NOT called unless there is something to
     * stamp. */
    const uint8_t arm_scan = ctrl_arm_scan(buf, len);
    if (arm_scan == CTRL_ARM_SCAN_ARM || ctrl.arm_state == CTRL_ARM_HELD) {
        ctrl_arm_observe(&ctrl, arm_scan, origin, rtc_uptime_s());
        if (arm_scan == CTRL_ARM_SCAN_ARM) {
            /* NOT ANNOUNCED FROM HERE, deliberately. This point is between
             * the caller's bus_wait_clear() and the transmit below, and a
             * console write in that gap is time in which a late reply to the
             * PREVIOUS exchange can arrive and prefix this one's - the
             * shifted-register failure the drain-then-ask discipline exists
             * to prevent. loop() prints it instead, on the next pass. */
        }
    }

    digitalWrite(PIN_BUS_DE, HIGH);
    delayMicroseconds(50);          /* transceiver enable; spec is sub-us */
    Serial1.write(buf, len);
    Serial1.flush();                /* TX_DONE: FIFO *and* the TX FSM */

    /* TURNAROUND: SHORT, AND NOWHERE NEAR A CONFORMING SLAVE'S START BIT.
     *
     * ONLY A LATE RELEASE COLLIDES. flush() above has already emptied the shift
     * register, so every microsecond here is dead time inside the reply window:
     * an early release cannot truncate anything, and a long margin buys nothing
     * while risking everything.
     *
     * Modbus RTU caps t3.5 at a flat 1.75 ms above 19200, and a slave that
     * implements that flat value at EVERY rate answers 1750 us after our last
     * stop bit. Capping at 1750 was the first attempt and it is wrong: both
     * clocks start at TX_DONE, so it converts a guaranteed 541 us overlap at
     * 9600 into a 1-5 us race against that slave's start-bit edge, and a
     * corrupted start bit loses the frame just as thoroughly. 500 us leaves the
     * margin intact.
     *
     * NOT justified by the o2-probe, which the first version of this comment
     * cited and misread. o2p_mb_rtor_bit_times() (o2-probe modbus_pdu.h:129) is
     * max(ceil(3.5*bits), ceil(0.00175*baud)) = 39 bit times = 4010 us at 9600,
     * so the probe never answers early, and the divergent copies that tree
     * found were too SHORT at high baud - the opposite error. The hazard is a
     * third-party slave: plausible, cheap to insure against, and not evidenced
     * by anything on this cable today. */
    uint32_t margin_us = (2u * 11u * 1000000u) /
                         (g_bus_baud ? g_bus_baud : 9600u);
    if (margin_us > 500u) margin_us = 500u;
    delayMicroseconds(margin_us);

    digitalWrite(PIN_BUS_DE, LOW);  /* listen */
}

/* Configure the direction pin, and PROVE IT MOVES.
 *
 * setMode() at least returned a value to check; a pinMode() does not, and "we
 * called it, so it must be fine" is the assumption that made this bring-up
 * expensive. So drive the pin both ways and read it back. R/T is a plain GPIO -
 * not UART-owned - so digitalRead() on it is meaningful here, unlike on the RX
 * pad, where the same call would detach the port.
 *
 * Leaves it LOW on both paths: receive, driver off. That is what the click's
 * own pull-down selects, so it is also the safe state to fail into - a node
 * that cannot key its driver goes quiet instead of holding the cable. */
static bool bus_direction_begin(uint32_t baud) {
    g_bus_baud = baud;
    pinMode(PIN_BUS_DE, OUTPUT);
    digitalWrite(PIN_BUS_DE, HIGH);
    delayMicroseconds(20);
    bool hi = digitalRead(PIN_BUS_DE);
    digitalWrite(PIN_BUS_DE, LOW);
    delayMicroseconds(20);
    bool lo = digitalRead(PIN_BUS_DE);
    if (!hi || lo) {
        Serial.printf("!! bus: R/T on GP%d does not follow (high reads %d, low "
                      "reads %d) - the driver cannot be keyed, so this node "
                      "cannot transmit on the bus\n", PIN_BUS_DE, hi, lo);
        return false;
    }
    return true;
}

static void relay(const uint8_t *frame, size_t len) {
    /* Drain anything stale before asking: a leftover byte from a previous
     * exchange would prefix this reply and shift every field in it. And wait
     * out any quarantine first - a drain only removes what has already
     * arrived. */
    bus_wait_clear();
    bus_write_from(frame, len, CTRL_ARM_OWNER_LORA);
    uint8_t  rbuf[MAX_FRAME];
    size_t   rlen = read_frame(rbuf, MAX_FRAME);
    Serial.printf("relay: sent %u, got %u\n", (unsigned)len, (unsigned)rlen);
    /* A FRAGMENT IS NOT WORTH AN UPLINK. read_frame() now returns a lone
     * turnaround 0x00 as a 1-byte reply, which `if (rlen)` alone would push
     * as a relayed answer - the server discards it on CRC, so it is a wasted
     * transmission out of a duty-cycle budget that has no spare. Refusing
     * restores exactly the behaviour this path had when read_frame() returned
     * 0 for the same bytes. */
    if (rlen && mb_frame_plausible(rbuf, rlen)) reply_push(rbuf, rlen, O2P_FPORT);
    else if (rlen)
        Serial.printf("relay: %u-byte fragment, not relaying it\n",
                      (unsigned)rlen);
}

/* The same exchange, for the BLE bus characteristic - which needs the REPLY,
 * not an uplink. relay() cannot be reused because it pushes what it got onto
 * the LoRaWAN queue and returns nothing; this returns the frame and pushes
 * nothing. The drain-then-ask discipline and the quarantine timer are the
 * same, and they stay in this file because Serial1 does.
 *
 * App task only. bleota.cpp calls it from ble_ota_tick(), never from the
 * NimBLE host task, because read_frame() blocks for up to REPLY_WAIT_MS. */
/* THE CLAIM CACHE. The rule, the reasoning and the risk are in pollcache.h,
 * which is a separate header precisely so the request-matching can be host
 * tested away from Arduino.h; this is only the wiring.
 *
 * App task only - claim_cache_note() is reached from ble_bus_exchange(), which
 * bleota.cpp calls from ble_ota_tick(), and claim_cache_xchg() from loop().
 * Same task, so no locking. */
static PollCache claim_cache = {};

/* One poll interval, and it follows the cadence rather than pinning a number:
 * if the operator has slowed the node to 15 minutes, a 6-minute-old reading is
 * still fresher than the cadence, and if they have sped it up to 60 s the
 * bound tightens with it. */
static size_t claim_cache_xchg(const uint8_t *req, size_t req_len,
                               uint8_t *rsp, size_t cap)
{
    return poll_cache_lookup(&claim_cache, req, req_len, rsp, cap,
                             millis(), poll_interval_ms());
}

static size_t ble_bus_exchange(const uint8_t *req, size_t req_len,
                               uint8_t *rsp, size_t cap) {
    bus_wait_clear();
    bus_write_from(req, req_len, CTRL_ARM_OWNER_BLE);
    size_t rlen = read_frame(rsp, cap);
    Serial.printf("ble bus: sent %u, got %u\n",
                  (unsigned)req_len, (unsigned)rlen);
    /* The operator's read is also this cycle's measurement, if it happened to
     * be one of the two questions the poller asks. See pollcache.h. */
    (void)poll_cache_note(&claim_cache, req, req_len, rsp, rlen, millis());
    return rlen;
}

/* What the BLE channel asks before it will drive the bus at all. See
 * bleota.h: a run of Modbus exchanges is one transaction and this firmware
 * cannot see that, so the claim - which is also what the uplink reports as
 * POLL_FLAG_BUS_CLAIMED - is the thing that stops a scheduled poll landing in
 * the middle of a firmware push. */
static bool ble_bus_claimed(void) { return ctrl.bus_claimed; }

/* The bus side of the poller. Same drain-then-ask discipline as relay(): a
 * byte left over from a previous exchange would prefix this reply and shift
 * every register in it, which decodes as a plausible wrong oxygen number
 * rather than as an error. */
static size_t bus_exchange_from(const uint8_t *req, size_t req_len,
                                uint8_t *rsp, size_t cap, uint8_t origin) {
    bus_wait_clear();
    bus_write_from(req, req_len, origin);
    return read_frame(rsp, cap);
}
/* The NODE's exchange, and the one poll_build_frame() takes as its function
 * pointer - which is why the origin is not simply a parameter here. */
static size_t poll_exchange(const uint8_t *req, size_t req_len,
                            uint8_t *rsp, size_t cap) {
    return bus_exchange_from(req, req_len, rsp, cap, CTRL_ARM_OWNER_NODE);
}

/* ---- CTRL OVER THE USB CONSOLE -----------------------------------------
 *
 * Mac -> USB serial -> node -> RS-485 -> probe, with tools/o2p_update.py
 * unmodified at the far end. The reasoning lives in serbus.h (why the bridge
 * has to be transparent), console.h (why logging has to stop) and
 * ctrl_serial_claim() (why the claim expires on its own); this is the glue
 * that owns Serial, Serial1 and `ctrl`, and it deliberately holds no policy
 * of its own.
 *
 * ONE EXCHANGE PER LOOP PASS. poll_exchange() blocks for up to REPLY_WAIT_MS,
 * and draining a whole USB packet of queued frames back-to-back would stall
 * everything else - including the Class C downlink poll, which RadioLib
 * overwrites rather than queues. The host tool is synchronous anyway: it
 * waits for each reply before sending the next request.
 */
#if PROTO_BUS
static SerbusRx serbus_rx;
static uint32_t serbus_last_byte_ms = 0;
static uint32_t serbus_hush_until   = 0;
static bool     serbus_hushed       = false;
static uint32_t serbus_frames       = 0;   /* forwarded */
static uint32_t serbus_refused      = 0;   /* someone else held the bus */
#endif

/* ==== `bus` : THE RS-485 LINE SETTINGS, AT RUN TIME ======================
 *
 * WHY THIS EXISTS, and why a compile-time O2P_UART_BAUD cannot do its job.
 *
 * THE PROBE ACCEPTS A NEW BAUD_CODE ONLY ON PROBATION. o2-probe
 * firmware/src/serial_probation.h:59 sets O2P_SERIAL_PROBATION_MS = 10000,
 * and serial_probation.h:130 puts the OLD transport back unless a frame
 * arrives at the NEW rate inside those ten seconds. That timer is exactly
 * what makes trying 115200 safe on a pair whose length, termination and error
 * rate at that rate have never been measured - regmap.h says so in as many
 * words ("NOT VERIFIED ON THE DEPLOYED CABLE"): a rate the cable cannot carry
 * un-does itself, with no site visit.
 *
 * The other half of that bargain is that THE MASTER MUST BE ABLE TO FOLLOW
 * THE PROBE ONTO THE NEW RATE INSIDE THE WINDOW, and a rate compiled into
 * this image cannot. The cheapest path from "wrote BAUD_CODE" to "this node
 * speaks at the new rate" would be edit platformio.ini, rebuild, flash, boot:
 * minutes against a ten-second deadline. Every attempt would end in the probe
 * reverting, and - this is the part that makes it worse than merely slow -
 * the operator could not tell a reverted probation from a cable that cannot
 * carry the rate, because both present as the same silence. BAUD_CODE would
 * be a register this bench is unable to write. So the rate has to be typeable
 * while the window is open, which is what this command is.
 *
 * PERSISTED ONLY ONCE THE CABLE HAS ANSWERED - CORRECTED 2026-09-09, and the
 * superseded reasoning is kept because it was half right and the half it got
 * wrong is the dangerous half.
 *
 * WHAT THIS FILE USED TO SAY: "NOT PERSISTED, AND THAT IS THE POINT. [...] it
 * means a rate that turns out not to work is one power cycle from gone." That
 * is true, and it is a real safety property, and it is not the whole trade.
 *
 * WHAT IT MISSED: the rate that DOES work is the dangerous one. A frame that
 * reaches the probe at the new rate satisfies the probe's probation, and the
 * probe COMMITS - permanently, in its own flash. A node holding the rate only
 * in a live UART then reverts at the next deep-sleep wake, and from that
 * moment every scheduled poll is addressed to a probe that is no longer
 * listening at that rate. The symptom is silence, which this firmware already
 * reports as "probe did not answer" with nothing naming the cause. At the
 * bench that is a nuisance - retype the command. On a BURIED probe it is
 * unattended, permanent data loss: nothing on the LoRaWAN path can change a
 * UART rate, so recovery is a laptop at the cable or a reflash.
 *
 * BOTH HAZARDS ARE REAL AND THEY POINT OPPOSITE WAYS, so neither "always
 * persist" nor "never persist" is correct. The rule that resolves them is the
 * probe's own, mirrored: PERSIST ONLY WHAT THE CABLE HAS ANSWERED AT. This
 * command arms a node-side probation (BUS_PROBATION_MS, deliberately equal to
 * the probe's O2P_SERIAL_PROBATION_MS) and bus_store_put() is called from
 * exactly one place - bus_prob_note_reply(), on a CRC-valid reply at the new
 * rate. An unusable rate never reaches flash and is reverted by
 * bus_prob_tick(). The stored value is therefore always a measurement rather
 * than an intention, which is what makes it safe to boot from unattended.
 *
 * THE STRONGER ORDERING CLAIM THAT STOOD HERE IS WITHDRAWN - it read that a
 * rate the probe committed to "is in flash BEFORE the probe could have
 * committed to it". Backwards: the probe commits on receiving our request, we
 * store on receiving its reply. busstore.h carries the correction and what is
 * actually guaranteed.
 *
 * OPEN DEFECTS IN THE MACHINERY BELOW, all found by review on 2026-09-09 and
 * none of them fixed - written here because the code reads as if they were:
 * the deadline is not checked before committing and setup()'s serbus_tick()
 * loops never tick probation (7); a failed NVS store clears probation with no
 * retry, and re-typing the same pair is a no-op (5); a second change before
 * the first is proven overwrites the rollback destination with an unproven
 * setting (6); bus_prob_tick()'s revert does not consult
 * bus_line_busy_reason() and can reconfigure the UART inside someone else's
 * transaction (8); the reply is authenticated but not attributed (3). See
 * docs/2026-09-09-bus-baud-review.md.
 *
 * A change takes hold_awake(AWAKE_HOLD_MS) - 60 s against a 10 s probation -
 * because a node that deep-sleeps four seconds after the switch would revert
 * before the operator could send anything at the new rate, and would take the
 * probation timer down with it.
 */
#if PROTO_BUS
static const char *bus_parity_name(uint8_t p)
{
    switch (p) {
    case CONSOLE_BUS_PARITY_NONE: return "8N1";
    case CONSOLE_BUS_PARITY_ODD:  return "8O1";
    case CONSOLE_BUS_PARITY_EVEN: return "8E1";
    default:                      return "8?1";
    }
}

static uint32_t bus_parity_config(uint8_t p)
{
    switch (p) {
    case CONSOLE_BUS_PARITY_NONE: return SERIAL_8N1;
    case CONSOLE_BUS_PARITY_ODD:  return SERIAL_8O1;
    default:                      return SERIAL_8E1;
    }
}

/* READ THE PERIPHERAL, NOT A VARIABLE. A `bus` that printed back whatever the
 * last `bus baud` was asked for would answer the question "what did I type",
 * and the question worth asking is "what is on the pair" - those differ
 * exactly when a re-begin() half failed, which is the failure this command
 * can produce and the one a setting you cannot query would hide.
 *
 * `*raw` IS THE DIVISOR READING AND IS NOT ALWAYS THE NOMINAL RATE.
 * HardwareSerial.h:42 states it plainly - "the baudrate returned by
 * baudRate() may be rounded, eg 115200 returns 115201" - because the APB
 * divisor is an integer. So `*nominal` is `*raw` snapped to the legal rate it
 * is within 2% of, and the caller prints the raw figure too whenever the two
 * differ, rather than quietly presenting one as the other. Nothing snaps if
 * the reading is not near a legal rate: an unrecognised rate must stay
 * visibly unrecognised. */
static bool bus_line_read(uint32_t *nominal, uint32_t *raw, uint8_t *parity)
{
    uint32_t b = 0;
    uart_parity_t p = UART_PARITY_DISABLE;
    if (uart_get_baudrate(UART_NUM_1, &b) != ESP_OK) return false;
    if (uart_get_parity(UART_NUM_1, &p)   != ESP_OK) return false;

    *raw     = b;
    *nominal = b;
    const uint32_t *legal = console_bus_baud_table();
    for (unsigned i = 0; i < CONSOLE_BUS_BAUD_N; i++) {
        const uint32_t n = legal[i];
        const uint32_t d = (b > n) ? (b - n) : (n - b);
        /* DIVIDE, DO NOT MULTIPLY. `d * 50u <= n` was the obvious spelling and
         * it overflows: a garbage divisor reading - the exact case this
         * function promises to leave visibly unrecognised - makes d large
         * enough that d * 50 wraps uint32_t to a small number and SNAPS onto a
         * legal rate. The guard would then dress up an unreadable peripheral
         * as a clean answer, which is the one thing a read-back must never do.
         * n / 50 cannot wrap, and for every rate in the table it is exact
         * (9600/50 = 192 = 2 % of 9600). */
        if (d <= n / 50u) { *nominal = n; break; }   /* within 2% */
    }
    *parity = (p == UART_PARITY_EVEN) ? CONSOLE_BUS_PARITY_EVEN
            : (p == UART_PARITY_ODD)  ? CONSOLE_BUS_PARITY_ODD
                                      : CONSOLE_BUS_PARITY_NONE;
    return true;
}

/* THE READ-BACK. Prints bus_direction_ok as well as the line settings,
 * because a reconfigure that lost setPins()/setMode() leaves a node that
 * cannot key the driver at all - a fault the baud and parity alone would
 * describe as perfectly healthy. */
static void bus_line_report(const char *lead)
{
    uint32_t nominal = 0, raw = 0;
    uint8_t  parity  = 0;
    if (!bus_line_read(&nominal, &raw, &parity)) {
        Serial.printf("%s: UART1 would not report its line settings - "
                      "treat the rate below as UNKNOWN, not as the default\n",
                      lead);
        return;
    }
    if (raw != nominal)
        Serial.printf("%s: %lu baud %s (divisor reads %lu; see "
                      "HardwareSerial.h:42), direction %s\n",
                      lead, (unsigned long)nominal, bus_parity_name(parity),
                      (unsigned long)raw,
                      bus_direction_ok ? "OK" : "FAILED - cannot transmit");
    else
        Serial.printf("%s: %lu baud %s, direction %s\n",
                      lead, (unsigned long)nominal, bus_parity_name(parity),
                      bus_direction_ok ? "OK" : "FAILED - cannot transmit");
}

/* ALL OF setup()'s STEPS, IN setup()'s ORDER, EVERY TIME.
 *
 * THE ORDER USED TO MATTER AND NO LONGER DOES. This paragraph described
 * begin() -> setPins() -> setMode() as a sequence that had to be repeated in
 * full on every re-configure, because begin() alone silently dropped the RTS
 * assignment that keyed the click's joined RE/DE. None of that survives: this
 * function calls neither setPins() nor setMode(), R/T is a plain GPIO, and
 * bus_direction_begin() below re-establishes it explicitly
 * subsequent transmission into no transmission - an intermittently silent
 * probe, the hardest symptom on this pair to attribute. So this function is
 * the ONLY way the rate is allowed to change, and it mirrors setup() line for
 * line. IF setup() EVER GAINS ANOTHER STEP, IT BELONGS HERE TOO.
 *
 * IT WAS FOUR STEPS UNTIL 0ac0733, and the fourth was actively harmful. This
 * comment used to name pinMode(RX, INPUT_PULLUP) as a required step and argue
 * that losing it "puts a spurious start bit in front of replies during
 * turnaround". Measured on hardware the same day: on arduino-esp32 3.x a
 * pinMode() on a peripheral-owned pin DETACHES THE UART
 * (perimanClearPinBus() -> _uartDetachBus_RX()), so Serial1.available() is 0
 * forever afterwards. The pull-up that step was reaching for is already
 * applied by begin()/setPins() via _uartApplyRxPull(), _rxPullEnabled
 * defaulting true. Use Serial1.enableRxInternalPull(bool) BEFORE begin() if it
 * ever needs changing - never pinMode() here. See 0ac0733 for the bring-up
 * that paid for this.
 *
 * end() FIRST, as src/bustest.cpp:232 does when it sweeps rates: begin() on
 * an already-installed UART is not a documented reconfigure, and the sweep
 * that is known to change rates successfully on this board tears down first.
 *
 * THE GAP end() OPENS DOES NOT KEY THE BUS, and that is a property of the
 * board rather than of the timing. THE MECHANISM HERE CHANGED on 2026-09-09
 * and the guarantee got STRONGER, so this is a rewrite rather than a deletion:
 * end() no longer drops an RTS assignment, because R/T is not UART-owned any
 * more - it is a plain GPIO this file drives, and end() does not touch it. The
 * pad therefore stays a DRIVEN output at LOW across the whole teardown, rather
 * than falling back on the click's own pull-down on the joined RE/DE pair -
 * which is a STRONGER guarantee than this paragraph used to claim, not a
 * weaker one. So for the whole of the teardown the driver is off and the
 * receiver is on: this node goes deaf and mute for a few hundred microseconds,
 * it does not shout over whoever else is on the cable.
 *
 * ONE HONEST EXCEPTION, since the rest of this paragraph is absolute:
 * bus_direction_begin() below drives R/T HIGH for ~20 us to prove the pad
 * moves. So a reconfigure - and every boot - does key the driver briefly, into
 * an idle-mark TX pad, on a cable the Dragino also masters and which our own
 * quarantine knows nothing about. 20 us is far inside any inter-frame gap, but
 * it is not nothing, and it is not covered by the sentence above. Any bytes that arrive in that gap are lost,
 * which is why bus_wait_clear() is called first - after it, nothing is owed a
 * reply.
 *
 * bus_direction_ok is updated, not merely consulted: it is what probe_answers()
 * checks before vouching for a self-update image, and leaving a stale `true`
 * behind a failed setPins() would let an un-keyable transceiver confirm an
 * image it cannot reach the bus to test. */
static bool bus_line_apply(uint32_t baud, uint8_t parity)
{
    /* Wait out any quarantine and drain, exactly as every other user of this
     * pair does before it speaks. A reply still in flight from a request that
     * timed out is bytes at the OLD rate; after the switch they are noise,
     * and noise ahead of the next reply shifts every field in it. Reusing
     * bus_wait_clear() rather than inventing a settling delay also means this
     * command honours the same REPLY_WAIT_MS window relay() and the poller
     * honour. */
    bus_wait_clear();

    Serial1.end();
    Serial1.begin(baud, bus_parity_config(parity), PIN_BUS_RX, PIN_BUS_TX);
    /* The rate moved, so the turnaround margin must move with it: it is two
     * character times at the ACTIVE baud, and a margin left at the old rate is
     * either wasted airtime or a truncated last byte. */
    bus_direction_ok = bus_direction_begin(baud);
    /* NO pinMode() ON THE RX PAD HERE. IT MAKES THE UART DEAF.
     *
     * On arduino-esp32 3.x, pinMode() on a peripheral-owned pin calls
     * perimanClearPinBus() -> _uartDetachBus_RX(), which points U1RXD at
     * GPIO_FUNC_IN_HIGH and sets uart->_rxPin = -1. From that instant
     * Serial1.available() is 0 forever, whatever the pad, the click, the cable
     * or the probe do - and digitalRead() still reads the pad correctly, so
     * every GPIO-level diagnostic keeps looking healthy. That combination cost
     * the 2026-09-09 bring-up most of a night: a scan of all 247 Modbus
     * addresses, four baud rates, both parities and six pin orderings, every
     * one of them run on a UART that had been disconnected from its pin.
     *
     * The line it replaced was added to hold the pad at mark while the click's
     * receiver is Hi-Z during transmit. That concern is real but the fix was
     * unnecessary: _rxPullEnabled defaults true, so begin()/setPins() already
     * apply the pull-up via _uartApplyRxPull(). If it ever needs changing, use
     * Serial1.enableRxInternalPull(bool) BEFORE begin(). */
    return bus_direction_ok;
}

/* ---- THE NODE'S PROBATION ----------------------------------------------
 *
 * Mirrors the probe's, and for the same reason: a line setting is a claim
 * about a cable, and the only thing that can settle it is the cable. The rule
 * is one sentence - A SETTING IS COMMITTED TO FLASH WHEN, AND ONLY WHEN, A
 * CRC-VALID REPLY ARRIVES AT IT - and everything below is that sentence plus
 * what happens when it does not.
 *
 * WHY CRC AND NOT "SOME BYTES CAME BACK". probe_answers() already argues this
 * at length for the self-update gate: length, address and function code are
 * three bytes that noise on an RS-485 pair satisfies by accident, and at a
 * WRONG baud rate noise is exactly what read_frame() collects - a mistimed
 * sample of a real transmission is bytes, plausibly shaped ones. Committing on
 * that would write the unreachable rate into flash and defeat the whole
 * mechanism, so the evidence has to be authenticated rather than recognised.
 *
 * WHAT THE REVERT CAN AND CANNOT FIX, stated because it is a partial remedy
 * and reading it as a complete one would be worse than not having it. If no
 * reply arrives we put OUR side back, which restores the working case where
 * the probe also reverted (it saw nothing at the new rate either). It does NOT
 * cover the split case: the probe received our frame and committed, but its
 * reply was lost or corrupt, so it is on the new rate and we went back to the
 * old one. Nothing this node can do detects that - both cases are silence -
 * so the revert message names the possibility explicitly rather than
 * announcing a recovery it cannot guarantee. */
static uint16_t modbus_crc16(const uint8_t *b, size_t n);   /* defined below */

static bool     bus_prob_active     = false;
static uint32_t bus_prob_until      = 0;
static uint32_t bus_prob_new_baud   = 0;
static uint8_t  bus_prob_new_parity = 0;
static uint32_t bus_prob_old_baud   = 0;
static uint8_t  bus_prob_old_parity = 0;

static void bus_prob_arm(uint32_t old_baud, uint8_t old_parity,
                         uint32_t new_baud, uint8_t new_parity)
{
    /* NOTHING TO PROVE when the setting did not move. Arming anyway would
     * schedule a revert to the value we are already at - harmless, but it
     * would also print a "reverted" line for an episode that never happened,
     * and a log that reports events that did not occur is worse than a quiet
     * one. */
    if (old_baud == new_baud && old_parity == new_parity) return;
    bus_prob_active     = true;
    bus_prob_until      = millis() + BUS_PROBATION_MS;
    bus_prob_new_baud   = new_baud;
    bus_prob_new_parity = new_parity;
    bus_prob_old_baud   = old_baud;
    bus_prob_old_parity = old_parity;
}

static void bus_prob_note_reply(const uint8_t *frame, size_t n)
{
    if (!bus_prob_active) return;
    /* Two CRC bytes plus at least an address and a function code. */
    if (n < 4) return;
    const uint16_t crc = modbus_crc16(frame, n - 2);
    if ((crc & 0xFF) != frame[n - 2] || (crc >> 8) != frame[n - 1]) return;

    /* OVERSTATED, AND KNOWN TO BE - the line below says "the probe", and this
     * function checked a CRC, not an address. Any device on the pair satisfies
     * it. Left as-is only because changing the wording without adding the
     * address check would move the overstatement rather than remove it; see
     * docs/2026-09-09-bus-baud-review.md, finding 3. */
    bus_prob_active = false;
    Serial.printf("bus: PROVEN - a CRC-valid reply arrived at %lu %s, so the "
                  "cable carries it. Storing it as this node's line setting.\n",
                  (unsigned long)bus_prob_new_baud,
                  bus_parity_name(bus_prob_new_parity));
    if (!bus_store_put(bus_prob_new_baud, bus_prob_new_parity)) {
        /* bus_store_put() has already said what failed and why. What it cannot
         * say is the consequence in this specific episode, which is the split:
         * the probe has committed - our evidence IS its reply - and this node
         * has not. */
        Serial.printf("!! bus: the probe has now COMMITTED to %lu %s and this "
                      "node did not store it. At the next wake this node comes "
                      "back at %lu %s and the probe will not answer. Set "
                      "O2P_UART_BAUD to match and reflash before leaving.\n",
                      (unsigned long)bus_prob_new_baud,
                      bus_parity_name(bus_prob_new_parity),
                      (unsigned long)bus_stored_baud(),
                      bus_parity_name(bus_stored_parity()));
    }
}

/* Called from loop(). Cheap, and unconditional: a probation that only ticked
 * while something else was happening would hold past its deadline exactly on
 * the quiet bus where the revert matters. */
static void bus_prob_tick(void)
{
    if (!bus_prob_active) return;
    if ((int32_t)(bus_prob_until - millis()) > 0) return;   /* wrap-correct */

    bus_prob_active = false;
    Serial.printf("bus: probation EXPIRED - nothing on the cable answered at "
                  "%lu %s within %lu ms. Reverting this node to %lu %s.\n",
                  (unsigned long)bus_prob_new_baud,
                  bus_parity_name(bus_prob_new_parity),
                  (unsigned long)BUS_PROBATION_MS,
                  (unsigned long)bus_prob_old_baud,
                  bus_parity_name(bus_prob_old_parity));
    Serial.println("bus: NOT stored, so nothing survives this. If the probe "
                   "DID receive a frame at the new rate and only its reply was "
                   "lost, it has committed and this node has not - that split "
                   "is silence at both ends and this firmware cannot tell it "
                   "from a rate the cable cannot carry. Re-run `bus baud` at "
                   "the new rate and send a frame if polls stay silent.");
    (void)bus_line_apply(bus_prob_old_baud, bus_prob_old_parity);
    bus_line_report("bus: back at");
}

/* WHO OWNS THE PAIR RIGHT NOW - and the rule is serbus_tick()'s, not a new one.
 *
 * ctrl_serial_claim() refuses a forwarded frame on exactly one condition,
 * ctrl.cpp: `if (st->open && !st->serial_claim) return false;` - somebody
 * else's session owns the bus. This restates that condition and nothing else,
 * so the line rate cannot be pulled out from under a BLE or LoRaWAN firmware
 * push, which is the multi-frame transaction this firmware cannot see the
 * boundaries of and the reason CTRL_F_CLAIM_BUS exists.
 *
 * WHY IT RESTATES THE TEST INSTEAD OF CALLING ctrl_serial_claim(). Calling it
 * would also TAKE a claim, for SERBUS_HOLD_MS = 30 s, and keep the poller off
 * the pair for all of it. The probation window this command exists to chase
 * is 10 s (O2P_SERIAL_PROBATION_MS). Taking a 30-second claim as a side
 * effect of a configuration command would therefore guarantee the exact
 * outcome the command is meant to prevent.
 *
 * WHY OUR OWN SERIAL CLAIM IS NOT A REFUSAL, stated plainly because it is the
 * one permissive case here. ctrl.h: "The operator at the cable is closer to
 * the probe than the operator at the network server, and this is how the
 * firmware says so." A `bus baud` line arrives on the same cable as that
 * claim, from that same operator. It also HAS to be allowed: writing
 * BAUD_CODE through this bridge sets serial_claim and refreshes it for 30 s
 * after the last frame, so refusing while it is held would put the command
 * out of reach for three times the probation window every single time it is
 * needed. The cost is real and is not hidden: a rate change between two
 * frames of a push in progress will break that push, so a warning is printed
 * and the operator - who is holding both ends - decides.
 *
 * NOT CHECKED HERE: ble_ota_flash_busy(). serbus_tick() returns before any
 * console byte is read when a flash erase is running, so no command reaches
 * this point during one; a second test would be dead code that reads as a
 * guarantee. */
static const char *bus_line_busy_reason(void)
{
    if (ctrl.open && !ctrl.serial_claim)
        return ctrl.ble_claim ? "a BLE CTRL session holds the RS-485 pair"
                              : "a LoRaWAN CTRL session holds the RS-485 pair";
    return NULL;
}

/* The dispatcher. Takes an ALREADY-PARSED command, never a raw line: the
 * caller has to know whether the line was a `bus` command before it decides
 * to un-hush the console for the answer, and parsing it twice to find out
 * would put the "is this a command" rule in two places. `kind` is never
 * CONSOLE_BUS_NONE here - the caller filtered that out. */
static void bus_cmd_run(const ConsoleBusCmd &c)
{
    if (c.kind == CONSOLE_BUS_REFUSED) {
        Serial.println(c.err);
        bus_line_report("bus: unchanged");
        return;
    }
    if (c.kind == CONSOLE_BUS_SHOW) {
        bus_line_report("bus");
        /* WHERE THE SETTING CAME FROM, not just what it is. "9600 because
         * nothing is stored" and "9600 because the cable answered at 9600" are
         * different facts that print as the same two numbers, and only the
         * second one survives a wake. */
        if (bus_store_is_set())
            Serial.printf("bus: STORED (%lu %s) - proven on the cable and "
                          "restored at every boot\n",
                          (unsigned long)bus_stored_baud(),
                          bus_parity_name(bus_stored_parity()));
        else
            Serial.printf("bus: nothing stored - this node boots at the "
                          "compiled %lu 8E1 until a setting is proven\n",
                          (unsigned long)O2P_UART_BAUD);
        if (bus_prob_active)
            Serial.printf("bus: PROBATION open - %ld ms left to prove %lu %s\n",
                          (long)(int32_t)(bus_prob_until - millis()),
                          (unsigned long)bus_prob_new_baud,
                          bus_parity_name(bus_prob_new_parity));
        Serial.println(CONSOLE_BUS_USAGE);
        return;
    }

    const char *busy = bus_line_busy_reason();
    if (busy) {
        Serial.printf("bus: REFUSED - %s. Changing the line rate under an "
                      "open transaction corrupts it; retry when the session "
                      "closes or expires.\n", busy);
        bus_line_report("bus: unchanged");
        return;
    }

    /* Read the CURRENT pair of settings first: changing one must carry the
     * other across unchanged, and re-begin() needs both. Taken from the
     * peripheral for the same reason bus_line_report() takes them from there. */
    uint32_t nominal = 0, raw = 0;
    uint8_t  parity  = 0;
    if (!bus_line_read(&nominal, &raw, &parity)) {
        Serial.println("bus: REFUSED - UART1 will not report its current "
                       "settings, so the other half of the pair cannot be "
                       "carried across. Nothing changed.");
        return;
    }

    const uint32_t want_baud   = (c.kind == CONSOLE_BUS_SET_BAUD)   ? c.baud   : nominal;
    const uint8_t  want_parity = (c.kind == CONSOLE_BUS_SET_PARITY) ? c.parity : parity;

    if (ctrl.serial_claim)
        Serial.println("bus: WARNING - a USB-serial bus claim is open on this "
                       "same cable. If a push is in progress it will break "
                       "here; the claim is yours, so this is not refused.");

    Serial.printf("bus: %lu %s -> %lu %s\n",
                  (unsigned long)nominal, bus_parity_name(parity),
                  (unsigned long)want_baud, bus_parity_name(want_parity));
    /* The return value is deliberately dropped: bus_line_apply() has already
     * printed the specific failure, and bus_line_report() below prints the
     * direction state either way. Two spellings of the same bad news read as
     * two faults. */
    (void)bus_line_apply(want_baud, want_parity);
    bus_line_report("bus: now");
    /* ARM AFTER THE APPLY, not before: if bus_line_apply() failed at
     * setPins()/setMode() the node cannot transmit, so no reply can arrive, so
     * the probation would expire and revert - which is the correct outcome and
     * is why arming is not conditional on the apply succeeding. Ordering it
     * after simply keeps the deadline measured from the moment the new rate
     * was actually live. */
    bus_prob_arm(nominal, parity, want_baud, want_parity);
    /* BOTH SIDES OF THE SAME WINDOW, spelled out where the operator is looking
     * when they need it. */
    Serial.println("bus: if the probe is on probation (O2P_SERIAL_PROBATION_MS "
                   "= 10 s) it reverts unless a frame reaches it at this rate "
                   "inside that window - send one now.");
    Serial.printf("bus: this node is on probation too - it stores this setting "
                  "only when a CRC-valid reply arrives at it, and reverts to "
                  "%lu %s in %lu ms otherwise. Sending that frame is what "
                  "makes the change survive a deep-sleep wake.\n",
                  (unsigned long)nominal, bus_parity_name(parity),
                  (unsigned long)BUS_PROBATION_MS);
    hold_awake(AWAKE_HOLD_MS);
}
#endif  /* PROTO_BUS */

/* `caps` on the USB console. Every line but `state` comes from caps.h, so this
 * transport and BLE cannot describe different images. The end line counts
 * every line printed, itself included. */
static void caps_print_serial(void)
{
    char line[160];
    unsigned n = 0;
    for (unsigned i = 0; caps_line(CAPS_SERIAL, i, line, sizeof(line)); i++, n++)
        Serial.printf("caps: %s\n", line);
    const int32_t  hold_ms = (int32_t)(awake_until - millis());
    const uint32_t q_until = rtc_console_quiet_until();
    const uint32_t now_s   = rtc_uptime_s();
    Serial.printf("caps: state awake_hold_left_s=%lu quiet_left_s=%lu ble_up=%d\n",
                  (unsigned long)(hold_ms > 0 ? (uint32_t)hold_ms / 1000u : 0u),
                  (unsigned long)(q_until ? q_until - now_s : 0u),
                  ble_ota_is_up() ? 1 : 0);
    n++;
    Serial.printf("caps: end lines=%u\n", n + 1u);
}

/* `caps` and `quiet`. console.h says what they do, and why neither has any
 * effect or output while a serial bus session holds the pair. */
static void console_verb_run(const ConsoleVerbCmd &vc)
{
    if (ctrl.serial_claim) return;
    switch (vc.kind) {
    case CONSOLE_VERB_CAPS:
        nodecon.set_quiet(false);
#if PROTO_BUS
        serbus_hushed = false;
#endif
        caps_print_serial();
        break;
    case CONSOLE_VERB_QUIET_ON: {
        rtc_console_quiet_set(rtc_uptime_s() + (uint32_t)CONSOLE_QUIET_LEASE_S);
        char msg[160];
        snprintf(msg, sizeof(msg),
                 "quiet: on for %lu s - log output off, across resets; "
                 "`quiet off` restores it", (unsigned long)CONSOLE_QUIET_LEASE_S);
        nodecon.say_anyway(msg);
        break;
    }
    case CONSOLE_VERB_QUIET_OFF:
        rtc_console_quiet_set(0u);
        nodecon.set_quiet(false);
#if PROTO_BUS
        serbus_hushed = false;
#endif
        Serial.println("quiet: off - log output restored");
        break;
    case CONSOLE_VERB_REFUSED:
        nodecon.say_anyway(vc.err);
        break;
    default:
        break;
    }
    if (console_quiet_leased()) nodecon.set_quiet(true);
}
/* THE GATE FOLLOWS THE LEASE, which ends by itself (review 2026-09-10: a lease
 * that expired while the console was not hushed left logging muted
 * indefinitely). Outside a serial-bus hush the gate equals the lease; inside
 * one, the hush owns it and its own expiry reconciles. */
static void console_quiet_reconcile(void)
{
#if PROTO_BUS
    if (serbus_hushed) return;
#endif
    const bool leased = console_quiet_leased();
    if (nodecon.quiet() == leased) return;
    nodecon.set_quiet(leased);
    if (!leased) Serial.println("console: quiet lease over - log output restored");
}

static void serbus_tick(void)
{
#if PROTO_BUS
    /* NOT INSIDE A FLASH ERASE. The cache is off and the other core is
     * stalled; a Modbus exchange started here would blow its own inter-
     * character timeout. The bytes stay in the CDC FIFO and are read on the
     * next pass, which costs latency and loses nothing. */
    if (ble_ota_flash_busy()) return;

    while (nodecon.available() > 0) {
        int c = nodecon.read();
        if (c < 0) break;
        const uint32_t now  = millis();
        const uint32_t idle = now - serbus_last_byte_ms;
        serbus_last_byte_ms = now;

        /* HUSH ON THE FIRST BYTE, claim or no claim - see SERBUS_HUSH_MS.
         * This grants nothing: the bus claim below still needs a whole
         * CRC-valid frame. */
        serbus_hush_until = now + SERBUS_HUSH_MS;
        if (!serbus_hushed) { serbus_hushed = true; nodecon.set_quiet(true); }

        /* THE TYPED-COMMAND SURFACE. One accumulator, every word.
         *
         * WHAT CHANGED AND WHY, since this replaces working code: the 8-byte
         * lowercase-only buffer that lived here could hold 'ble'/'bleup' and
         * nothing with an argument in it, and it sat inside `#if
         * PROTO_BLE_OTA`, so a bus-only build had no typed commands at all.
         * ConsoleLine (console.h) is that same accumulator - same
         * reset-on-any-other-byte rule - widened to take digits and spaces and
         * moved somewhere a host test can reach it. The 'ble' words behave
         * exactly as before.
         *
         * UN-HUSH ONLY FOR A LINE WE RECOGNISE. The first inbound byte
         * silenced the console (SERBUS_HUSH_MS), and a command whose answer
         * went into that gate would be indistinguishable from a command that
         * was ignored - fatal for `bus`, whose whole job is to be readable.
         * But un-hushing for ANY completed line would be worse: a Modbus
         * payload that happened to be all lowercase/digits/spaces and ended
         * in 0x0A would put log text back on the wire in the middle of a
         * firmware push, which is the exact thing console.h exists to prevent.
         * So recognition comes first and the gate opens only after it. */
        static ConsoleLine cmdline = {};
        const char *ln = console_line_push(&cmdline, c);
        if (ln && *ln) {
            const ConsoleVerbCmd vc = console_verb_parse(ln);
            if (vc.kind != CONSOLE_VERB_NONE) console_verb_run(vc);
            const ConsoleBusCmd bc = console_bus_parse(ln);
#if PROTO_BLE_OTA
            const bool is_ble = (strcmp(ln, "ble") == 0 ||
                                 strcmp(ln, "bleup") == 0);
#else
            const bool is_ble = false;
#endif
            if (bc.kind != CONSOLE_BUS_NONE || is_ble) {
                nodecon.set_quiet(false);
                serbus_hushed = false;
            }
            if (bc.kind != CONSOLE_BUS_NONE) {
                bus_cmd_run(bc);
            }
#if PROTO_BLE_OTA
            else if (is_ble) {
                Serial.println("console: bringing up BLE window (15 min)");
                ble_ota_up(PROTO_BLE_BENCH_NONCE, BLE_OTA_MAX_UP_MS);
                hold_awake(BLE_OTA_MAX_UP_MS);
            }
#endif
            /* A quiet lease outlives the un-hush above: the command's own
             * reply has printed, and the log goes dark again after it. */
            if ((bc.kind != CONSOLE_BUS_NONE || is_ble) && console_quiet_leased())
                nodecon.set_quiet(true);
        }

        size_t n = serbus_push(&serbus_rx, (uint8_t)c, idle);
        if (n == 0) continue;

        /* THE FRAME IS THE CLAIM. Refused only when a session that is not
         * ours already holds the pair - a BLE or LoRaWAN push in flight - in
         * which case the frame is DROPPED rather than served into a race, and
         * the host sees the silence its own timeout is already written
         * around. */
        if (!ctrl_serial_claim(&ctrl, SERBUS_HOLD_MS, rtc_uptime_s())) {
            ctrl_window_persist();
            serbus_refused++;
            serbus_reset(&serbus_rx);
            continue;
        }
        serbus_frames++;

        uint8_t rsp[MAX_FRAME];
        /* SERIAL, not NODE: this is the USB operator's frame, and it is the
         * presence that keeps THEIR arm alive. */
        size_t  rlen = bus_exchange_from(serbus_rx.buf, n, rsp, sizeof(rsp),
                                         CTRL_ARM_OWNER_SERIAL);
        serbus_reset(&serbus_rx);
        /* VERBATIM, and past the gate: this is the answer the host is blocked
         * reading, not log output. Silence stays silence - o2p_update.py's
         * NoReply is a meaningful outcome and must not be dressed up as
         * anything else. */
        if (rlen) {
            nodecon.write_raw(rsp, rlen);
            nodecon.flush_raw();
        }

        /* Re-arm from the END of the exchange, not its start: a 253-byte FC16
         * at the cable's rate is a large fraction of a second and the hold is
         * meant to bound the gap to the NEXT frame. */
        (void)ctrl_serial_claim(&ctrl, SERBUS_HOLD_MS, rtc_uptime_s());
        ctrl_window_persist();
        serbus_last_byte_ms = millis();
        serbus_hush_until   = millis() + SERBUS_HUSH_MS;
        return;                       /* one exchange per pass */
    }

    /* The claim lapses in ctrl_expire() on the ordinary path; this is only
     * the console coming back, and it deliberately waits for BOTH the hush
     * window and the claim, so a lapsed hush cannot un-silence the console in
     * the middle of a push. */
    if (serbus_hushed && !ctrl.serial_claim &&
        (int32_t)(serbus_hush_until - millis()) <= 0) {
        serbus_hushed = false;
        nodecon.set_quiet(console_quiet_leased());
        Serial.printf("console: serial bus claim over - %u frame%s forwarded, "
                      "%u refused; RS-485 back with the poller\n",
                      (unsigned)serbus_frames,
                      serbus_frames == 1u ? "" : "s",
                      (unsigned)serbus_refused);
        serbus_frames  = 0;
        serbus_refused = 0;
    }
#else
    /* A BUS-OFF IMAGE STILL ANSWERS `caps` AND `quiet` (review 2026-09-10:
     * esp32s3_bench could not describe itself over USB). There is no Modbus
     * relay and no hush here, so every line goes straight to the accumulator. */
    {
        static ConsoleLine cmdline = {};
#if PROTO_BLE_OTA
        /* SERIAL OTA, DATA PHASE. Same receiver as the BLE path, different
         * transport: control lines arrive as text below, then `size` raw bytes
         * arrive here with no framing at all - exactly as the BLE DATA
         * characteristic delivers them, and for the same reason (framing the
         * payload would cost more than the transport saves).
         *
         * THE BYTE COUNT IS THE FRAME. There is no escape sequence and none is
         * wanted: an escape would have to be paid on every byte of a 700 kB
         * image to solve a problem the length already solves. `begin` said how
         * many bytes follow, so exactly that many are consumed here and the
         * accumulator never sees them - which is what keeps a firmware byte
         * that happens to be 0x0A from completing a command line.
         *
         * STAGE IN BLOCKS, AND LET stage() DRAIN. ble_ota_serial_stage() ticks
         * the consumer on every call because otaBleStageBytes() neither blocks
         * nor drops on a full ring - it latches a failure that surfaces as
         * OTAB FAIL credit-overrun. Blocking on flash here is the flow control:
         * nothing is read while it blocks, the CDC FIFO backs up, and the
         * host's write() blocks with it.
         *
         * NO EARLY RETURN. Falling through costs one empty available() test
         * and keeps console_quiet_reconcile() on its schedule; it also hands
         * any bytes past the last image byte straight to the accumulator,
         * which is where the following `end` belongs. */
        static uint32_t ota_rx_left = 0;
        /* A rejected begin or an expired transfer must release binary mode.
         * Discard queued payload after an abort; it is never console text. */
        if (ota_rx_left && !ble_ota_serial_active()) {
            while (nodecon.available() > 0) nodecon.read();
            ota_rx_left = 0;
            cmdline = {};
        }
        if (ota_rx_left) {
            uint8_t buf[512];
            size_t  k = 0;
            while (ota_rx_left && nodecon.available() > 0) {
                int c = nodecon.read();
                if (c < 0) break;
                buf[k++] = (uint8_t)c;
                ota_rx_left--;
                if (k == sizeof(buf)) { ble_ota_serial_stage(buf, k); k = 0; }
            }
            if (k) ble_ota_serial_stage(buf, k);
            if (!ota_rx_left)
                Serial.println("ota-serial: image received, waiting for `end`");
            else {
                console_quiet_reconcile();
                return;  /* newly arrived bytes still belong to the payload */
            }
        }
#endif
        while (nodecon.available() > 0) {
            int c = nodecon.read();
            if (c < 0) break;
            const char *ln = console_line_push(&cmdline, c);
            if (ln && *ln) {
#if PROTO_BLE_OTA
                /* THE RECEIVER'S OWN VERBS, over USB. caps.h lists these as
                 * CMD_BLE and says the begin/data sequence "must stay on the
                 * NimBLE task" - that overstates ota_ble.h, which says
                 * otaBleSubmitCommand() is "Safe from any task" and that the
                 * load-bearing split is producer/consumer, not BLE/not-BLE.
                 * The split exists so flash writes cannot stall the radio
                 * task; there is no radio task on this path.
                 *
                 * PUMP IMMEDIATELY: submit only LATCHES. A `begin` erases the
                 * passive slot inside that pump and takes seconds, so the
                 * host must not start streaming until this returns. */
                if (!strncmp(ln, "begin ", 6) || !strcmp(ln, "end") ||
                    !strcmp(ln, "abort")      || !strcmp(ln, "info")) {
                    const bool ok = ble_ota_serial_submit(ln);
                    ble_ota_serial_pump();
                    if (ok && !strncmp(ln, "begin ", 6) && ble_ota_serial_active()) {
                        ota_rx_left = (uint32_t)strtoul(ln + 6, nullptr, 10);
                        /* Leave the text loop before even one payload byte
                         * can be mistaken for a command in this same tick. */
                        break;
                    }
                    continue;
                }
#endif
                const ConsoleVerbCmd vc = console_verb_parse(ln);
                if (vc.kind != CONSOLE_VERB_NONE) console_verb_run(vc);
            }
        }
    }
#endif
    console_quiet_reconcile();
}

/* Wait `ms` while keeping the USB console serviced. setup()'s radio retry loops
 * used a bare delay(), and the BLE bench loop never returns to loop(), so in
 * both `caps` and `quiet` went unanswered and an expired quiet lease left the
 * log muted (review 2026-09-10, round two). */
static void console_serviced_delay(uint32_t ms)
{
    const uint32_t t0 = millis();
    do {
        serbus_tick();
        delay(10);
    } while (millis() - t0 < ms);
}

/* Modbus RTU CRC-16. The ONE place this firmware computes a CRC, and it is
 * not for publishing data - poll replies still go up untouched. It is for the
 * guard below, which decides whether a freshly installed image is allowed to
 * keep running on the node that owns every probe's update path. A guard that
 * accepts a frame it has not authenticated is not a guard. */
static uint16_t modbus_crc16(const uint8_t *b, size_t n) {
    uint16_t c = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        c ^= b[i];
        for (int k = 0; k < 8; k++)
            c = (c & 1) ? (uint16_t)((c >> 1) ^ 0xA001) : (uint16_t)(c >> 1);
    }
    return c;
}

static bool probe_answers(void) {
    /* The direction pin never came up, so nothing this function sends can
     * reach the pair. Report that as "no reply" - which it is - rather than
     * spending three attempts and REPLY_WAIT_MS each discovering it, and
     * never as a pass: an uninitialised transceiver must not be able to
     * vouch for an image. */
    if (!bus_direction_ok) return false;
    bus_wait_clear();
    bus_write(PROBE_PING, sizeof(PROBE_PING));
    uint8_t r[MAX_FRAME];
    size_t n = read_frame(r, sizeof(r));

    /* SAY WHICH FAILURE THIS WAS, AND SHOW THE BYTES.
     *
     * Every rejection below used to be a bare `return false`, and the caller
     * printed "no valid reply" and then "bus at boot: SILENT - slave 3 did not
     * answer" over all of them. Those are not the same fact and they send
     * different people to the pile: NOTHING CAME BACK is a cable, a rate, a
     * parity or a dead probe, while SOMETHING CAME BACK THAT WAS NOT THIS
     * REPLY is a wrong slave answering, a rate that is close enough to frame
     * bytes but not to frame them correctly, a collision with the Dragino, or
     * our own driver keyed into the reply. A console reading "SILENT" for the
     * second kind is an unevaluated distinction reported as an evaluated one,
     * and it is the reason this fault has needed a separate fixture build to
     * make any progress on - env:bussniff_poll's whole advantage over the
     * production image was that it PRINTS THE BYTES.
     *
     * The flag itself is unchanged and still means "slave 3 did not answer",
     * which is the question the OTA gate asks and the answer it needs; this
     * adds the diagnosis next to it rather than changing what is measured. */
    if (n == 0) {
        Serial.println("probe ping: silence - not one byte inside the reply "
                       "window");
        return false;
    }
    if (n != PROBE_REPLY_LEN || r[0] != PROBE_PING_SLAVE || r[1] != 0x03 ||
        r[2] != PROBE_REPLY_BYTECOUNT) {
        Serial.printf("probe ping: %u bytes came back, but not slave 3's "
                      "21-byte FC03 reply (addr %02X, fc %02X, count %02X):",
                      (unsigned)n, r[0], (unsigned)(n > 1u ? r[1] : 0u),
                      (unsigned)(n > 2u ? r[2] : 0u));
        for (size_t i = 0; i < n && i < 24u; i++) Serial.printf(" %02X", r[i]);
        if (n > 24u) Serial.printf(" ...");
        Serial.println();
        return false;
    }
    /* CRC BEFORE CONTENT. Length, address, function code and byte count are
     * four bytes that noise on an RS-485 pair can satisfy by accident. This
     * reply is the sole evidence that decides su_confirm() versus
     * su_reject_and_reboot(), so it has to be authenticated rather than merely
     * recognised - otherwise a corrupt bus can vouch for an image that cannot
     * actually reach the bus.
     *
     * NOTHING IS CHECKED IN THE REGISTERS THEMSELVES, and that is deliberate:
     * the question is whether the exchange happened, not whether the oxygen
     * reading is plausible. A probe reporting a fault is still a probe this
     * node can reach and update, and it is the register block in the uplink -
     * not this gate - that says so. */
    uint16_t crc = modbus_crc16(r, n - 2);
    if ((crc & 0xFF) != r[n - 2] || (crc >> 8) != r[n - 1]) {
        /* DUMP HERE TOO. This is the MOST diagnostic rejection of the three
         * and it was the one still printing a bare verdict: a frame with the
         * right length, address, function code and byte count that fails its
         * CRC is a reply from the right device that got corrupted in transit -
         * a marginal rate, a parity mismatch that still frames, a collision
         * with another master, or our own driver released late into it. The
         * bytes distinguish those and the verdict alone distinguishes none of
         * them. Found by review 2026-09-10. */
        Serial.printf("probe ping: %u bytes with the right header but a BAD "
                      "CRC (want %02X %02X, got %02X %02X):", (unsigned)n,
                      (unsigned)(crc & 0xFF), (unsigned)(crc >> 8),
                      r[n - 2], r[n - 1]);
        for (size_t i = 0; i < n && i < 24u; i++) Serial.printf(" %02X", r[i]);
        if (n > 24u) Serial.printf(" ...");
        Serial.printf("\n");
        return false;
    }
    return true;
}

/* Resolved once, in setup(), before anything arms a wake source. */
static bool warm_boot = false;
static bool boot_ble_requested = false;

void setup() {
    /* FIRST, before any delay or peripheral: the wake cause is only meaningful
     * until something else arms a wake source, and everything below depends on
     * knowing whether the RTC state is usable. */
    warm_boot = rtc_warm_boot();
    /* IMMEDIATELY AFTER, and before any reading of rtc_uptime_s(): this
     * validates the RTC_NOINIT spend budgets and establishes the clock origin
     * every later timestamp is relative to. A budget that fails its checks is
     * seeded SPENT, so a reset can never refund one. */
    rtc_budget_boot();
    /* rtc_budget_boot() has already printed which of the two cases this was -
     * a clean power-on, or a corrupt budget after a live reset - and seeded
     * accordingly. This only flags that the stored values were not used. */
    if (rtc_budget_was_reseeded())
        Serial.println("   (budget was re-seeded, not restored)");
    /* Before anything can open a session: a warm boot inherits the run it was
     * already inside, a cold boot correctly gets zeros and starts fresh. */
    ctrl_window_restore();

    /* RELEASE THE PIN HOLDS FIRST. park_for_sleep() latches the RF-switch pins
     * and the bridge TX line so they keep their level through deep sleep, and
     * a held pad IGNORES every later pinMode()/digitalWrite() until the hold is
     * cleared. Leaving them latched would silently disable the radio's RF
     * switch for the whole of the next cycle: SPI would work, the stack would
     * report a clean transmit, and nothing would reach the air. */
    /* UNCONDITIONALLY, NOT ONLY ON A WARM BOOT. warm_boot requires a
     * TIMER/EXT0 wake AND an intact RTC struct, and rtc_warm_boot() has an
     * explicit branch for "timer wake, struct not intact" that returns COLD -
     * reachable after any OTA that changes sizeof(RtcState), or RTC-RAM
     * corruption. On that boot the pads are still latched from
     * park_for_sleep() and were never released: GPIO1 is an RTC pad, so the
     * hold survives, and a latched pad IGNORES every later pinMode() and
     * digitalWrite(). For the RF-switch pins that is the failure the paragraph
     * above describes - SPI clean, stack happy, nothing on the air, and no
     * guard anywhere catches it. gpio_hold_dis() on an unheld pad is a no-op
     * register write, so the condition bought nothing and cost that. */
    gpio_hold_dis((gpio_num_t)PIN_LORA_RXEN);
    gpio_hold_dis((gpio_num_t)PIN_LORA_TXEN);
#if PROTO_BUS
    gpio_hold_dis((gpio_num_t)PIN_BUS_DE);
#endif
    gpio_deep_sleep_hold_dis();

    Serial.begin(115200);
    delay(warm_boot ? 20 : 300);

    /* A `quiet on` LEASE SURVIVES A RESTART - a DTR/RTS-low port open, a panic,
     * a deep-sleep wake - which is why it lives in RTC_NOINIT memory (config.h,
     * CONSOLE_QUIET_LEASE_S). Said once, through the gate's bypass, so a quiet
     * boot is never mistaken for a dead one; then the log goes dark. The ROM's
     * own banner has already printed by now and cannot be silenced from here. */
    if (console_quiet_leased()) {
        char msg[160];
        snprintf(msg, sizeof(msg),
                 "console: quiet lease, %lu s left - boot log suppressed; "
                 "`quiet off` restores it, `caps` still answers",
                 (unsigned long)(rtc_console_quiet_until() - rtc_uptime_s()));
        nodecon.say_anyway(msg);
        nodecon.set_quiet(true);
    }

    /* The button pad comes out of deep sleep still owned by the RTC IO block
     * (rtcstate.cpp armed it there for ext0). Hand it back to the digital
     * GPIO matrix before reading it, or the read can see the RTC mux rather
     * than the pin. A no-op on a cold boot. */
    rtc_gpio_deinit((gpio_num_t)PIN_BOOT_BUTTON);
    pinMode(PIN_BOOT_BUTTON, INPUT_PULLDOWN);
    if (rtc_button_wake() ||
        digitalRead(PIN_BOOT_BUTTON) == BOOT_BUTTON_ACTIVE) {
        boot_ble_requested = true;
    }
    if (!warm_boot) {
        /* Repeat: with USB CDC the host often attaches after setup() started,
         * and a one-shot banner is simply lost.
         *
         * COLD BOOT ONLY. 2.4 s of banner on every wake is 2.4 s of radio-idle
         * current 288 times a day, spent on a console that deep sleep has
         * already disconnected - the USB CDC drops on sleep and the port
         * re-enumerates on the far side, so there is rarely anyone listening
         * to a warm boot's banner anyway. */
        for (int i = 0; i < 6; i++) {
            Serial.printf("=== farm node prototype boot (%d) ===\n", i);
            delay(400);
        }
    } else {
        Serial.printf("=== wake %lu, uptime %lu s ===\n",
                      (unsigned long)rtc_wake_count(),
                      (unsigned long)rtc_uptime_s());
    }
    /* WHAT A REBOOT DID TO AN OPEN TRANSFER, said at boot rather than inferred
     * from the first BEGIN's answer over the air.
     *
     * su_resume_pending() existed and was never called, which meant the one
     * fact a resume test needs - did the cursor survive this reset, and at
     * what offset - was only observable by sending a BEGIN and reading what
     * came back. That conflates "the node kept its cursor" with "the node
     * answered our downlink", and when a resume goes wrong those are the two
     * explanations you most need to tell apart.
     *
     * Printed unconditionally, including the no-cursor case: "nothing staged"
     * is a result, not an absence, and a line that only appears on success
     * cannot distinguish a fresh node from a node whose store failed to load. */
    {
        uint32_t su_len = 0, su_got = 0;
        if (su_resume_pending(&su_len, &su_got))
            Serial.printf("self-update: cursor SURVIVED this boot - %lu of %lu bytes staged\n",
                          (unsigned long)su_got, (unsigned long)su_len);
        else
            Serial.println("self-update: no staged transfer (nothing to resume)");
    }

#if PROTO_BUS
    Serial.println("step 1: Serial1.begin");
    /* 8E1 AT THE PROBE'S OWN RATE. This UART is the RS-485 bus now, not a
     * private link to a bridge that re-transmitted onto the cable at a
     * different rate, so these settings are the probe's own compiled defaults
     * (o2-probe regmap.h:664 - 9600 8E1, matching the Truebner SMT100s and the
     * Dragino's AT+BAUDR=9600 / AT+PARITY=2). A baud or parity mismatch
     * presents as silence, not as an error.
     *
     * AMENDED 2026-09-09: the compiled pair above is now the FALLBACK, not the
     * setting. bus_store_begin() reads a pair that a CRC-valid reply has
     * already proved on this cable and, if one exists, that is what this UART
     * comes up at - see busstore.h for why persisting an unproven rate and
     * persisting nothing at all are both wrong. The paragraph above still
     * describes what a virgin node does, which is every node until someone
     * runs `bus baud`. It also still describes the failure mode: a mismatch is
     * silence, whichever of the two pairs is in force. */
    bus_store_begin(O2P_UART_BAUD, CONSOLE_BUS_PARITY_EVEN);
    Serial1.begin(bus_stored_baud(), bus_parity_config(bus_stored_parity()),
                  PIN_BUS_RX, PIN_BUS_TX);
    /* Direction is driven here, not by the UART's RTS. The long form of why,
     * and what was measured, is in the comment above bus_write(). */
    bus_direction_ok = bus_direction_begin(bus_stored_baud());
    /* NO pinMode() ON THE RX PAD HERE. IT MAKES THE UART DEAF.
     *
     * On arduino-esp32 3.x, pinMode() on a peripheral-owned pin calls
     * perimanClearPinBus() -> _uartDetachBus_RX(), which points U1RXD at
     * GPIO_FUNC_IN_HIGH and sets uart->_rxPin = -1. From that instant
     * Serial1.available() is 0 forever, whatever the pad, the click, the cable
     * or the probe do - and digitalRead() still reads the pad correctly, so
     * every GPIO-level diagnostic keeps looking healthy. That combination cost
     * the 2026-09-09 bring-up most of a night: a scan of all 247 Modbus
     * addresses, four baud rates, both parities and six pin orderings, every
     * one of them run on a UART that had been disconnected from its pin.
     *
     * The line it replaced was added to hold the pad at mark while the click's
     * receiver is Hi-Z during transmit. That concern is real but the fix was
     * unnecessary: _rxPullEnabled defaults true, so begin()/setPins() already
     * apply the pull-up via _uartApplyRxPull(). If it ever needs changing, use
     * Serial1.enableRxInternalPull(bool) BEFORE begin(). */
#else
    /* NOT STARTED, and that is the whole point of PROTO_BUS_OFF.
     *
     * begin() drives PIN_BUS_TX as a UART TX, and a UART TX IDLES HIGH. With
     * the bridge board that alone was the fault: it put a push-pull driver on
     * the bridge's USART2 pads, which is where the bench CP2102 lead also sits,
     * so the contention was created in setup() before anything decided to send.
     * README.md said so about the lead in the very next paragraph ("an idle TX
     * still idles high") and this file then did it anyway; found by a Codex
     * review, 2026-09-07.
     *
     * With a transceiver the idle level no longer reaches the cable on its own
     * - PIN_BUS_DE decides that, and an unconfigured pad sits at the click's
     * pull-down, receiving. Leaving the port un-begun is still what this build
     * wants, and now for two reasons: both pads stay high-impedance, and
     * bus_direction_ok stays false, so nothing in this image can key the
     * driver even if something later decides to write. */
    Serial.println("step 1: SKIPPED - PROTO_BUS_OFF, Serial1 is never started, "
                   "so GP2/GP3 stay high-impedance and R/T stays pulled down");
#endif

    /* THE STORED CADENCE, BEFORE ANYTHING CAN ASK FOR THE INTERVAL.
     *
     * poll_interval_ms() resolves through cad_default_ms(), and that reports
     * the compiled default until this has run. Loading it late would not fail
     * loudly - it would simply schedule the first measurement, and any sleep
     * taken before the load, on the WRONG interval, and the node would then
     * settle onto the right one as if nothing had happened. A cadence that is
     * briefly wrong at every boot is exactly the kind of fault nobody reports
     * and nobody can reproduce, so the load goes first. */
    cad_begin(POLL_INTERVAL_MS);

    /* BEFORE the radio, because radio.begin() and setTCXO() below retry
     * FOREVER. An image that cannot bring its radio up would otherwise never
     * reach su_pending_verify(), never start the 30-minute clock, and sit
     * awake indefinitely on a node whose previous image worked - the precise
     * situation the deadline exists for, and the one it used to miss because
     * it was installed after the loops rather than before them. */
    /* BEFORE the radio, because radio.begin() and setTCXO() below retry
     * FOREVER on failure. An image that cannot bring its radio up would
     * otherwise never reach su_pending_verify(), never start its 30-minute
     * clock, and sit awake indefinitely on a node whose PREVIOUS image worked
     * - which is precisely the situation the deadline exists for, and the one
     * it used to miss because it was installed after those loops.
     *
     * WALL CLOCK, not millis(): under deep sleep millis() restarts on every
     * wake, so a deadline expressed in it can never be reached. */
    {
        su_verify_t vst = su_verify_state();
        pending_verify  = (vst == SU_VERIFY_PENDING);
        verify_unknown  = (vst == SU_VERIFY_UNKNOWN);
        if (verify_unknown)
            Serial.println("su: verify state UNKNOWN - this image will NOT be "
                           "rolled back on that basis, and will not block "
                           "sleep. If it really is provisional the bootloader "
                           "reverts it at the next reset, which is the answer "
                           "we are entitled to make.");
    }
    Serial.printf("running from %s%s\n", su_running_partition(),
                  pending_verify ? " (PENDING VERIFY - must prove itself)" : "");
#if PROTO_BENCH_CONFIRM_OTA
    /* CONFIRM HERE, not at the end of setup(). The other bench-confirm branch
     * below sits after the radio bring-up and the join, which on a bench board
     * with no SX1262 is never reached - so an OTA'd bench image stayed
     * PENDING_VERIFY and the NEXT push was refused with
     * ESP_ERR_OTA_ROLLBACK_INVALID_STATE. Measured 2026-09-10: two consecutive
     * BLE pushes, the second one dead on arrival, recoverable only with a USB
     * flash. Confirming before anything that can block is the whole point. */
    if (pending_verify) {
        if (su_confirm()) {
            pending_verify = false;
            Serial.println("confirmed early (PROTO_BENCH_CONFIRM_OTA) - bench "
                           "build, nothing here can prove a bus or a radio");
        } else {
            Serial.println("early bench confirm did NOT take - image stays "
                           "provisional and will roll back at the next reset");
        }
    }
#endif

    /* Generous: a join can legitimately take many minutes on a bad day, and
     * rolling back a good image because coverage was poor for ten minutes
     * would be its own outage.
     *
     * WALL CLOCK, not millis(). Under deep sleep millis() restarts every wake,
     * so a deadline expressed in it can never be reached - the 30-minute
     * rollback guard would silently never fire, and an image that runs but
     * cannot confirm itself would stay installed forever. rtc_uptime_s()
     * counts the sleep too. */
    verify_deadline = rtc_uptime_s() + 30UL * 60UL;

    /* The BLE bus channel's wire. Registered before any window can open -
     * PROTO_BLE_BENCH brings one up inside setup() a few lines below - so
     * there is no interval in which a peer could write a request the module
     * would answer "this build does not drive the RS-485 pads". */
    ble_bus_set_exchange(PROTO_BUS ? &ble_bus_exchange : nullptr,
                         &ble_bus_claimed);
    /* Same reasoning, same moment: a PROTO_BLE_BENCH window opens inside this
     * function, so the predicate has to be in place before a peer can arm the
     * stream and the teardown ask whether anything is running. */
    ble_telem_set_armed_hook(&telem_armed_now);

#if PROTO_BLE_OTA
    /* Bring up BLE after all peripherals, Serial1, bus exchange and hooks are fully initialized.
     * Brings up BLE on BOOT button wake/press, or automatically when cabled to a live USB host. */
    bool usb_bench = false;
#if ARDUINO_USB_CDC_ON_BOOT
    usb_bench = HWCDC::isPlugged();
#endif
    if (boot_ble_requested || usb_bench) {
        Serial.printf("bringing up BLE window (15 min) [%s]\n",
                      boot_ble_requested ? "BOOT button" : "USB host");
        ble_ota_up(PROTO_BLE_BENCH_NONCE, BLE_OTA_MAX_UP_MS);
        hold_awake(BLE_OTA_MAX_UP_MS);
    }
#endif

    Serial.println("step 2: spi.begin");
    delay(100);
    spi.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS);
#ifdef PROTO_BLE_BENCH
#ifndef PROTO_BLE_BENCH_CORE
#define PROTO_BLE_BENCH_CORE 0
#endif
    /* BLE-ONLY BENCH BUILD: no radio, no join, no bus, no sleep.
     *
     * The BLE OTA receiver is normally reachable only through a fPort-13 CTRL
     * session, which needs an SX1262, a gateway and ChirpStack. That makes the
     * receiver itself untestable on a bare ESP32-S3 - and worse, it makes every
     * BLE experiment compete with the one node that is carrying live data.
     * Without a radio the loop above never exits: radio.begin() retries
     * forever, so setup() never reaches ble_ota_up() at all.
     *
     * The nonce is a COMPILE-TIME CONSTANT here, because there is no uplink to
     * carry a fresh one. That removes the whole authentication story - anyone
     * in radio range who reads this file can push firmware to this board. It is
     * why the banner says so, and why this must never be built for a node that
     * leaves the bench.
     */
    Serial.println("*** PROTO_BLE_BENCH: LoRaWAN, the bus and sleep are all "
                   "SKIPPED. The BLE window opens at boot and re-opens when it "
                   "expires, and the auth nonce is a fixed compile-time "
                   "constant - there is NO authentication worth the name. "
                   "Bench only, never on a deployed node.");
    ble_ota_up(PROTO_BLE_BENCH_NONCE, BLE_OTA_MAX_UP_MS);
    /* Historical core-0 experiment. The handoff's nine-run comparison found
     * no speedup (26.97 vs 28.08 kB/s). The causal claims below were refuted;
     * this pinning remains bench-only pending a controlled replacement.
     * ON CORE 0, WITH THE RADIO, BECAUSE THE ARDUINO TASK IS ON CORE 1.
     *
     * MEASURED 2026-09-10: the node pushes at 25.8 kB/s (25.64/25.68/26.06,
     * three real-write runs) where fugu does 41.9 kB/s (41.3-43.7, n=5) on the
     * same board, the same receiver, the same 256 KB ring and the same 30 ms
     * interval. Subtracting flash from both says where it goes: of ~28 s the
     * node spends 13.5 s in write_ms (9.5 s of it erase), leaving 14.5 s that
     * is neither erase nor write; fugu carries 2.5x more payload in the SAME
     * 13.8 s of non-flash time. Per byte the node's link runs at 49.5 kB/s
     * against fugu's 129.6. The ceiling is not flash - if the link fed at
     * fugu's rate this image would floor at write_ms, i.e. ~53 kB/s.
     *
     * THE STRUCTURAL DIFFERENCE IS THE CORE, and it is not overridable by
     * config. CONFIG_ARDUINO_RUNNING_CORE is 1 here and 0 on fugu, but ours
     * comes from the PREBUILT framework
     * (~/.platformio/packages/framework-arduinoespressif32-libs/esp32s3/
     * sdkconfig:613), not from platformio.ini, so it cannot be changed without
     * rebuilding the framework. Both firmwares put NimBLE and the BT
     * controller on core 0 (BT_NIMBLE_PINNED_TO_CORE=0,
     * BT_CTRL_PINNED_TO_CORE=0). So on fugu the controller, the host and the
     * OTA consumer are all core 0 and a staged byte never crosses a core
     * boundary between arrival and flash; here it crossed one every chunk.
     * ota_ble.h prescribes exactly this remedy: "either give this its own
     * task".
     *
     * ONE TASK, NOT A SECOND TICKER. The whole loop body moves, rather than
     * ble_ota_tick() alone, because otaBleTick() must have a single consumer
     * and serbus_tick() reaches it too through ble_ota_serial_stage(). Two
     * tasks calling it would be two consumers of one ring.
     *
     * Priority 1 and a captureless lambda (which converts to TaskFunction_t),
     * so this matches what the Arduino loop task gave it - only the core
     * changes. 16 KB because telemetry_emit() builds its payload on this
     * stack and the Arduino loop task it used to run on has 8 KB by default;
     * this is the one place a stack overflow would look like a throughput
     * result. The Arduino task then parks forever below: setup() must never
     * return in this build, exactly as the endless loop it replaces never
     * did, or loop() would start driving LoRaWAN on a board with no radio. */
    xTaskCreatePinnedToCore([](void *) {
    for (;;) {
        ble_ota_tick();

        /* During a transfer, service the receiver AND its USB producer.
         * The original fast path below skipped serbus_tick(), which starved
         * serial OTA immediately after READY. Its throughput explanation is
         * historical: the handoff found lower variance, no mean speedup.
         *
         * ota_ble.h says it plainly: "HOW OFTEN YOU CALL THIS IS THE
         * TRANSFER'S THROUGHPUT. This is not a housekeeping tick... any delay
         * between ticks is dead air on the link... either give this its own
         * task or make sure everything else sharing the task is non-blocking."
         *
         * MEASURED 2026-09-10, and this is the last unexplained gap: the node
         * pushes at ~28 kB/s where fugu does 40.7 kB/s on the same board, the
         * same receiver, the same 256 KB ring and the same 30 ms connection
         * interval (macOS grants 24 to both; asking for 12..24 changed
         * nothing). Chunk size makes no difference (28.7 at 244 B vs 27.9 at
         * 512 B) and the host's write pacing cannot be removed - without it
         * the transfer stalls outright. What is left is this loop: fugu pumps
         * its consumer every network-loop pass behind a 1 ms yield, while this
         * one paid telemetry_control() + telemetry_emit() + serbus_tick() +
         * delay(2) between every pair of ticks.
         *
         * Nothing here is dropped, only deferred: a transfer is bounded, and
         * telemetry/console work resumes the moment it ends. delay(1) rather
         * than a bare yield keeps the idle task fed - starving it trips the
         * task watchdog, which would turn a throughput fix into a reboot. */
        if (ble_ota_busy()) {
            serbus_tick();  /* serial OTA must keep feeding the receiver */
            delay(1);
            continue;
        }
        /* THE BENCH LOOP HAS TO DRIVE TELEMETRY ITSELF, because it never
         * returns to loop(). Review 2026-09-09: without these two calls
         * esp32s3_blebench COMPILED the fast path and could not run a byte of
         * it - the `telem` verb set its latch and nothing ever consumed it. The
         * one environment able to exercise BLE without a radio, a gateway or a
         * field node was therefore the one environment that could not close the
         * feature's verification gap, which is the opposite of what it is for.
         *
         * Same order as loop(): control before emit, so an arming that lapsed
         * this iteration is gone before anything asks whether to send. */
        telemetry_control();
        telemetry_emit();
        serbus_tick();          /* caps/quiet, and the lease's own expiry */
        if (!ble_ota_is_up()) {
            Serial.println("ble bench: window expired, re-opening");
            ble_ota_up(PROTO_BLE_BENCH_NONCE, BLE_OTA_MAX_UP_MS);
        }
        delay(2);
    }
    }, "blebench", 16384, nullptr, 1, nullptr, PROTO_BLE_BENCH_CORE);
    /* The Arduino task's only remaining job is to not return. */
    for (;;) delay(1000);
#endif

    Serial.println("step 3: radio.begin");
    delay(100);
    /* Print in a loop, not once. With USB CDC the host may attach after
     * setup() has already run, and a one-shot error message is simply gone -
     * which looks identical to a board that never booted. */
    int st = radio.begin();
    while (st != RADIOLIB_ERR_NONE) {
        rollback_if_overdue();
        Serial.printf("radio.begin failed: %d (-2=chip not found: check 3V3, "
                      "GND, MISO/MOSI/CLK/CS, BUSY, RESET)\n", st);
        console_serviced_delay(2000);
        st = radio.begin();
    }
    Serial.println("radio.begin OK - SX1262 answered over SPI");
    /* Both of these fail SILENTLY if omitted: SPI keeps working and the radio
     * simply never gets on the air, which reads as poor coverage. */
    st = radio.setTCXO(LORA_TCXO_V);
    while (st != RADIOLIB_ERR_NONE) {
        rollback_if_overdue();
        Serial.printf("setTCXO(%.1f) failed: %d - try 1.8 V\n",
                      (double)LORA_TCXO_V, st);
        console_serviced_delay(2000);
        st = radio.setTCXO(LORA_TCXO_V);
    }
    Serial.println("setTCXO OK");
    radio.setRfSwitchPins(PIN_LORA_RXEN, PIN_LORA_TXEN);
    radio.setOutputPower(LORA_TX_DBM);
    /* THE BAND MUST BE FILLED IN BEFORE beginOTAA(), not after.
     *
     * This block used to sit BELOW the beginOTAA() call, under a comment
     * saying it had to happen before activation - which was half right and
     * therefore wrong. beginOTAA() itself calls clearSession(), and
     * clearSession() copies the RX2 frequency, RX2 data rate and TX power out
     * of the band object into the session (RadioLib LoRaWAN.cpp, clearSession).
     * At that moment `band56a` was still zero-initialised, so the session was
     * seeded with an RX2 frequency of 0 Hz and DR0.
     *
     * Nothing about that fails loudly: SPI works, the join works, and the node
     * simply listens in the wrong place - the same class of fault as the
     * 2026-09-06 measurement recorded above, where the server was certain the
     * device knew and the device had never been told.
     *
     * freq is in 100 Hz steps. */
    band56a = EU868;
    if (O2P_RX2_FREQ_HZ) {
        band56a.rx2.freq = O2P_RX2_FREQ_HZ / 100;
        band56a.rx2.dr   = O2P_RX2_DR;
        Serial.printf("RX2 override: %.3f MHz DR%u\n",
                      O2P_RX2_FREQ_HZ / 1e6, (unsigned)O2P_RX2_DR);
    } else {
        Serial.printf("RX2: band default %.3f MHz, network moves us\n",
                      band56a.rx2.freq / 1e4);
    }

    /* nwkKey MUST be NULL. RadioLib's beginOTAA does
     *     if (nwkKey) { this->rev = 1; }
     * i.e. a non-NULL nwkKey switches the node to LoRaWAN 1.1. The ChirpStack
     * device profile is LORAWAN_1_0_3, and the mismatch fails as a permanent
     * MIC error that looks like a wrong AppKey. */
    node.beginOTAA(LW_JOIN_EUI, LW_DEV_EUI, NULL, LW_APP_KEY);

    /* ENFORCE THE BAND LIMIT FROM HERE, not after the join.
     *
     * It used to be enabled only once a session existed, which left the join
     * itself outside it - and a node that cannot join transmits every 20 s
     * for as long as the grace allows. beginOTAA() has selected the band by
     * now, which is all setDutyCycle() needs.
     *
     * msPerHour = 0 takes the band's own figure: 36000 ms/h for EU868
     * (LoRaWANBands.cpp:28), exactly the 1 % Vfg. 91/2025 requires, and
     * deliberately not a hand-typed constant. */
    node.setDutyCycle(true, 0);   /* returns void: it only sets the flag */
    Serial.println("duty cycle: enforcing the band limit from the join onward");

    /* Restore BEFORE activating. Nonces stop the DevNonce-replay rejections; a
     * restored session skips the join entirely, which matters because
     * flush_queue_on_activate=True means a join throws away any downlink
     * waiting for us.
     *
     * WHERE FROM depends on how we got here, and the difference is the fCnt.
     *
     *   WARM (deep-sleep wake): RTC memory. Its fCnt is the one the last
     *   uplink actually used, so the session resumes exactly where it left
     *   off. This is the normal path, 288 times a day.
     *
     *   COLD (power loss, OTA reboot, watchdog): nonces from NVS, and NO
     *   session - we join. The NVS session's fCnt is at best the last hourly
     *   backstop and is therefore BEHIND what the server has already seen.
     *   ChirpStack rejects an uplink whose fCnt does not advance, so resuming
     *   from it produces a node that transmits happily and is silently
     *   discarded until it catches up - which reads as a coverage problem and
     *   is one of the harder faults to diagnose from the ground. A join costs
     *   a few seconds and one DevNonce, and only happens on a real power
     *   event. */
    uint8_t nb[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
    uint8_t sb[RADIOLIB_LORAWAN_SESSION_BUF_SIZE];
    /* RADIOLIB CHECKS THESE BUFFERS AND WE WERE THROWING THE ANSWER AWAY.
     *
     * rtc_warm_boot() validates only the RTC struct's own magic and size, so
     * the session and nonce BODIES can be corrupt while the header is
     * perfect - which is exactly what a partially-written struct or a
     * marginal RTC domain looks like. RadioLib carries its own signature
     * inside each buffer and returns an error when it rejects one
     * (LoRaWAN.cpp:317), but both int16_t returns were discarded and the node
     * printed "restored" either way.
     *
     * A node that believes it restored a session it did not transmits with
     * keys nobody recognises and is silently discarded - the coverage-shaped
     * fault the comment above warns about. Say so instead, and fall through
     * to a real join, which is the recoverable outcome. */
    if (warm_boot && rtc_load_nonces(nb, sizeof(nb)) &&
        rtc_load_session(sb, sizeof(sb))) {
        int16_t rn = node.setBufferNonces(nb);
        int16_t rs = node.setBufferSession(sb);
        if (rn == RADIOLIB_ERR_NONE && rs == RADIOLIB_ERR_NONE) {
            Serial.println("restored nonces + session from RTC memory");
        } else {
            Serial.printf("RTC header was intact but RadioLib REJECTED the "
                          "contents (nonces=%d session=%d) - joining fresh\n",
                          (int)rn, (int)rs);
        }
    } else if (lw_load_nonces(nb, sizeof(nb))) {
        int16_t rn = node.setBufferNonces(nb);
        if (rn == RADIOLIB_ERR_NONE)
            Serial.println("cold boot: restored nonces from NVS, joining fresh "
                           "(an NVS session's fCnt is behind the server's)");
        else
            Serial.printf("cold boot: NVS nonces REJECTED by RadioLib (%d) - "
                          "joining with a fresh DevNonce\n", (int)rn);
    }
    /* THE JOIN OWES THE BAND TOO.
     *
     * The duty gate lives in loop(), and activateOTAA() runs here in setup()
     * before loop() has ever executed - so the stored obligation was recorded
     * faithfully and then ignored by the very next thing the node did. That is
     * not a corner case: after a self-update COMMIT the firmware transmits an
     * acknowledgement and calls esp_restart() 200 ms later, so the reboot lands
     * squarely inside the silence the frame just bought, and the JoinRequest
     * goes out in the same 1 % sub-band.
     *
     * RadioLib cannot help: its lastToA and tUplinkEnd are reconstructed at
     * zero on every boot, which is why this accounting exists at all. Seeding
     * them is not an option either - they are millis()-based and millis() is
     * near zero here, so the arithmetic would wrap.
     *
     * So wait it out, in chunks, feeding the watchdog. This is placed after the
     * peripherals and BLE are up, so the node stays reachable while it waits -
     * a cold-boot reseed can owe several minutes and a node that is silent AND
     * deaf during them would be the worse failure. */
    {
        uint32_t owed_s = 0, jt_s = 0, jtoa_ms = 0;
        rtc_duty_load(&jt_s, &jtoa_ms);
        if (!duty_may_send(rtc_uptime_s(), jt_s, jtoa_ms, &owed_s) && owed_s) {
            Serial.printf("join held %lu s: the band owes silence after a "
                          "%lu ms transmission before this reboot\n",
                          (unsigned long)owed_s, (unsigned long)jtoa_ms);
            const uint32_t until_s = rtc_uptime_s() + owed_s;
            while ((int32_t)(until_s - rtc_uptime_s()) > 0) {
                serbus_tick();           /* stay reachable while we wait */
                ble_ota_tick();
                delay(10);
            }
            Serial.println("join: the silence is served");
        }
    }

    Serial.println("activating...");
    for (;;) {
        /* Timed so the grace below can be charged what the attempt ACTUALLY
         * cost. activateOTAA() blocks through the join TX and both RX windows
         * (~5-6 s), and the retry sleep adds 20 more, so a nominal 20 s credit
         * loses ground on every iteration. */
        uint32_t attempt_start_s = rtc_uptime_s();
        int jst = node.activateOTAA();
        if (jst == RADIOLIB_LORAWAN_SESSION_RESTORED) {
            Serial.println("session resumed - no join, no queue flush");
            break;
        }
        if (jst == RADIOLIB_LORAWAN_NEW_SESSION) {
            Serial.println("new session (joined)");
            /* CHARGE THE JOIN. A NEW_SESSION means a JoinRequest actually left
             * the radio, and until 2026-09-09 nothing recorded its airtime:
             * rtc_duty_store() was called only around the ordinary uplink path,
             * so the join's obligation existed on the air and nowhere in this
             * node's accounting. Two consequences, both real - the first uplink
             * after a join was gated as though the join had cost nothing, and a
             * reset between the join and that uplink lost the obligation
             * entirely, which is reset-as-refund by another route.
             *
             * Same fail-safe rule as the uplink path: getLastToA() is zeroed by
             * a TX_TIMEOUT and is not always answerable (see the notes at the
             * top of this file), and an unknown airtime must be charged, not
             * excused. A SESSION_RESTORED is deliberately NOT charged - it
             * resumes from NVS without transmitting at all. */
            const uint32_t join_toa = (uint32_t)node.getLastToA();
            rtc_duty_store(rtc_uptime_s(),
                           join_toa ? join_toa : DUTY_UNKNOWN_TOA_MS);
            Serial.printf("duty: charging the join %lu ms%s\n",
                          (unsigned long)(join_toa ? join_toa : DUTY_UNKNOWN_TOA_MS),
                          join_toa ? "" : " (airtime unknown - charged the "
                                          "worst case rather than nothing)");
            break;
        }
        /* Print the CODE. "join failed" alone cannot distinguish credentials
         * from wiring, and the two need opposite fixes:
         *   -5    TX_TIMEOUT     -> the radio never finished sending: DIO1
         *   -6    RX_TIMEOUT     -> sent, nothing heard back at all
         *   -1116 NO_JOIN_ACCEPT -> sent, no JoinAccept: gateway, coverage,
         *                           or the gateway not forwarding this JoinEUI
         *   -1101 NETWORK_NOT_JOINED -> asked to do something needing a session
         *
         * THE CODES USED TO BE WRONG HERE, and misdiagnosed a real outage.
         * -1101 was labelled NO_JOIN_ACCEPT; it is NETWORK_NOT_JOINED, and
         * NO_JOIN_ACCEPT is -1116. Worse, -1116 fell through to the catch-all
         * "check your keys", so on 2026-09-08 - farmgw off the air for an hour
         * - the node insisted the credentials were wrong while the keys were
         * perfect and the gateway was simply absent. That is exactly the
         * confusion the comment above exists to prevent. */
        Serial.printf("join failed: %d %s\n", jst,
            jst == RADIOLIB_ERR_TX_TIMEOUT     ? "(TX_TIMEOUT - DIO1 not connected?)" :
            jst == RADIOLIB_ERR_RX_TIMEOUT     ? "(RX_TIMEOUT - nothing heard: gateway down or out of range?)" :
            jst == RADIOLIB_ERR_NO_JOIN_ACCEPT ? "(NO_JOIN_ACCEPT - transmitted, but no JoinAccept came back: "
                                                 "gateway down, out of range, or not forwarding this JoinEUI)" :
            "(check DevEUI/JoinEUI/keys against ChirpStack)");

        /* WHOSE FAULT IS THIS? rollback_if_overdue() DESTROYS A WORKING IMAGE,
         * so it must not fire on evidence that points away from the image.
         *
         * "I transmitted and nobody answered" is a statement about the
         * gateway, not about this firmware. A gateway outage longer than 30
         * minutes is an ordinary event - measured 2026-09-08, farmgw was off
         * the air for over an hour - and reverting good firmware because of
         * one is the destructive half of scoring UNEVALUABLE input as FAIL.
         *
         * NOT simply "never roll back on these": an image that genuinely
         * cannot join must still eventually give up, or the safety net stops
         * existing. So an external-looking failure BUYS TIME rather than
         * cancelling the deadline, and the extension is bounded. A local
         * failure (-5, or anything suggesting our own radio or credentials)
         * leaves the original 30-minute deadline exactly as it was. */
        if (jst == RADIOLIB_ERR_RX_TIMEOUT || jst == RADIOLIB_ERR_NO_JOIN_ACCEPT) {
            /* CHARGE WHAT THE CYCLE REALLY COSTS, not a nominal 20 s.
             *
             * This used to credit a flat 20 s while one iteration burns the
             * blocking join (~5-6 s) PLUS the 20 s sleep below - so the
             * deadline lost ~5-6 s per attempt and the intended 6 h of grace
             * expired after roughly 2.2-2.5 h, most of join_grace_left_s
             * unspent. Measuring the whole cycle, including the sleep, makes
             * the credit exactly cover the time it is meant to buy. */
            uint32_t spent = rtc_uptime_s() - attempt_start_s + JOIN_RETRY_S;
            if (spent == 0) spent = 1;          /* never credit nothing */
            if (join_grace_left_s > 0) {
                uint32_t step = (join_grace_left_s < spent) ? join_grace_left_s
                                                            : spent;
                join_grace_left_s -= step;
                verify_deadline   += step;
            } else if (!join_grace_exhausted) {
                join_grace_exhausted = true;
                Serial.println("no gateway heard for the whole grace window "
                               "- from here an unverified image rolls back");
            }
        }
        /* Persist the advanced nonce even on failure, or the next attempt
         * replays a DevNonce the server has already rejected. */
        lw_save_nonces(node.getBufferNonces(),
                       RADIOLIB_LORAWAN_NONCES_BUF_SIZE);

        /* THE DEADLINE HAS TO BE CHECKED HERE, and until now it was checked
         * nowhere at all: verify_deadline was assigned in setup() and never
         * read, so the guard README.md describes - "a 30-minute deadline
         * covers the case where the image runs but never manages to confirm;
         * it rolls back rather than continuing unverified" - did not exist.
         *
         * This loop is exactly where it was needed. A freshly installed image
         * that cannot join never leaves it: it never reaches the bridge check,
         * never calls su_confirm(), and never reboots, so the bootloader's
         * rollback - which only acts at a boot - is never given the chance to
         * act. The node retries a join every 20 s forever, on an image that
         * may be the reason it cannot join.
         *
         * Roll back instead. The previous image was known to work. */
        rollback_if_overdue();
        const uint32_t wait_until = millis() + (JOIN_RETRY_S * 1000UL);
        while ((int32_t)(wait_until - millis()) > 0) {
            serbus_tick();
            ble_ota_tick();
            delay(10);
        }
    }
    /* ENFORCE THE EU868 DUTY CYCLE. It was never enforced at all.
     *
     * RadioLib defaults `dutyCycleEnabled` to FALSE (LoRaWAN.h:1051) and
     * nothing in this firmware ever called setDutyCycle(), so the node was
     * free to transmit as often as the cadence told it to. A comment in the
     * downlink path claimed "RadioLib enforces the dwell between
     * transmissions"; it does not, and that claim has been corrected.
     *
     * msPerHour = 0 takes the band's own figure, which for EU868 is
     * 36000 ms/h (LoRaWANBands.cpp:28) - exactly the 1 % that Vfg. 91/2025
     * requires. Deliberately not a hand-typed number.
     *
     * THIS CAN NOW REFUSE AN UPLINK, with RADIOLIB_ERR_UPLINK_UNAVAILABLE.
     * That is the point, and the send path names it explicitly rather than
     * logging it as a generic radio error - a skipped uplink and a broken
     * radio look identical in a log otherwise. */
    /* Save immediately: the nonce advanced even if the join failed, and
     * losing that is what causes the next boot's replay rejections. */
    lw_save_nonces(node.getBufferNonces(), RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
    rtc_save_nonces(node.getBufferNonces(), RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
    rtc_save_session(node.getBufferSession(), RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
    /* NO lw_save_session(). NVS holds the NONCES only, and deliberately.
     *
     * A cold boot never restores an NVS session - its fCnt is behind what the
     * server has already seen, so we join instead (see the restore block
     * above). Writing a session nothing will ever read is not a backstop, it
     * is flash wear with a reassuring name. The nonces are the part that must
     * survive a power cut, because replaying a consumed DevNonce is what gets
     * the next join rejected. */
    Serial.println("joined");

    /* Confirm only on BOTH conditions: the radio joined and the bus answers.
     * Joining alone is not enough - an image that talks to ChirpStack but not
     * to the bridge has silently ended field updates for every probe. */
#if O2P_CLASS_C
    /* Class C AFTER activation, not before: the class is a property of the
     * session, and ChirpStack only believes it once the device profile says
     * supports_class_c AND the session has been (re)activated with it. */
    {
        int cst = node.setClass(RADIOLIB_LORAWAN_CLASS_C);
        class_c_active = (cst == RADIOLIB_ERR_NONE);
        Serial.printf("class C: %s (%d)\n",
                      class_c_active ? "ON - RX2 open continuously" : "FAILED",
                      cst);
        /* The class is part of the session, so persist it where the next
         * wake will actually look - RTC, not NVS. */
        if (class_c_active)
            rtc_save_session(node.getBufferSession(),
                             RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
    }
#endif

    /* WHAT THIS GATE IS FOR, stated rather than left emergent.
     *
     * It proves "this image can still do its job", NOT "this image runs". Those
     * differ, and the difference is deliberate: review finding F11 - the node
     * is the ONLY field-update master for every probe on its bus, so an image
     * that reaches ChirpStack but cannot reach the bridge has silently ended
     * field updates for all of them while looking perfectly healthy.
     *
     * The cost of that choice is real and accepted: an unplugged bridge or an
     * RS-485 fault will roll back a good image, throwing away a transfer that
     * may have taken 43 minutes of radio time. That is the right trade only
     * because the alternative is a node that confirms itself into being unable
     * to update anything, which needs a person at the pile to fix. A false
     * negative costs a retransfer; a false positive costs a site visit.
     *
     * Hence the retries below rather than a single attempt: they reduce
     * noise-induced false negatives without weakening the condition itself.
     *
     * One ping, unconditionally, and both consumers read its result.
     *
     * It used to run only when an image was pending verification, which left
     * bus_silent_at_boot meaning either "the bus answered" or "nobody asked"
     * on every ordinary boot - an unevaluated check reported as a pass. The
     * ping costs one exchange per boot; being able to trust the flag is worth
     * more than that. */
#if PROTO_BUS
    /* SAY WHAT IS BEING ASKED, AND AT WHAT, BEFORE ASKING IT. A ping that
     * fails is about to be reported as a fact about the cable, and a reader of
     * this log cannot check that without the two numbers it was sent at - nor
     * whether they came out of flash or out of the build, which busstore.h is
     * emphatic are different kinds of claim. */
    Serial.printf("probe ping: FC03 to slave 3 at %lu %s (%s)\n",
                  (unsigned long)bus_stored_baud(),
                  bus_parity_name(bus_stored_parity()),
                  bus_store_is_set() ? "stored - the cable answered at this "
                                       "once" : "compiled default");
    bus_silent_at_boot = true;
    for (int attempt = 0; attempt < PROBE_PING_ATTEMPTS; attempt++) {
        if (probe_answers()) { bus_silent_at_boot = false; break; }
        Serial.printf("probe ping %d/%d: no valid reply\n",
                      attempt + 1, PROBE_PING_ATTEMPTS);
    }

    /* IF THE STORED SETTING IS SILENT, ASK AT THE COMPILED ONE BEFORE
     * CONCLUDING ANYTHING ABOUT THE CABLE.
     *
     * busstore.h names this hazard and then does not close it: the stored pair
     * is a MEASUREMENT, but nothing guarantees the device that proved it is
     * still on the cable, and node-and-probe-disagree is SILENCE AT BOTH ENDS
     * with nothing anywhere detecting it. It is not hypothetical on this node.
     * The pair in flash can only have been written while a `bus baud` was on
     * probation - which on this bench was during the era of the o2-probe BRIDGE
     * BOARD, retired 2026-09-09, at a compiled default of 115200 that commit
     * 753aa83 set and this build no longer uses. Worse, bus_prob_note_reply()
     * accepts any CRC-valid frame from anyone (busstore.h, findings 3 and 4),
     * so the answer that proved it need never have come from the probe.
     *
     * A stale pair survives a reflash - it is in NVS, not in the image - and
     * from then on every poll is addressed at a rate the probe is not
     * listening at, forever, with the line below as the only symptom. That is
     * unattended permanent data loss on a buried probe, and the recovery is a
     * laptop at the cable.
     *
     * TWO CANDIDATES, NOT A SWEEP. The stored setting gets the first burst and
     * the compiled setting gets a second; whichever ANSWERS is adopted. A full
     * rate sweep belongs in env:bustest_sweep, not in a field image sharing a
     * pair with the Dragino and the SMT100s. And this is only ever entered
     * after the first burst has already failed, so a healthy node transmits
     * exactly what it transmitted before this change.
     *
     * ADOPTION IS THE FULL probe_answers() TEST - slave 3, FC03, a 16-byte
     * count and the CRC - which is strictly stronger evidence than the
     * any-frame-from-anyone rule that wrote the stale pair in the first place.
     * It is persisted, because a rate held only in a live UART fixes nothing:
     * the next deep-sleep wake re-reads flash.
     *
     * IF NEITHER ANSWERS, the stored pair is put back. Both are unproven at
     * that point, and leaving the node on the one it did not boot with would
     * change what the next boot reports without any evidence for the change.
     *
     * WHO ELSE MAY BE DRIVING THE PAIR, and why this stands down rather than
     * reconfiguring. setup() calls serbus_tick() while it waits for a join
     * (:2016, :2142) and serbus_tick() dispatches console commands, so by the
     * time this runs an operator may already have typed `bus baud` and armed
     * the probation, or opened a serial bus claim for a firmware push. Three
     * refusals, each for a different reason:
     *
     *   bus_prob_active   AN ARMED PROBATION MAKES THIS UNSAFE, not merely
     *                     impolite. bus_prob_note_reply() fires from
     *                     read_frame() on ANY CRC-valid frame and persists
     *                     bus_prob_new_baud/parity - the pair the OPERATOR
     *                     asked for, with no check that it is the pair the
     *                     UART is currently receiving at. So a fallback that
     *                     switched the port to 9600 and then took a reply
     *                     would write the operator's untested 19200 into flash
     *                     as PROVEN. That is an unevaluable input passing a
     *                     persistence guard, and it would corrupt the one
     *                     store this whole mechanism depends on. Found by
     *                     review 2026-09-10; the first version of this block
     *                     had it.
     *   serial_claim      an operator's push is in flight on this cable. The
     *                     `bus` command is allowed to break one because a
     *                     person holding both ends decided to; nothing here
     *                     decided anything, so it must not.
     *   busy_reason()     a BLE or LoRaWAN CTRL session owns the pair.
     *
     * CORRECTED 2026-09-10: this comment claimed a NimBLE-task race that does
     * not exist. BLE callbacks enqueue and bus_service() runs synchronously
     * from ble_ota_tick() (bleota.cpp:672, :718, :746, :1372), which
     * docs/2026-09-09-bus-baud-review.md:194 says in as many words; its
     * finding 8 is about ownership BETWEEN frames, which is what the claim
     * tests above are for. The hazard was real, the mechanism named was not. */
    const char *fb_busy = bus_line_busy_reason();
    if (bus_silent_at_boot && (bus_prob_active || ctrl.serial_claim || fb_busy)) {
        Serial.printf("bus: not retrying at another line setting - %s\n",
                      bus_prob_active
                          ? "a line-setting probation is already armed, and "
                            "reconfiguring under it would store the untested "
                            "pair as proven"
                          : (ctrl.serial_claim
                                 ? "a USB-serial bus claim is open; an operator "
                                   "is on this cable"
                                 : fb_busy));
    } else if (bus_silent_at_boot && bus_direction_ok && bus_store_is_set() &&
        (bus_stored_baud()   != (uint32_t)O2P_UART_BAUD ||
         bus_stored_parity() != (uint8_t)CONSOLE_BUS_PARITY_EVEN)) {
        const uint32_t was_baud   = bus_stored_baud();
        const uint8_t  was_parity = bus_stored_parity();

        Serial.printf("bus: no answer at the stored %lu %s - retrying at the "
                      "compiled %lu %s before blaming the cable\n",
                      (unsigned long)was_baud, bus_parity_name(was_parity),
                      (unsigned long)O2P_UART_BAUD,
                      bus_parity_name(CONSOLE_BUS_PARITY_EVEN));

        if (!bus_line_apply((uint32_t)O2P_UART_BAUD,
                            (uint8_t)CONSOLE_BUS_PARITY_EVEN)) {
            Serial.println("!! bus: the retry could not be set up - R/T did "
                           "not come up at the compiled setting, so nothing "
                           "was transmitted at it");
        } else {
            for (int attempt = 0; attempt < PROBE_PING_ATTEMPTS; attempt++) {
                if (probe_answers()) { bus_silent_at_boot = false; break; }
                Serial.printf("probe ping at the compiled setting %d/%d: no "
                              "valid reply\n", attempt + 1,
                              PROBE_PING_ATTEMPTS);
            }
        }

        if (!bus_silent_at_boot) {
            /* "ANSWERED AT", NOT "THE STORED PAIR WAS STALE". A failure
             * followed by a success establishes that the node reaches slave 3
             * after the switch - not that the line setting is why. The switch
             * also re-ran Serial1.end()/begin() and re-keyed R/T, and the
             * probe may simply have become ready; interference on a shared
             * pair is a third candidate. The pair IS adopted, because the only
             * setting worth booting at is one that has answered, but the log
             * must not hand the next reader a cause it did not measure. */
            Serial.printf("bus: slave 3 answered at the compiled %lu %s - "
                          "adopting it in place of the stored %lu %s\n",
                          (unsigned long)O2P_UART_BAUD,
                          bus_parity_name(CONSOLE_BUS_PARITY_EVEN),
                          (unsigned long)was_baud, bus_parity_name(was_parity));
            if (!bus_store_put((uint32_t)O2P_UART_BAUD,
                               (uint8_t)CONSOLE_BUS_PARITY_EVEN))
                /* NOT "goes silent again": the next wake re-runs this whole
                 * recovery and should reach the probe on its second burst. The
                 * cost is a repeated recovery every wake, and the risk is the
                 * two-key write (busstore.cpp:154-172) having left a MIXED
                 * pair - a stored baud with the other parity - which is a
                 * setting nothing has ever answered at. */
                Serial.println("!! bus: ...but it was NOT persisted. This boot "
                               "reaches the probe; every later wake has to "
                               "rediscover it, and flash may now hold a "
                               "half-written pair. Check `bus` and set it.");
        } else {
            /* NOT "this is the cable, the probe, or the wiring". Two settings
             * failing eliminates two settings. A third rate, framing, this
             * firmware, or another master on the pair are all still open. */
            Serial.printf("bus: no answer at the compiled %lu %s either - "
                          "restoring the stored %lu %s. Neither is proven; "
                          "what is wrong is not established by this test.\n",
                          (unsigned long)O2P_UART_BAUD,
                          bus_parity_name(CONSOLE_BUS_PARITY_EVEN),
                          (unsigned long)was_baud, bus_parity_name(was_parity));
            if (!bus_line_apply(was_baud, was_parity))
                Serial.println("!! bus: the restore FAILED - R/T did not come "
                               "up, so this node can no longer transmit on the "
                               "bus at all");
        }
    }
    /* THREE OUTCOMES, NOT TWO. "Silent" while the direction line is
     * unconfigured is a fault on this board, before a single bit reached the
     * pair, and it must not be read off a console as evidence about the cable
     * or the probe. */
    if (!bus_direction_ok)
        Serial.println("bus at boot: NOT ASKED - RS-485 direction control "
                       "failed to initialise, so nothing was transmitted");
    else
        Serial.printf("bus at boot: %s\n",
                      bus_silent_at_boot ? "SILENT - slave 3 did not answer"
                                         : "slave 3 answered");
#else
    /* Not asked, so not reported. bus_silent_at_boot stays false and the
     * uplink carries POLL_FLAG_BUS_OFF instead - which says the bus was never
     * touched, rather than claiming it answered. */
    Serial.println("PROTO_BUS_OFF: bench build, the RS-485 bus is not touched");
#endif
#if PROTO_MOCK
    /* Loud, and repeated in the banner, because the ONE way this build does
     * harm is by being mistaken for the real one - the uplinks it sends look
     * exactly like measurements unless the payver is read. */
    Serial.println("*** PROTO_MOCK_PROBE: the o2-probe readings in every "
                   "fPort 12 frame are SYNTHETIC (payver 0x82). No probe is "
                   "being measured. ***");
#endif

    /* Make the first measurement happen immediately rather than one interval
     * from now. Unsigned wrap is intentional and correct: `millis() -
     * last_uplink` reads back as POLL_INTERVAL_MS on the first iteration
     * whatever millis() currently is, so the first loop() uplinks.
     *
     * Without this the node would enter maybe_sleep() before ever
     * transmitting, and a cold boot would take five minutes to show any sign
     * of life - indistinguishable, from the ground, from one that did not
     * boot at all. */
    last_uplink = millis() - poll_interval_ms();
    last_poll   = last_uplink;
    /* Seed with NOW, not 0. Zero is a real instant on the millis() circle, and
     * after 2^31 ms of uptime the signed comparisons against it inverte: a
     * valid future deadline reads as past, and maybe_sleep() reads (0 -
     * millis()) as a deadline far in the future and refuses to sleep until the
     * 49.7-day wrap. Unreachable under deep sleep, reachable on a node stuck
     * in the join loop, which is exactly where uptime accumulates. */
    awake_until = millis();

    if (verify_unknown) {
        /* Try to confirm anyway - harmless if the image was never pending,
         * and the only way to save one that was. Never reject on UNKNOWN. */
        if (su_confirm())
            Serial.println("verify state was UNKNOWN; confirm succeeded, so "
                           "the image is valid either way");
        else
            Serial.println("verify state UNKNOWN and the confirm did not take "
                           "- continuing anyway; NOT rolling back on a failed "
                           "query");
        verify_unknown = false;
    }
    if (pending_verify) {
#if PROTO_BUS
        if (!bus_silent_at_boot) {
            /* ONLY clear the flag if the confirm actually took. This used to
             * clear it unconditionally, so a failed confirm was announced as
             * "confirmed" and the bootloader quietly reverted a working image
             * at the next reset. Leaving it set means we try again next pass,
             * which is free, instead of losing the image. */
            if (su_confirm()) {
                pending_verify = false;
                Serial.println("confirmed: joined AND the probe answered");
            } else {
                Serial.println("joined and the bus answered, but the confirm "
                               "did NOT take - retrying, image still provisional");
            }
        } else {
            /* "The bus is silent" is now the only bus fact obtainable, and it
             * includes "the probe is dead for its own reasons" - see
             * PROBE_PING_ATTEMPTS in config.h for what that costs. */
            Serial.println("joined but the bus is silent - rolling back");
            su_reject_and_reboot();          /* does not return */
        }
#elif PROTO_BENCH_CONFIRM_OTA
        /* BENCH ONLY, and it costs exactly the safety the branch below keeps.
         *
         * A PROTO_BUS_OFF board has no bus and no radio to prove itself with,
         * so the branch below never confirms - which is right for a node and
         * wrong for the BLE bench board, where it makes every OTA provisional:
         * the image rolls back at the next reset, and a SECOND push onto the
         * unconfirmed first one is refused outright with
         * ESP_ERR_OTA_ROLLBACK_INVALID_STATE. Measured 2026-09-10: that is what
         * ended a throughput run, and the only exit is a physical reset because
         * nothing in this firmware reboots on command.
         *
         * So this build confirms unconditionally. It is safe here for one
         * reason only: the board is on a cable with somebody watching, and a
         * bad image is recovered with esptool, not with the bootloader. Never
         * set this on an env a node could be flashed from. */
        if (su_confirm()) {
            pending_verify = false;
            Serial.println("confirmed unconditionally (PROTO_BENCH_CONFIRM_OTA "
                           "- bench build, no bus to prove anything with)");
        } else {
            Serial.println("bench confirm did NOT take - image still "
                           "provisional, it will roll back on the next reset");
        }
#else
        /* This build cannot run the check that decides the question, so it
         * must not answer it either way. Declining to confirm means the
         * bootloader rolls the image back at the next reset - the correct
         * outcome for a bench build that reached a node by mistake. */
        Serial.println("!! PENDING VERIFY on a PROTO_BUS_OFF build - the bus "
                       "cannot be proven, so this image is NOT confirmed and "
                       "will roll back on the next reset");
#endif
    }
}

/* One place where a downlink is dispatched, used by BOTH the Class A path
 * (sendReceive) and the Class C path (getDownlinkClassC). Route by fPort,
 * never by the first payload byte.
 *
 * An earlier version dispatched on `dl[0] >= SU_BEGIN && dl[0] <= SU_STATUS`
 * with a comment claiming no Modbus address collides with those opcodes. That
 * was wrong in the same breath as it was written: SU_BEGIN/DATA/COMMIT are
 * 1/2/3 and the bus addresses ARE 1, 2 and 3 - the two SMT100s and the
 * o2-probe. Every probe request would have been swallowed by the self-updater
 * instead of reaching the bus.
 *
 * fPort is the only thing that actually separates the two streams, it is what
 * farm-node-decoder.js already assumes, and it cannot collide with content. */
static void handle_downlink(const uint8_t *dl, size_t dllen, uint8_t fport) {
    if (fport == O2P_CTRL_FPORT) {
        /* DECIDE ON A COPY, COMMIT ONLY IF THE HOST WILL HEAR IT.
         *
         * ctrl_handle() used to mutate the live session and the ACK was then
         * pushed with its return value ignored. With a four-deep queue that
         * meant a session could be granted - two hours of open receiver, the
         * bus taken off the poller - while the host never learned it, and went
         * on retrying against a node that had already said yes. A grant nobody
         * is told about is worse than a refusal. */
        CtrlState next = ctrl;
        uint8_t a[16];
        size_t  n = ctrl_handle(dl, dllen, uplink_nonce, CTRL_MAX_SESSION_MIN,
                                rtc_uptime_s(),
                                &next, a, sizeof(a));
        if (n && reply_push(a, n, O2P_CTRL_FPORT)) {
            ctrl = next;
            /* The window moved with it, so carry it into RTC in the same
             * breath - the commit point is the only place that knows the
             * decision was actually taken. */
            ctrl_window_persist();
            /* COMMITTED ONLY HERE, inside the same "the host will hear it"
             * gate the session grant uses. A cadence changed on a downlink the
             * host never got an ACK for would leave the two sides disagreeing
             * about the interval, and that disagreement does not look like an
             * error from either end - it looks like missing data. */
            if (next.cadence_updated && next.cadence_persist) {
                /* CLEAR THE LEASE TOO, and not as tidiness. A live lease
                 * outranks the stored value in poll_interval_ms(), so storing
                 * one while a lease was running would appear to do nothing
                 * until the lease lapsed - minutes later, with no message. The
                 * operator would reasonably conclude the write had failed and
                 * write it again. */
                cadence_clear(rtc_cadence());
                bool stored = cad_store_s(next.cadence_s);
                if (!stored)
                    Serial.printf("!! cadence: %u s was GRANTED to the host but "
                                  "NOT stored - the ACK already said yes, so the "
                                  "host believes this survived. It will revert at "
                                  "the next reset\n", (unsigned)next.cadence_s);
                else if (next.cadence_s)
                    Serial.printf("cadence: %u s STORED in NVS (CTRL) - survives "
                                  "reset, power cycle and OTA\n",
                                  (unsigned)next.cadence_s);
                else
                    Serial.println("cadence: stored value REMOVED (CTRL) - back "
                                   "to the compiled default");
            } else if (next.cadence_updated) {
                cadence_commit(rtc_cadence(), next.cadence_s,
                               next.cadence_lease_min, rtc_uptime_s());
                if (next.cadence_s)
                    Serial.printf("cadence: %u s for %u min (CTRL)\n",
                                  (unsigned)next.cadence_s,
                                  (unsigned)next.cadence_lease_min);
                else
                    Serial.printf("cadence: lease cleared (CTRL) - back to %lu ms\n",
                                  (unsigned long)cad_default_ms());
            }
            /* THE RADIO FOLLOWS THE SESSION, and only after the ACK is queued.
             * Bringing BLE up on the decision copy would have meant an OPEN
             * that was granted-but-unannounced still put an unauthenticated
             * write path to esp_ota_write() on the air, with the host - the
             * only party that could close it - unaware there was anything to
             * close.
             *
             * The window is the SESSION's remaining time, not the granted
             * duration: a renewal arriving mid-window must not stack. It is
             * clamped again to BLE_OTA_MAX_UP_MS inside ble_ota_up(), which is
             * the shorter of the two ceilings and the one that matters. */
            if (ctrl.ble_ota)
                ble_ota_up(ctrl.nonce, ctrl.expires_ms - millis());
            else
                ble_ota_down("CTRL session closed or did not ask for BLE");
            /* Only accepted traffic buys awake time. Holding for 60 s before
             * validating meant a stream of malformed frames arriving once a
             * minute kept the node awake indefinitely while every one of them
             * was refused.
             *
             * THE REFUSAL BRANCH WAS STILL UNBOUNDED, which made the sentence
             * above only half true: ctrl_handle() answers malformed ops and
             * stale nonces with an ACK too, and a queued ACK was treated as
             * acceptance, so a repeating stale queue item bought 60 s every
             * time while being refused every time. */
            if (ctrl.open) {
                note_accepted_traffic();
                hold_awake(ctrl.expires_ms - millis());
            } else {
                hold_awake_unaccepted("ctrl");
            }
            Serial.printf("  -> ctrl: open=%d bus=%d\n",
                          (int)ctrl.open, (int)ctrl.bus_claimed);
        } else {
            Serial.println("!! CTRL ACK could not be queued - session NOT "
                           "changed, the host must ask again");
        }
        return;
    }
    if (fport == O2P_SU_FPORT) {
        /* A BEGIN starts a NEW transfer, so every status still queued from the
         * old one is obsolete -- and worse than useless: each goes out on a
         * fresh uplink with a HIGHER fCnt, so the host's fCnt correlation
         * cannot tell it from a real reply. Measured 2026-09-06: a BEGIN was
         * answered by a queued status from the previous aborted run reporting
         * "RECEIVING/SEQUENCE next_seq=8", and the push aborted against a node
         * that had in fact accepted the BEGIN. Drop the backlog here. */
        if (dllen >= 1 && dl[0] == SU_BEGIN && rq_count) {
            Serial.printf("BEGIN: dropping %u stale status replies\n",
                          (unsigned)rq_count);
            rq_head = rq_tail = rq_count = 0;
        }
        uint8_t st[MAX_FRAME];
        size_t  n = su_handle(dl, dllen, st, sizeof(st), class_c_active);
        /* n == 0 is a deliberately silent DATA accept - see su_handle. Only
         * BEGIN/COMMIT/ABORT/STATUS and errors produce a frame to send back,
         * which is what lets the host burst blocks without a reply each.
         *
         * THE QUIET FLAG IS class_c_active, AND THAT COUPLES US TO THE HOST'S
         * BURST SIZE. In Class C a DATA block is acknowledged by nothing at
         * all, so a host sending ONE block and then waiting for a reply waits
         * forever - the node is behaving exactly as designed and the transfer
         * is stalled. Measured 2026-09-06 by the session driving the pusher:
         * `--burst 1`, which is the pusher's DEFAULT, is silently incompatible
         * with a Class C node, and the symptom that identified it was this
         * function's own "-> selfupdate, 0 status bytes" line.
         *
         * Left as it is rather than "fixed" here: acknowledging every block
         * would throw away the throughput the burst exists for, and the node
         * cannot see burst boundaries - it has no idea which block is the last
         * one the host intends to send before listening. So the constraint is
         * real and belongs on the host: burst > 1 in Class C, or stay Class A.
         * Recorded here because this line is where the coupling is created,
         * and it is invisible from the host end. */
        if (n) reply_push(st, n, O2P_SU_FPORT);
        /* A frame this session actually acted on produces a status frame; an
         * unrecognised one produces nothing (su_session_handle returns 0) and
         * must not buy awake time on its own account. */
        /* A REFUSAL RETURNS A STATUS FRAME TOO, so `n` alone cannot say
         * whether this frame was accepted - which is how a stream of
         * malformed but recognised frames kept buying unbudgeted awake time
         * while every one of them was rejected. */
        if (ctrl.open) {
            note_accepted_traffic();
            hold_awake(AWAKE_HOLD_MS);
        } else if (n && su_session_last_accepted()) {
            hold_awake(AWAKE_HOLD_MS);
        } else {
            hold_awake_unaccepted("selfupdate");
        }
        Serial.printf("  -> selfupdate, %u status bytes\n", (unsigned)n);
    } else {
#if PROTO_BUS
        relay(dl, dllen);
#else
        /* A relay downlink is a request to drive the bus, and this build must
         * not. Refusing here rather than in relay() keeps the refusal next to
         * the routing decision that would otherwise reach it.
         *
         * This was the live hazard: fPorts other than 11 and 13 fall through
         * to relay(), so ANY fPort 10 frame - enqueued from the ChirpStack UI,
         * a host tool, or left in a queue from an earlier session - actively
         * drove the CP2102-shared net from a build documented as driving no
         * pads. Nothing about PROTO_BUS_OFF stopped it.
         *
         * Silence is the right answer to the host, and it is not ambiguous:
         * the scheduled fPort 12 frames carry POLL_FLAG_BUS_OFF continuously,
         * so a host that sees no reply can already tell "not asked" from "did
         * not answer" without this path saying anything. */
        Serial.printf("relay: REFUSED %u bytes - PROTO_BUS_OFF, this build "
                      "never drives the bus\n", (unsigned)dllen);
        (void)dl;
#endif
        /* A relay frame is unauthenticated by construction - it is whatever
         * was in the downlink queue. Inside an open CTRL session that is the
         * legitimate probe-update path and stays unbounded; outside one it is
         * budgeted like any other unaccepted traffic. */
        if (ctrl.open) {
            note_accepted_traffic();
            hold_awake(AWAKE_HOLD_MS);
        } else {
            hold_awake_unaccepted("relay");
        }
    }
}

/* Sleep if there is nothing left to do before the next measurement.
 *
 * Every condition here is a reason NOT to sleep, and each is separate on
 * purpose: a single "busy" flag maintained in five places is how a node ends
 * up asleep in the middle of a firmware transfer.
 */
/* Put the BOARD to sleep, not just the CPU.
 *
 * esp_deep_sleep() powers down the ESP32 and nothing else. The SX1262 is a
 * separate chip on its own 3V3 rail, and this firmware runs Class C - which
 * means RadioLib has parked it in CONTINUOUS RECEIVE. Without this function it
 * stayed there for the whole 300 s interval, listening into an MCU that was
 * asleep and could not act on anything it heard.
 *
 * The arithmetic is what makes it the first thing to fix rather than a
 * refinement. SX1262 datasheet (DS.SX1261-2.W.APP, rev 2.1, table 3-2):
 * continuous RX is 4.6 mA with the internal DC-DC and 10.1 mA on the LDO;
 * sleep with cold start is 600 nA. At 3.3 V that is 15-33 mW held for 96 % of
 * every cycle, against the ~11 mW AVERAGE the whole node is budgeted at in
 * config.h. The radio alone was therefore costing more than the entire node
 * was supposed to - and buying nothing, because a Class C downlink arriving
 * while the MCU is in deep sleep is not received by anything.
 *
 * These are datasheet figures, NOT measured on this board: nobody has put a
 * meter on it, and config.h is explicit that the ~40 mW board estimate has
 * never been measured either. Treat the improvement as "the radio stops
 * drawing RX current", which is certain, and not as a runtime figure, which
 * is not.
 *
 * The RF-switch pins matter for the same reason at smaller scale: the
 * Core1262-HF carries a PA and an LNA, and an ESP32 pad left floating through
 * deep sleep can bias RXEN high and hold the LNA on. Driving them low and
 * latching the level costs nothing and removes the question. */
static void park_for_sleep(void)
{
    /* Cold start (retainConfig = false) is the deeper of the two SX1262 sleep
     * modes and safe here precisely because setup() re-runs radio.begin() and
     * setTCXO() on every wake - there is no configuration worth retaining
     * across a reset that reinitialises the part anyway. */
    int st = radio.sleep(false);
    if (st != RADIOLIB_ERR_NONE) {
        /* Say so rather than sleep on a radio that may still be receiving.
         * This is a console message on a port deep sleep disconnects, which is
         * exactly the criticism this codebase keeps making of itself - but the
         * alternative is refusing to sleep, and a node that stops measuring
         * because its radio would not park is worse than one that measures
         * while drawing too much. */
        Serial.printf("!! radio.sleep() returned %d - parking anyway\n", st);
    }

    pinMode(PIN_LORA_RXEN, OUTPUT); digitalWrite(PIN_LORA_RXEN, LOW);
    pinMode(PIN_LORA_TXEN, OUTPUT); digitalWrite(PIN_LORA_TXEN, LOW);
    gpio_hold_en((gpio_num_t)PIN_LORA_RXEN);
    gpio_hold_en((gpio_num_t)PIN_LORA_TXEN);

#if PROTO_BUS
    /* HOLD R/T LOW. THE POLARITY IS THE WHOLE POINT, AND IT IS THE OPPOSITE OF
     * WHAT THIS BLOCK USED TO DO.
     *
     * With the bridge board the pin that mattered was TX and the safe level was
     * HIGH, because high is what an idle UART looks like and a run of low on
     * the STM32's RX is a BREAK. That fault cost one node one poll.
     *
     * The pin that matters now is the transceiver's direction line, and the
     * cost is not symmetric. A node that sleeps with R/T high leaves the driver
     * keyed onto the pair for 96 % of every cycle, which does not mute this
     * node - it mutes the CABLE: the o2-probe, both SMT100s and the Dragino,
     * all of them silent while a sleeping ESP32 holds the line. Deep sleep is
     * exactly when nobody is watching a console, so it would be found from
     * missing data on three devices at once.
     *
     * Driven low and latched, so the level survives the sleep rather than
     * drifting; the click's pull-down agrees with it, so the pad and the board
     * pull the same way.
     *
     * TX IS NO LONGER HELD. Its idle level stopped mattering when the STM32's
     * receiver left the harness - what the pad feeds now is the click's UART
     * input, whose driver is off whenever R/T is low. */
    pinMode(PIN_BUS_DE, OUTPUT); digitalWrite(PIN_BUS_DE, LOW);
    gpio_hold_en((gpio_num_t)PIN_BUS_DE);
#endif

    gpio_deep_sleep_hold_en();
}

/* Called from esp-ota-ble's restart hook, which fires from inside otaBleEnd()
 * once the new image is written and the boot slot selected.
 *
 * NO gpio_hold_en() here, unlike park_for_sleep(). A hold is meant to survive
 * a sleep that the same firmware wakes from and releases; latching pads on the
 * way into a reset that lands in a DIFFERENT image means the new one comes up
 * with pads it never configured and does not know to release. Driving the
 * RF-switch pins low is enough - the reset itself is what ends the LNA being
 * biased on, and a few milliseconds of a floating pad is not worth handing the
 * incoming image a puzzle. */
void node_prepare_restart(void)
{
#if !PROTO_BLE_BENCH
    radio.sleep(false);
#endif
    pinMode(PIN_LORA_RXEN, OUTPUT); digitalWrite(PIN_LORA_RXEN, LOW);
    pinMode(PIN_LORA_TXEN, OUTPUT); digitalWrite(PIN_LORA_TXEN, LOW);
    Serial.flush();
    delay(50);
}

/* USB PRESENCE, SAMPLED RATHER THAN GLANCED AT.
 *
 * usb_present_tick() is called from loop() on every iteration; the sleep gate
 * below asks usb_present() instead of calling isPlugged() itself. The two are
 * NOT the same question. isPlugged() answers "is the host emitting SOF right
 * now", which goes false whenever macOS suspends an idle CDC, and the sleep
 * decision used to be taken on exactly one such reading. usb_present() answers
 * "has this host been emitting SOF at any point in the last
 * USB_PRESENT_HOLD_MS", which is what "the cable is in and someone is at the
 * bench" actually looks like from in here. See config.h for the measurement
 * that forced the change and for why the hold is bounded.
 *
 * millis() ONLY, AND DELIBERATELY NOT RTC MEMORY. The latch must not survive a
 * deep-sleep wake: if it did, one cabled boot would suppress sleep for every
 * subsequent wake with no host present at all. A wake starts with the latch
 * clear and the first tick re-establishes it within one loop iteration if a
 * host is really there.
 *
 * SUBTRACT, DO NOT COMPARE - the same millis() wrap rule read_frame() spells
 * out, and for the same reason: a comparison would misread the window across
 * the 49.7-day wrap.
 *
 * `usb_seen_ever` EXISTS BECAUSE ZERO IS AMBIGUOUS. A node that has never seen
 * a host has usb_seen_ms == 0, and for the first USB_PRESENT_HOLD_MS after
 * boot the subtraction alone would read that as "seen just now" and hold an
 * uncabled node awake through its first five minutes of every wake. The flag
 * is what separates "never" from "long ago". */
static uint32_t usb_seen_ms   = 0;
static bool     usb_seen_ever = false;

static void usb_present_tick(void) {
#if PROTO_USB_KEEPS_AWAKE && ARDUINO_USB_CDC_ON_BOOT
    if (HWCDC::isPlugged()) {
        usb_seen_ms   = millis();
        usb_seen_ever = true;
    }
#endif
}

static bool usb_present(void) {
#if PROTO_USB_KEEPS_AWAKE && ARDUINO_USB_CDC_ON_BOOT
    if (!usb_seen_ever) return false;
    return (uint32_t)(millis() - usb_seen_ms) < (uint32_t)USB_PRESENT_HOLD_MS;
#else
    return false;
#endif
}

static void maybe_sleep(void) {
    if (!PROTO_SLEEP) return;


    /* AN UNVERIFIED IMAGE MUST NOT SLEEP. This is not tidiness; read it before
     * relaxing it.
     *
     * ESP-IDF gives a new image exactly one boot in PENDING_VERIFY. Any reset
     * before it calls esp_ota_mark_app_valid_cancel_rollback() - and a
     * deep-sleep wake IS a reset - makes the bootloader mark the image aborted
     * and select the previous slot. So an image that slept while pending would
     * be reverted on its very next wake.
     *
     * What that looks like from the ground is the problem. A rescue image
     * pushed over a 43-minute LoRaWAN window reports a successful transfer,
     * reboots, sleeps, and comes back running the OLD firmware. That is
     * indistinguishable from a transfer that failed, and it sends whoever
     * debugs it to the link budget - where this project has already lost
     * several days to an unrelated bug. The image was fine; the sleep ate it.
     *
     * So: stay awake, confirm or reject, and only then sleep. */
    if (pending_verify) return;

    /* A transfer is open, or a staged image is waiting for its reboot. Both
     * mean the host is mid-conversation with us. */
    if (su_busy() || su_reboot_armed()) return;

    /* A CTRL session means someone asked us to stay reachable. */
    if (ctrl.open) return;

    /* The BLE radio is up. It can outlive the CTRL session by up to
     * BLE_OTA_GRACE_MS when a transfer is actually moving, so this is a
     * SEPARATE test and not a consequence of ctrl.open - deep sleep would
     * reset the chip out from under a push that is seconds from finishing,
     * and the host would see the disconnect that means success.
     *
     * FAST BLE TELEMETRY DEPENDS ON THIS LINE AND ADDS NO RULE OF ITS OWN.
     * A BLE link cannot survive deep sleep, so a node streaming telemetry must
     * stay awake; it can only stream while the radio is up, so the inhibit it
     * needs is this one - already here, already the tested path. What the
     * telemetry feature adds is the other half: an arming LEASE, so a stream
     * left running by an operator who drove away ends by itself rather than
     * holding the node awake until the battery is flat. telemetry.h records
     * why staying awake was chosen over reconnecting on every fast sample. */
    if (ble_ota_is_up()) return;

    /* A USB HOST HAS THE CONSOLE OPEN, so somebody is standing at the bench.
     *
     * This is the answer to "wake it up on USB input", which the hardware
     * cannot do: see the note on ESP32-S3 USB wake in README.md. Deep sleep
     * powers the USB peripheral down and the CDC device disappears from the
     * host entirely, so there is nothing left to type into and no wake source
     * for it to drive. Staying awake WHILE the port is open is the reachable
     * half of that wish, and it is the half that matters on the bench - the
     * node stops disappearing mid-session.
     *
     * COSTS NOTHING IN THE FIELD, and that is a documented guarantee rather
     * than an assumption: isPlugged() is true only while the host is emitting
     * SOF packets, so a power bank, a charger, a dead cable or a floating VBUS
     * all read false - see the note on PROTO_USB_KEEPS_AWAKE in config.h. A
     * host that suspends stops SOF too, and the node sleeps again by itself.
     *
     * Set PROTO_USB_KEEPS_AWAKE to 0 for a build that might sit cabled to a
     * live host indefinitely; that is the one case SOF gating does not
     * bound. */
    /* SAMPLED, NOT GLANCED AT. isPlugged() here read false whenever the host
     * happened to be between SOF bursts at this exact instant, which is how a
     * cabled node slept anyway - see usb_present() above. */
#if PROTO_USB_KEEPS_AWAKE && ARDUINO_USB_CDC_ON_BOOT
    if (usb_present()) return;
#endif

    /* Something is queued to go up: send it before sleeping, or it waits five
     * minutes for a slot it could have had now. */
    if (rq_count > 0) return;

    /* The listening window, and any extension a downlink has bought. */
    if ((int32_t)(awake_until - millis()) > 0) return;

    uint32_t elapsed = millis() - last_poll;
    uint32_t sleep_ms = 0;
    if (elapsed < poll_interval_ms()) {
        /* Sleep only the remainder, so a long awake period does not push the
         * next measurement a full interval into the future - the cadence is
         * what the series is read against. */
        sleep_ms = poll_interval_ms() - elapsed;
    } else if (duty_defer_ms > 0u) {
        /* THE POLL IS DUE AND THE BAND OWES SILENCE, so sleep through the debt
         * rather than spinning awake for it.
         *
         * The cadence has elapsed, so the test above declines - which is right
         * when a measurement is about to be taken and wrong here, because the
         * duty gate has already refused and nothing will be sent until the
         * debt clears. Without this the loop turned over every 50 ms, or every
         * 5 ms in Class C, for the whole wait: fifteen 595 ms frames owe 893 s,
         * about ten minutes awake at a 300 s cadence. That is the battery cost
         * the deferral was accidentally buying in exchange for not deleting the
         * measurement. Found by review 2026-09-10.
         *
         * Waking a little early is harmless - the gate is re-asked on the next
         * pass - so no margin is added and the debt is not padded. */
        sleep_ms = duty_defer_ms;
    } else {
        return;                                 /* due now; do not sleep past it */
    }

    /* NEVER SLEEP ACROSS THE HEATER ARM'S DEADLINE.
     *
     * This consulted neither the lease nor the retry deadline, so at a long
     * cadence - 3600 s is permitted - a relayed ARM followed by the listening
     * window could put the node to sleep straight across its 300 s lease. The
     * node is the only thing that can send the disarm, and it cannot send it
     * asleep; UNPROVEN-on-wake then cleaned up one whole cadence late, which
     * is eventual cleanup, not a 300 s bound. Review 2026-09-10.
     *
     * So cap the sleep at the deadline. Waking there is enough: `ctrl` is not
     * carried across deep sleep, so the woken node starts UNPROVEN and the loop
     * issues the disarm on its first pass. A deadline too close to be worth a
     * sleep-and-wake cycle keeps the node awake instead. */
    uint32_t arm_left_s = 0;
    if (ctrl_arm_sleep_bound_s(&ctrl, rtc_uptime_s(), &arm_left_s)) {
        if (arm_left_s <= 10u) return;
        const uint32_t arm_cap_ms = arm_left_s * 1000UL;
        if (sleep_ms > arm_cap_ms) sleep_ms = arm_cap_ms;
    }

    park_for_sleep();
    rtc_deep_sleep(sleep_ms);                       /* does not return */
}

/* The flags byte, in ONE place.
 *
 * Extracted when the BLE telemetry path started building the same frame: two
 * copies of this would drift, and the way they would drift is that the BLE
 * copy of a measurement would describe a different node state than the
 * LoRaWAN copy of the SAME measurement - which is worse than either being
 * wrong, because it makes the two series disagree about a node that was fine.
 *
 * `su_open` is a parameter rather than a call because the caller has already
 * decided, on the same value, whether to touch the bus at all; reading
 * su_busy() twice would let the two answers differ inside one frame. */
static uint8_t poll_flags_now(bool su_open) {
    uint8_t flags = 0;
    if (class_c_active)         flags |= POLL_FLAG_CLASS_C;
    if (su_reboot_armed())      flags |= POLL_FLAG_SU_STAGED;
    /* ONLY WHEN THE BUS WAS ACTUALLY ASKED. bus_silent_at_boot is set true
     * before the ping loop and nothing clears it when probe_answers() refuses
     * for want of a direction pin - so without this term a node that cannot
     * key its driver uplinked BUS_SILENT *and* BUS_OFF: the cable-is-dead lie
     * this was written to remove, plus a second claim contradicting it. The
     * previous commit said BUS_OFF replaced BUS_SILENT here; it did not, until
     * now.
     *
     * The variable itself is deliberately NOT cleared: su_confirm() reads it as
     * the self-update gate, and there its true is right - a node that cannot
     * reach the bus must not vouch for a freshly installed image. */
    if (bus_silent_at_boot && bus_direction_ok)
                                flags |= POLL_FLAG_BUS_SILENT;
    if (su_open)                flags |= POLL_FLAG_SU_BUSY;
    if (ctrl.bus_claimed)       flags |= POLL_FLAG_BUS_CLAIMED;
    /* BUS_OFF ALSO MEANS "THE DIRECTION PIN NEVER CAME UP".
     *
     * All eight flag bits are spoken for, so rather than spend a payver on a
     * ninth, this reuses the bit whose meaning already fits: BUS_OFF is the
     * NOT-ASKED flag - poll.h and farm-node-decoder.js both read it as "the
     * node deliberately never drove the pair", as distinct from BUS_SILENT's
     * "it was asked and did not answer". A node that cannot key its driver did
     * not ask, so that is the honest bit. Without this the frame would claim
     * the probe was silent, sending someone to the pile for a fault that is in
     * the node's own hand. */
    if (!PROTO_BUS || !bus_direction_ok)
                                flags |= POLL_FLAG_BUS_OFF;
    if (warm_boot)              flags |= POLL_FLAG_WOKE_SLEEP;
    /* A dropped reply used to be reported only by printf, to a USB console
     * that a buried node does not have and deep sleep disconnects anyway.
     * The counter existed so a loss would be visible rather than silent -
     * which it was not, to anyone who could act on it. Put it on the air. */
    if (rq_dropped)             flags |= POLL_FLAG_REPLY_DROPPED;
    return flags;
}

/* ---- FAST TELEMETRY OVER BLE ---------------------------------------------
 *
 * The control half: drain the `telem` verb's latch, expire the arming, and
 * notice when the radio has gone out from under an armed stream. Runs early in
 * loop() so that a `telem 0` typed this iteration is honoured before anything
 * decides whether to keep the radio up on the stream's behalf.
 *
 * RTC memory is this task's to write, and the verb arrives on the NimBLE host
 * task - the same hand-off, for the same reason, as the cadence latch beside
 * it. */
static void telemetry_control(void) {
#if PROTO_BLE_OTA
    uint16_t want_s = 0, want_lease = 0;
    if (ble_take_telem_request(&want_s, &want_lease)) {
        uint16_t got_s = 0, got_lease = 0;
        uint8_t r = telem_clamp(want_s, want_lease, &got_s, &got_lease);
        telem_arm(&telem, got_s, got_lease, rtc_uptime_s(), millis());
        ble_report_telem(got_s, got_lease, r == TELEM_CLAMPED);
        if (got_s)
            Serial.printf("telem: fast BLE stream every %u s for %u min\n",
                          (unsigned)got_s, (unsigned)got_lease);
        else
            Serial.println("telem: fast BLE stream off");
    }

    if (telem_expire(&telem, rtc_uptime_s()))
        Serial.println("telem: lease expired - fast BLE stream stopped");

    /* THE RADIO WENT DOWN UNDER AN ARMED STREAM. Every BLE teardown path ends
     * here eventually - the cumulative ceiling, an exhausted auth counter, a
     * CTRL session that expired with nobody listening - and an arming that
     * outlived its radio would keep telem_armed_now() true, which is the
     * predicate the teardown itself consults. Clearing it here is what stops
     * the two disagreeing; the lease alone would take minutes to do it. */
    if (telem.seconds && !ble_ota_is_up()) {
        telem_disarm(&telem);
        Serial.println("telem: BLE radio is down - fast stream disarmed");
    }
#endif
}

/* The emitting half. One frame, built by the SAME builder the LoRaWAN uplink
 * uses, so the two transports cannot carry different shapes of the same
 * measurement.
 *
 * Deliberately does NOT touch last_poll: that stamp is the LoRaWAN schedule,
 * and a fast BLE session must not push the legal 60 s uplink around. The two
 * paths poll the bus independently and both see whatever the probe currently
 * reads, which is the honest answer - the probe samples continuously, so
 * neither is a resample of the other's cached value.
 *
 * Called from loop() only after the ble_ota_flash_busy() early return, so a
 * Modbus exchange can never start inside an erase. */
/* WHERE THIS FRAME'S NUMBERS COME FROM. One decision, made once, because the
 * LoRaWAN uplink and the fast BLE stream carry THE SAME FRAME and a difference
 * between them would be two copies of one measurement describing two different
 * nodes - which is the property both call sites already went out of their way
 * to preserve by spelling `bus_held` identically.
 *
 *   POLL_XCHG         the ordinary case: drive the bus now.
 *   claim_cache_xchg  a CTRL bus claim is held, so the poller must not drive
 *                     the bus - but the claim holder has been driving it, and
 *                     what it got is a measurement. See pollcache.h.
 *   NULL              nothing may be asked and nothing is known.
 *
 * THE CACHE IS FOR THE CLAIM AND FOR NOTHING ELSE. `bus_held` has three
 * causes and they are not interchangeable:
 *
 *   a claim              somebody is on the bus - there is a cache behind it
 *   PROTO_BUS_OFF        this build never drove a pad, so the cache is empty
 *                        by construction and always was
 *   !bus_direction_ok    the R/T pin did not come up: NOTHING on this node can
 *                        transmit, including ble_bus_exchange(), so anything
 *                        in the cache predates the fault
 *
 * The last one is the dangerous one and the reason this is not just
 * `if (ctrl.bus_claimed)`. A node whose direction pin failed is in permanent
 * unattended data loss (poll.h, POLL_FLAG_BUS_OFF), and serving stale records
 * across that would dress the fault up as a working probe - the reassuring
 * direction, which poll.h is already emphatic is the worse one. The age bound
 * would eventually clear it, but "eventually" is not a guard. Test the fault
 * directly.
 *
 * su_open keeps its old behaviour deliberately: a firmware transfer is not a
 * bus claim, nobody is reading the probe during one, and widening the cache to
 * cover it would be a second change riding along on this one.
 *
 * A USB-SERIAL CLAIM ALSO SETS ctrl.bus_claimed AND GETS NOTHING, which is a
 * consequence rather than an oversight: the serial bridge drives the pair
 * through serbus_tick(), not through ble_bus_exchange(), so the cache is never
 * filled and every lookup misses. The frame is header-only exactly as it was
 * before - which is the right default, and the case is a bench cable with an
 * operator standing at it rather than an unattended node in a windrow.
 *
 * AND YES, THE BLE STREAM GETS THE CACHE BACK. telemetry_emit() calls this
 * too, so a subscribed client holding a claim is notified with readings it
 * produced. That is mildly circular and entirely harmless, and the
 * alternative - a different source for the two transports - would break the
 * one property both call sites exist to preserve. */
static PollExchangeFn poll_source(bool su_open, bool bus_held)
{
    if (su_open)   return NULL;
    if (!bus_held) return POLL_XCHG;
    if (ctrl.bus_claimed && PROTO_BUS && bus_direction_ok)
        return claim_cache_xchg;
    return NULL;
}

static void telemetry_emit(void) {
#if PROTO_BLE_OTA
    if (!telem_due(&telem, rtc_uptime_s(), millis())) return;
    /* Nobody listening: do not spend the bus. A notification to an
     * unsubscribed peer is dropped by the stack, so this would be a poll for
     * nobody every few seconds - see ble_telem_subscribed(). Not stamping is
     * deliberate too: the first frame after somebody subscribes goes out at
     * once rather than one interval later. */
    if (!ble_telem_subscribed()) return;

    bool su_open  = su_busy();
    /* Identical to the uplink's test, and identical because it must be: the
     * BLE frame is the same frame, and a bus this node is not allowed to touch
     * is not allowed to be touched for a different consumer. A held bus yields
     * a header-only frame whose flags SAY why, exactly as the uplink does -
     * silence would be indistinguishable from a stalled stream. */
    /* PROTO_BUS-GUARDED. bus_direction_ok is only ever ASSIGNED inside
     * #if PROTO_BUS, so in a bus-off build it keeps its false initialiser for
     * the life of the image - and an unguarded disjunct is then permanently
     * true, which silenced env:esp32s3_mock: header-only frames, no synthetic
     * records, and the whole server chain that build exists to exercise fed
     * nothing, quietly and on schedule. The (!PROTO_BUS && !PROTO_MOCK) term
     * right here exists to stop exactly that, and the new one stepped over it.
     * PROTO_BUS is 0/1, so this folds away. */
    bool bus_held = ctrl.bus_claimed || (!PROTO_BUS && !PROTO_MOCK) ||
                    (PROTO_BUS && !bus_direction_ok);
#if PROTO_MOCK
    mock_probe_set_time(rtc_uptime_s());
#endif
    static uint8_t telembuf[MAX_FRAME];
    /* NO getMaxPayloadLen() CAP, and no nonce. See bleota.h: BLE has no data
     * rate to fit inside, and a live session nonce in cleartext would hand a
     * listener the `auth` gate that same nonce opens. Zero is not a nonce the
     * node will ever accept, so a frame from here cannot be replayed into a
     * SESSION_OPEN either. */
    size_t n = poll_build_frame(poll_source(su_open, bus_held),
                                poll_flags_now(su_open), rtc_uptime_s(), 0,
                                telembuf, sizeof(telembuf));
    if (n == 0) return;
    /* Stamped ONLY on a frame that actually left. A send that failed - an
     * empty mbuf pool, a peer that vanished between the check and the notify -
     * is retried on the next pass rather than counted, or the stream would
     * silently thin to one frame per failure. */
    if (ble_telem_send(telembuf, n)) telem_sent(&telem, millis());
#endif
}

#if PROTO_BUS
/* Reliable Modbus FC06 disarm with retries and response validation. */
static bool o2p_disarm_heater_sync(void) {
    static const uint8_t disarm_cmd[] = { 0x03, 0x06, 0x00, 0x60, 0x00, 0x00, 0x88, 0x36 };
    uint8_t rsp[MAX_FRAME];

    /* SAY IT HERE, LOUDLY, RATHER THAN LET IT LOOK LIKE A SILENT PROBE.
     *
     * bus_write() refuses when the direction pin never came up, so without this
     * the three retries below each drain a full REPLY_WAIT_MS through the
     * quarantine - about 7.5 s of blocked loop() - and end in the caller's
     * "heater disarm attempt unacknowledged", which points at the probe. The
     * truth is worse and is on this board: the node cannot disarm the heater AT
     * ALL, and will not be able to until it reboots, because it cannot key the
     * transceiver. That is the one case where bus_write()'s silent refusal is
     * more dangerous than transmitting would have been, so it gets its own line
     * and returns immediately instead of pretending to try. */
    if (!bus_direction_ok) {
        Serial.println("!! HEATER DISARM IMPOSSIBLE - the RS-485 direction pin "
                       "did not come up, so no command can reach the probe. "
                       "This is a fault on the NODE, not a silent probe. The "
                       "heater cannot be commanded off until this is fixed.");
        return false;
    }

    for (int retry = 0; retry < 3; retry++) {
        size_t rlen = poll_exchange(disarm_cmd, sizeof(disarm_cmd), rsp, sizeof(rsp));
        /* THE WHOLE ECHO, INCLUDING THE VALUE AND THE CRC.
         *
         * This accepted rlen >= 6 with four matching header bytes and checked
         * neither the echoed value nor the CRC, so it could not tell a real
         * FC06 echo from any six bytes that happened to start that way. Since
         * read_frame() now assembles a reply across gap windows, two unrelated
         * bursts - `03 06 00 60` then `12 34` - join into something this
         * accepted, and the caller recorded the arm as OFF and suppressed the
         * lease cleanup that is the entire point of the lease. The weak check
         * predates the resync; the resync gave it a second route in.
         *
         * A conforming FC06 reply echoes the request exactly: address,
         * function, register, THE VALUE WRITTEN (zero, because that is what
         * disarms), and a valid CRC over all six. Nothing less is evidence
         * that the probe disarmed. */
        /* rlen == 8 WAS TOO STRICT, and it broke the safety path it tightened.
         *
         * mb_resync() trims a trailing turnaround zero only for FC03/FC04 -
         * an FC06 reply keeps it - so a perfectly conforming echo followed by
         * the artefact this whole bus is known to produce arrives as NINE
         * bytes and was rejected. A real disarm then became unconfirmable, the
         * lease stayed HELD/UNPROVEN, and it retried forever: 60 s, then 900 s
         * after three failed batches, with no terminal give-up. The probe was
         * already disarmed by the write, so nothing was energised - but the
         * confirmation and retry behaviour was a regression I introduced.
         * Found by review 2026-09-10.
         *
         * So: the first eight bytes must be the exact echo with a valid CRC,
         * and EVERYTHING AFTER THEM MUST BE 0x00 - the measured artefact and
         * nothing else. A ninth non-zero byte is another frame or corruption
         * and is still refused. */
        size_t ack_tail = 8u;
        while (ack_tail < rlen && rsp[ack_tail] == 0x00u) ack_tail++;
        if (rlen >= 8u && ack_tail == rlen &&
            rsp[0] == 0x03 && rsp[1] == 0x06 &&
            rsp[2] == 0x00 && rsp[3] == 0x60 &&
            rsp[4] == 0x00 && rsp[5] == 0x00) {
            const uint16_t ack_crc = modbus_crc16(rsp, 6);
            if ((uint8_t)(ack_crc & 0xFFu) == rsp[6] &&
                (uint8_t)(ack_crc >> 8) == rsp[7])
                return true;
            Serial.println("heater disarm: echo shape right, CRC WRONG - not "
                           "treating that as confirmation");
            continue;
        }
        delay(20);
    }
    return false;
}
#endif

void loop() {
    /* Advance the budget clock's origin. A watchdog reset, a brownout or a
     * panic gives no chance to do this on the way out, so doing it here bounds
     * what such a reset can forgive to one loop iteration instead of the whole
     * run. One RTC RAM write. */
    rtc_clock_checkpoint();

    /* USB PRESENCE, BEFORE ANYTHING THAT CAN BLOCK. serbus_tick() and the bus
     * poll below can each hold this loop for a second or more, and a sample
     * taken only after them would miss SOF that arrived while they ran. */
    usb_present_tick();

    /* THE USB CONSOLE, FIRST. A host tool blocked on a reply is the most
     * latency-sensitive thing this loop serves, and everything below it can
     * block for a second or more. It is also the path that has to be
     * serviced before maybe_sleep() gets a chance to look at the state it
     * changes. */
    serbus_tick();

    /* THE LINE-SETTING PROBATION'S DEADLINE. After serbus_tick(), because a
     * frame typed or forwarded in this same iteration is exactly the evidence
     * that should cancel the revert, and before anything that can block: a
     * revert that fires late has already spent the window it was bounding. */
#if PROTO_BUS
    bus_prob_tick();
#endif

    /* THE BLE RECEIVER'S CONSUMER HALF, on the app task and nowhere else.
     * esp_ota_begin/write/end block on flash for milliseconds; running them on
     * the NimBLE host task stalls it long enough to trip the connection
     * supervision timeout and drop the link mid-update. Also enforces the
     * stall and hard-ceiling deadlines, so it has to run even when nothing is
     * connected. */
    ble_ota_tick();

    /* STAND OFF WHILE FLASH IS BUSY. A flash erase disables the CPU cache and
     * stalls the other core - long enough to blow a Modbus inter-character
     * timeout, and long enough to matter to a LoRa transmission. The quiesce
     * hook is asserted for the whole transfer, not per write, so this parks
     * the measurement cadence for the ~50 s a push takes. That is the right
     * trade at a 300 s interval: one late point against a corrupted exchange
     * or a failed push.
     *
     * Returning here also skips maybe_sleep(), which is required and not
     * incidental - a deep sleep mid-erase is a half-written OTA slot. */
    if (ble_ota_flash_busy()) { delay(2); return; }

    /* CLASS C: a downlink can arrive at any moment, not only in the window
     * after our own uplink. Poll before anything else and keep the poll cheap,
     * because an unread Class C downlink is overwritten by the next one --
     * RadioLib's own Class C example warns about exactly this. */
    if (class_c_active) {
        uint8_t dl[MAX_FRAME];
        size_t  dllen = sizeof(dl);
        LoRaWANEvent_t ev = {};
        int st = node.getDownlinkClassC(dl, &dllen, &ev);
        /* Do NOT trust dllen. getDownlinkClassC has the same trap as
         * sendReceive: it leaves lenDown untouched when there is no
         * application payload, so `dllen` is still our own sizeof(dl) and a
         * `dllen > 0` test is always true. MEASURED on the first Class C boot:
         * "classC dl len=256 fPort=0" -- 256 being the initial value, and
         * fPort 0 being a MAC-only downlink (ChirpStack sends RXParamSetupReq
         * there immediately after an rx2 change). Acting on that would have
         * relayed 256 bytes of stack garbage onto the Modbus bus.
         *
         * fPort is the discriminator, exactly as it is for the dispatch: a
         * frame is ours only if it carries one of our two ports. */
        bool ours = (ev.fPort == O2P_FPORT || ev.fPort == O2P_SU_FPORT ||
                     ev.fPort == O2P_CTRL_FPORT);
        /* st > 0, NOT st == RADIOLIB_ERR_NONE. getDownlinkClassC returns the
         * RX window number on success, exactly like sendReceive -- and
         * RADIOLIB_ERR_NONE is 0, which here means NOTHING ARRIVED. Testing
         * for it inverts the condition: the body then runs only when there is
         * no downlink, and every real one is dropped. That is precisely the
         * semantics section 2 records for sendReceive; it was applied
         * backwards here and cost three failed transfer attempts, which
         * looked like a link-budget problem because the gateway logs showed
         * clean transmissions and the node showed nothing. */
        if (st > 0 && ours && dllen > 0 && dllen <= MAX_FRAME) {
            Serial.printf("classC dl len=%u fPort=%u\n",
                          (unsigned)dllen, (unsigned)ev.fPort);
            handle_downlink(dl, dllen, ev.fPort);
        }
    }

    /* Uplink when there is something to say, or on the measurement interval.
     * A queued reply is worth an early uplink. The 1 % duty cycle applies -
     * but note this used to say "RadioLib enforces the dwell between
     * transmissions", which was FALSE: dutyCycleEnabled defaults to false and
     * setDutyCycle() was never called, so nothing enforced anything. The join
     * path now enables it, which is what makes this sentence true. */
    /* Close an abandoned transfer before deciding anything else: while one is
     * open the node skips the bus and never sleeps, so a session that outlived
     * its host would hold both hostage indefinitely. */
    /* THE CADENCE LEASE, drained and expired on the app task.
     *
     * Both halves are here rather than anywhere earlier because RTC memory is
     * this task's to write: the BLE verb runs on the NimBLE host task and only
     * latches an intent (see ble_ota_take_cadence_request). */
#if PROTO_BLE_OTA
    /* Disconnect disarm: if a BLE peer dropped, immediately issue Modbus FC06
     * write to slave 3, reg 0x0060 (HEATER_TEST_ARM) = 0x0000 before releasing. */
    if (ble_ota_take_disconnect_disarm()) {
#if PROTO_BUS
        if (o2p_disarm_heater_sync()) {
            /* AND RECORD IT, so the transport-agnostic lease below does not
             * repeat a write that has just been acknowledged. This is the one
             * place a disarm is already known to have succeeded. */
            ctrl_arm_confirmed_off(&ctrl);
            Serial.println("ble: peer disconnected - heater confirmed disarmed on probe");
        } else {
            ctrl_arm_disarm_failed(&ctrl, rtc_uptime_s());
            Serial.println("!! ble: peer disconnected - heater disarm attempt unacknowledged");
        }
#endif
    }

    if (ble_ota_take_release_request()) {
#if PROTO_BUS
        o2p_disarm_heater_sync();
#endif
        ctrl_ble_release(&ctrl, rtc_uptime_s());
        ctrl_window_persist();
        Serial.println("ble: bus claim released (heater disarmed)");
    }

    {
        uint16_t mins = 0;
        if (ble_ota_take_claim_request(&mins)) {
            bool ok = ctrl_ble_claim(&ctrl, (uint8_t)mins, rtc_uptime_s());
            ctrl_window_persist();
            ble_ota_report_claim(mins, ok);
            if (ok) {
                Serial.printf("ble: bus claimed for %u min\n", (unsigned)mins);
                hold_awake((uint32_t)mins * 60000UL);
            } else {
                Serial.println("ble: bus claim REFUSED");
            }
        }
    }

    /* Offline field wake: pressing the BLE-window button (GP13 to 3V3,
     * config.h) while running brings up BLE */
    if (digitalRead(PIN_BOOT_BUTTON) == BOOT_BUTTON_ACTIVE && !ble_ota_is_up()) {
        delay(50);
        if (digitalRead(PIN_BOOT_BUTTON) == BOOT_BUTTON_ACTIVE &&
            !ble_ota_is_up()) {
            Serial.println("BLE button (GP13) pressed: bringing up offline field BLE window");
            ble_ota_up(PROTO_BLE_BENCH_NONCE, BLE_OTA_MAX_UP_MS);
            hold_awake(BLE_OTA_MAX_UP_MS);
        }
    }

    {
        uint16_t want_s = 0, want_lease = 0;
        if (ble_ota_take_cadence_request(&want_s, &want_lease)) {
            uint16_t got_s = 0, got_lease = 0;
            uint8_t  cr = cadence_clamp(want_s, want_lease, &got_s, &got_lease);
            /* SAME CLAMP AS THE LoRaWAN PATH, deliberately - the BLE surface
             * exists to remove a round trip, not to grant a wider range. */
            cadence_commit(rtc_cadence(), got_s, got_lease, rtc_uptime_s());
            ble_ota_report_cadence(got_s, got_lease, cr == CADENCE_CLAMPED);
            if (got_s)
                Serial.printf("cadence: %u s for %u min (BLE)\n",
                              (unsigned)got_s, (unsigned)got_lease);
            else
                Serial.println("cadence: back to the compiled default (BLE)");
        }
    }
#endif
    if (cadence_expire(rtc_cadence(), rtc_uptime_s()))
        Serial.println("cadence: lease expired - back to the compiled default");

    /* BEFORE ctrl_expire(), so that an arming which lapsed or was turned off
     * this iteration is already gone when the teardown asks whether a stream
     * is holding the radio up. */
    telemetry_control();

    bool was_ble_claim = ctrl.ble_claim;
    if (ctrl_expire(&ctrl, rtc_uptime_s())) {
        ctrl_window_persist();
        if (was_ble_claim) {
#if PROTO_BUS
            o2p_disarm_heater_sync();
#endif
            Serial.println("BLE bus claim expired - heater disarmed");
        }
        Serial.println("CTRL session expired - bus released, sleeping again");
        /* THE GRACE WINDOW BELONGS TO ble_ota_tick(), SO DO NOT PRE-EMPT IT.
         *
         * The reasoning here used to be that the BLE ceiling (15 min) is far
         * shorter than the CTRL one (120), so by the time a session expired
         * the radio would already be down and this would be a no-op. That
         * holds only when the host asks for more than 15 minutes - and
         * tools/ota_ble_push.py asks for exactly 15 by default, which puts
         * both deadlines in the SAME loop iteration. ble_ota_tick() would
         * grant its grace to a transfer seconds from finishing, and then this
         * line, running later in the same pass, would abort it anyway.
         *
         * A live transfer is left to its own bounded grace, and to
         * BLE_OTA_MAX_TOTAL_MS above that. Those are what bound it; this is
         * not. */
        /* A BLE BUS PUSH COUNTS TOO, and for a worse reason than the node's
         * own image does. An interrupted node OTA leaves a passive partition
         * half written, which the bootloader ignores. An interrupted o2-probe
         * push leaves the PROBE's staging slot half written mid-FC16, and the
         * probe is potted at the end of a cable this node is the only route
         * to - so the recovery for cutting it is another whole session, not a
         * reboot. Same grace, same reason, one more predicate. */
        /* ...AND AN OPERATOR MID-CONVERSATION IS THE THIRD PREDICATE.
         *
         * Review finding 2026-09-08: the connection grace added in
         * ble_ota_tick() was granted and then revoked HERE, later in the same
         * loop iteration, because neither ble_ota_busy() nor ble_bus_active()
         * is true for someone issuing CTRL commands. The feature never worked.
         * ble_ota_in_conn_grace() is the one predicate both sites now share,
         * so they cannot disagree again; it is bounded, activity-gated, and
         * outranked by the cumulative ceiling inside ble_ota_tick(). */
        if (ble_ota_busy() || ble_bus_active())
            Serial.println("  ...BLE transfer still running; its own grace "
                           "window decides when the radio goes down");
        else if (ble_ota_in_conn_grace())
            Serial.println("  ...an authenticated peer is still working the "
                           "link; its bounded grace decides when it goes down");
        /* AND A LIVE FAST-TELEMETRY STREAM IS THE FOURTH PREDICATE, added for
         * the reason the third one was: a grace granted inside ble_ota_tick()
         * and revoked here, later in the same loop iteration, is a feature
         * that never works. ble_telem_active() is the one predicate both sites
         * consult. It is stricter than the connection grace - the peer must be
         * subscribed and the arming lease still running - and it is outranked
         * by BLE_TELEM_MAX_TOTAL_MS inside ble_ota_tick(), which is what stops
         * a stream holding the radio up indefinitely. */
        else if (ble_telem_active())
            Serial.println("  ...a fast telemetry stream is running; its lease "
                           "and the cumulative BLE ceiling decide when it "
                           "goes down");
        else
            ble_ota_down("CTRL session expired");
    }

    if (su_idle_timeout(SU_IDLE_TIMEOUT_MS))
        Serial.println("!! self-update session timed out - staged image "
                       "discarded, measuring again");

    /* DROP THE CLAIM CACHE THE MOMENT THE CLAIM ENDS.
     *
     * One falling edge covers every way a claim can end - unclaim, peer
     * disconnect, ctrl_expire(), a SESSION_CLOSE, a cable claim outranking the
     * BLE one - rather than a poll_cache_clear() at each of those sites, which
     * is the arrangement that leaves one of them out. Placed after all of
     * them and before telemetry_emit(), so no frame in this iteration can be
     * built from a cache the release has already invalidated.
     *
     * The age bound would also retire these records, but only after the fact:
     * without this, a claim taken two minutes after the previous one ended
     * could serve a reading from the earlier session as if the new claim
     * holder had just taken it. Belt and braces, and the braces are the ones
     * that hold when the cadence is long. */
    {
        static bool claim_was_held = false;
        if (claim_was_held && !ctrl.bus_claimed) {
            poll_cache_clear(&claim_cache);
            Serial.println("claim cache: cleared - the bus claim ended");
        }
        claim_was_held = ctrl.bus_claimed;
    }

    /* THE ARM LEASE, ENFORCED FOR EVERY TRANSPORT - the second half of the
     * hook in bus_write(), and the answer to "an armed heater override has no
     * communication lease at all".
     *
     * AFTER ctrl_expire(), so that a claim which lapses on this pass has
     * already been released when the guard looks: the arm's owner is then
     * visibly gone and the disarm goes out in the same iteration rather than
     * the next one. (It is correct either way - one loop pass later at worst -
     * but "the owner is gone" is the condition, and this is where it becomes
     * true.)
     *
     * A FAILED DISARM IS NOT A DISARM. Only a probe ACK reaches
     * ctrl_arm_confirmed_off(); an unacknowledged attempt keeps the arm
     * recorded as held (or as never proven, after a reset) and comes back at
     * the retry interval, forever, at a slower rate after CTRL_ARM_TRIES. The
     * node never concludes "clean" from a probe that did not answer.
     *
     * This runs on every loop pass and costs, in the common case, one compare
     * of two u32s and a branch - the state is CTRL_ARM_OFF and it returns
     * immediately. */
#if PROTO_BUS
    /* The arm's announcement, moved off the bus_write() path - see there. One
     * byte of state, printed on the transition only. */
    {
        static uint8_t arm_said = CTRL_ARM_UNPROVEN;
        if (ctrl.arm_state != arm_said) {
            arm_said = ctrl.arm_state;
            if (arm_said == CTRL_ARM_HELD)
                Serial.printf("HEATER ARM seen on the bus (owner %u) - leased "
                              "for %lu s of owner silence, and for as long as "
                              "that owner holds the pair\n",
                              (unsigned)ctrl.arm_owner,
                              (unsigned long)CTRL_ARM_LEASE_S);
        }
    }
    /* ctrl_arm_unproven() FIRST, and only for cost: it is one byte compare,
     * and in the steady state (the arm proven off) it keeps rtc_uptime_s()'s
     * fold-and-divide out of every loop pass. It cannot mask the guard - the
     * guard's own first line returns false on exactly the same condition. */
    if (ctrl_arm_unproven(&ctrl) && ctrl_arm_needs_disarm(&ctrl, rtc_uptime_s())) {
        if (o2p_disarm_heater_sync()) {
            ctrl_arm_confirmed_off(&ctrl);
            Serial.println("heater arm lease lapsed - confirmed disarmed on the probe");
        } else {
            ctrl_arm_disarm_failed(&ctrl, rtc_uptime_s());
            Serial.println("!! heater arm lease lapsed and the probe did NOT "
                           "acknowledge the disarm - state stays UNPROVEN, "
                           "retrying");
        }
    }
#endif

    /* THE FAST PATH, on its OWN interval and independent of everything below.
     *
     * Placed before the uplink decision so a BLE frame is not held up by a
     * LoRaWAN cycle - and after the teardown decisions above so it never
     * drives the bus for a link that has just been taken down. It costs
     * nothing at all when no stream is armed. */
    telemetry_emit();

    bool have_reply = rq_count > 0;
    bool poll_due   = (millis() - last_poll) >= poll_interval_ms();

    /* A BLE OTA IN FLIGHT OWNS THIS LOOP, AND THE BUS POLL MUST STAND ASIDE.
     *
     * MEASURED 2026-09-10, one complete push of 751 744 B from macOS: 108.58 s
     * end to end = 6.76 kB/s, while the middle half of the SAME transfer moved
     * 375 808 B in 14.34 s = 25.59 kB/s. Device accounting for that push:
     * write_ms=7587, erase_ms=5531 - 13.1 s of flash work in a 108 s transfer.
     * So ~75 % of the wall clock was neither link nor flash.
     *
     * WHERE IT WENT. otaBleTick() is the CONSUMER: it drains the 8 KB staging
     * ring to flash, and grantCredit() advances the host's window to
     * `written + RING_CAP` as it drains. It runs from this loop. So does the
     * bus poll - and read_frame() blocks for up to REPLY_WAIT_MS (1500 ms),
     * with bus_quarantine_until costing a second REPLY_WAIT_MS after a silent
     * exchange. A poll landing during a push freezes the consumer for seconds;
     * the ring stops draining, credit stops advancing, and the host sits in
     * its credit wait. That is why the rate is fine in the middle of a
     * transfer and poor across the whole of one.
     *
     * WHY ble_ota_busy() AND NOT ble_ota_is_up(). is_up() is true for the
     * whole 15-minute BLE window, which a cabled node OPENS AT BOOT - gating
     * on it would stop telemetry for fifteen minutes at a time, a far worse
     * bug than the one being fixed. ble_ota_busy() is `up && otaBleActive()`,
     * true only between a successful begin and its matching end/abort.
     *
     * DELIBERATELY NOT STAMPING last_poll: the poll is postponed, not
     * consumed, so the measurement happens as soon as the push finishes
     * instead of a further full interval later. otaBleTick()'s own 30 s
     * no-bytes abort bounds how long this can hold the bus. */
    if (ble_ota_busy()) {
        static uint32_t ota_hold_said_at = 0;
        if (poll_due && millis() - ota_hold_said_at > 5000u) {
            ota_hold_said_at = millis();
            Serial.println("poll held: a BLE OTA is in flight - the bus stands "
                           "aside so the flash consumer keeps draining");
        }
        poll_due = false;
    }
    /* AND THE BAND HAS TO BE READY, ASKED ON THE GATE'S OWN CLOCK.
     *
     * The cadence floor alone does NOT prevent the deletion it was added for,
     * and this is why: last_poll is stamped BEFORE the transmission while the
     * persisted duty timestamp is stamped AFTER sendReceive() and its receive
     * windows, so the two clocks are offset by however long that took. The
     * floor covers the AIRTIME and not the offset. Worked example from the
     * review that found it: floor 60 s, last_tx_s = 3, next attempt at second
     * 60 - only 57 s elapsed against 60 required, poll built, frame refused,
     * MEASUREMENT DESTROYED, and identical good diagnostic frames therefore
     * still produce ~120 s spacing at DR3 on an awake node with no ADR or
     * FOpts change at all. Predicting the offset would be guessing at RX
     * window durations; asking duty_may_send() the question with the same
     * inputs the gate itself uses cannot be off by construction.
     *
     * DEFERRED, NOT DELETED, and that is the whole point. Nothing is built, so
     * the probe is not polled and no measurement exists to lose; maybe_sleep()
     * declines to sleep because the interval HAS elapsed, so the loop stays
     * awake and sends as soon as the band allows - roughly 62 s spacing rather
     * than 120. */
    duty_defer_ms = 0;
    if (poll_due) {
        uint32_t pd_tx_s = 0, pd_toa_ms = 0, pd_owed_s = 0;
        rtc_duty_load(&pd_tx_s, &pd_toa_ms);
        if (!duty_may_send(rtc_uptime_s(), pd_tx_s, pd_toa_ms, &pd_owed_s)) {
            poll_due = false;
            duty_defer_ms = pd_owed_s * 1000UL;
            /* Once per wait, not once per loop pass - this branch runs every
             * 50 ms and a console that repeats itself twenty times a second
             * is a console nobody reads. */
            static uint32_t pd_said_at = 0;
            if (millis() - pd_said_at > 5000u) {
                pd_said_at = millis();
                Serial.printf("poll deferred %lu s: the band owes silence "
                              "after a %lu ms frame - not building one to "
                              "throw away\n",
                              (unsigned long)pd_owed_s,
                              (unsigned long)pd_toa_ms);
            }
        }
    }
    if (!have_reply && !poll_due) {
        /* Nothing to send and the next measurement is not due. THIS is the
         * branch a sleeping node spends its life in, so the sleep decision has
         * to be made here and not only after an uplink - a maybe_sleep() that
         * ran only on the uplink path would leave the node spinning through
         * this delay for the whole interval, awake, at full radio-idle
         * current. It returns without sleeping while a listening window is
         * open or a transfer is in flight. */
        maybe_sleep();
        delay(class_c_active ? 5 : 50);
        return;
    }
    last_uplink = millis();

    Reply    rep;
    uint8_t  pollbuf[MAX_FRAME];
    bool     sending_reply = reply_pop(&rep);
    const uint8_t *ul      = nullptr;
    size_t   ullen         = 0;
    uint8_t  ul_port       = 0;

    if (sending_reply) {
        /* Only a real pending reply wears the port its request arrived on.
         * MEASURED 2026-09-06: when that port was sticky, the node kept
         * uplinking on fPort 11 afterwards and the host matched a heartbeat as
         * a status frame. */
        ul      = rep.buf;
        ullen   = rep.len;
        ul_port = rep.port;
    } else {
        /* A SCHEDULED MEASUREMENT, not a heartbeat.
         *
         * The 1-byte 0x00 this used to send existed only to open a Class A
         * downlink window. It is gone: an uplink that carries a reading opens
         * the same window and costs the same wake.
         *
         * WHILE A TRANSFER IS OPEN: send the frame, skip the bus.
         *
         * An earlier version returned here without transmitting at all, on the
         * reasoning that a lost block costs a resync of the whole transfer
         * while a lost poll costs five minutes of one series. That reasoning
         * had a hole: in Class A a downlink arrives ONLY in the window opened
         * by our own uplink, so suppressing uplinks also suppresses the ABORT
         * that would end a stuck transfer. The node sealed itself in, and the
         * only symptom visible from the ground was that it had gone quiet.
         *
         * What IS worth skipping is the bus poll: read_frame() blocks for up
         * to REPLY_WAIT_MS per device, and RadioLib overwrites an unread Class
         * C downlink with the next one, so blocking during a burst really does
         * drop blocks. Skipping it costs one record.
         *
         * So: a header-only frame with POLL_FLAG_SU_BUSY set. The radio window
         * opens, the node stays visible, and the absent measurement is
         * explained rather than merely missing. */
        bool su_open  = su_busy();
        /* A claimed bus is not ours to touch. The o2-probe's OWN firmware
         * update travels as ordinary fPort 10 relay frames, and the node
         * cannot see that a run of individually complete Modbus exchanges is
         * one transaction - so without an explicit claim a scheduled poll
         * would be inserted between two updater frames. The claim is what
         * makes that transaction visible to this firmware. */
        /* A CLAIMED bus is not ours; a bus that is OFF has nothing to ask.
         * In a mock build neither applies to the source of the numbers - the
         * probe is in this binary, not on a wire - so PROTO_BUS_OFF stops
         * gating the poll. The CLAIM still does: CTRL_F_CLAIM_BUS is the
         * host saying "do not talk right now", and a mock that kept
         * chattering through a session the operator opened would be testing
         * something other than the firmware that will ship.
         *
         * POLL_FLAG_BUS_OFF is still set below, and it is still true: this
         * build genuinely does not drive the RS-485 pads, which is what makes
         * it safe to run while the bench CP2102 lead is on them. What tells
         * the server the records are synthetic is the payver, not this flag. */
        /* PROTO_BUS-GUARDED. bus_direction_ok is only ever ASSIGNED inside
         * #if PROTO_BUS, so in a bus-off build it keeps its false initialiser for
         * the life of the image - and an unguarded disjunct is then permanently
         * true, which silenced env:esp32s3_mock: header-only frames, no synthetic
         * records, and the whole server chain that build exists to exercise fed
         * nothing, quietly and on schedule. The (!PROTO_BUS && !PROTO_MOCK) term
         * right here exists to stop exactly that, and the new one stepped over it.
         * PROTO_BUS is 0/1, so this folds away. */
        bool bus_held = ctrl.bus_claimed || (!PROTO_BUS && !PROTO_MOCK) ||
                        (PROTO_BUS && !bus_direction_ok);
        /* Built by poll_flags_now(), shared with the BLE telemetry path so the
         * two copies of one measurement cannot describe different node
         * states. */
        uint8_t flags = poll_flags_now(su_open);
        /* A fresh nonce on every uplink: this frame is the invitation, and
         * binding a SESSION_OPEN to it is what stops a queue item left over
         * from an hour ago opening a session nobody wants. */
        uplink_nonce = rtc_next_nonce();
#if PROTO_MOCK
        /* The mock's only input is time, and it must be the SAME clock the
         * frame's uptime field uses, or the settle countdown on the dashboard
         * would not match the uptime beside it. */
        mock_probe_set_time(rtc_uptime_s());
#endif
        /* Cap the frame at what THIS data rate can actually carry.
         *
         * The two o2-probe blocks are 96 bytes, which fits DR3 and above. ADR
         * is enabled and EU868 permits DR0-DR2, where the application limit is
         * 51 bytes - and sendReceive() REFUSES an oversized payload outright,
         * so the node would simply stop uplinking at the moment the link got
         * bad enough to lower the rate. Losing the diagnostic block is a
         * degraded reading; losing the uplink is a silent node.
         *
         * getMaxPayloadLen() already subtracts the FHDR and the current FOpts,
         * so it is the real ceiling and not a nominal one. poll_build_frame()
         * omits whole records that do not fit and never truncates one. */
        size_t cap = sizeof(pollbuf);
        uint8_t radio_cap = node.getMaxPayloadLen();
        if (radio_cap > 0u && (size_t)radio_cap < cap) {
            cap = (size_t)radio_cap;
        }
        ullen = poll_build_frame(poll_source(su_open, bus_held),
                                 flags, rtc_uptime_s(), uplink_nonce, pollbuf,
                                 cap);
        if (ullen > 0 && cap < 86u) {
            Serial.printf("!! DR limits the uplink to %u bytes; sent %u "
                          "(diagnostic block dropped)\n",
                          (unsigned)cap, (unsigned)ullen);
        }
        if (ullen == 0) {
            /* Cannot happen with the compiled-in table and a 256-byte buffer,
             * but an empty uplink would decode as a codec failure downstream
             * and be blamed on the radio. Say so here instead. */
            Serial.println("!! poll_build_frame produced nothing - not sending");
            return;
        }
        ul      = pollbuf;
        ul_port = O2P_SENSOR_FPORT;
        /* Stamped here, where a measurement was actually taken - not on every
         * uplink. See last_poll's declaration. */
        last_poll = millis();
    }

    /* DUTY CYCLE ACROSS SLEEP. RadioLib cannot do this for us: its lastToA
     * and tUplinkEnd are reconstructed at zero on every wake and are not in
     * the session buffer, so its own guard admits the first uplink after each
     * wake unconditionally - and setup() makes that uplink due immediately.
     * With a fast cadence that is one full-airtime transmission per boot with
     * no silence between them at all.
     *
     * A 1 % duty cycle means the period must be at least 100x the airtime.
     * Rounded UP to whole seconds so the coarse clock always errs toward more
     * silence, never less. */
    {
        uint32_t last_tx_s = 0, last_toa_ms = 0;
        rtc_duty_load(&last_tx_s, &last_toa_ms);
        uint32_t owed_s = 0;
        if (!duty_may_send(rtc_uptime_s(), last_tx_s, last_toa_ms, &owed_s)) {
            if (sending_reply) reply_unpop(&rep);
            /* A REFUSAL HERE DESTROYS A MEASUREMENT, so it is counted rather
             * than merely logged to a console nobody is attached to.
             *
             * The poll_due duty precondition above should make this
             * unreachable for POLL frames, because nothing is built while the
             * band owes silence. A nonzero count therefore means the airtime
             * grew between that check and this one - a LinkADRReq's FOpts or
             * an ADR step down inside the same pass - or that this frame is a
             * queued CTRL reply rather than a poll. An earlier version of this
             * comment claimed the floor alone made it unreachable; it did not,
             * because the floor covers airtime and not the offset between
             * last_poll and the duty timestamp.
             *
             * NOT RETRIED FROM A BUFFER, and the reason is not oversight: the
             * frame is built by poll_build_frame(), which DRIVES THE BUS for
             * every POLL_CMD. Rolling last_poll back to force a rebuild
             * re-polls the probe on every loop iteration until the gate opens -
             * up to a minute of that - and re-sending the stored frame instead
             * would re-send its uptime and its session nonce, which poll.h
             * calls "the node's invitation to open a session". Retrying
             * properly needs poll_build_frame() split into a collect step and a
             * frame step so the header can be rebuilt around cached records.
             * That is a real change and it is NOT done. */
            if (ul == pollbuf) duty_deleted_polls++;
            Serial.printf("uplink held %lu s: the band owes %lu s of "
                          "silence after a %lu ms transmission%s\n",
                          (unsigned long)owed_s,
                          (unsigned long)duty_need_s(last_toa_ms),
                          (unsigned long)last_toa_ms,
                          sending_reply ? " (reply re-queued)" : "");
            return;
        }
    }

    /* SNAPSHOT THE FRAME'S COST BEFORE TRANSMITTING IT.
     *
     * These three values must describe the sequence that is about to go out,
     * and all three are stale the moment sendReceive() returns:
     *
     *   getMacUplinkLen() is the live fOptsUpLen. RadioLib builds the frame
     *   with the value it has NOW (LoRaWAN.cpp:141), then a downlink queues
     *   MAC responses for the NEXT one - so one DevStatusReq makes the
     *   after-the-fact reading three bytes too long.
     *
     *   The data rate can move inside the call: with no downlink RadioLib runs
     *   adrBackoff() before returning (LoRaWAN.cpp:273-280), so a DR5 frame is
     *   costed at DR4 afterwards - 328 ms for something that took 184 ms.
     *
     *   nbTrans can be changed by a LinkADRReq in the downlink we just
     *   received.
     *
     * Reading them afterwards made the exact-equality test fail on ORDINARY
     * traffic and charged roughly double, which on a 30 s cadence suppresses
     * the next measurement while last_poll has already advanced - the node
     * goes quiet for a minute at a time, from a guard meant to keep it legal.
     *
     * The rate is safe to read here: selectChannels() saves the uplink DR and
     * restores it (LoRaWAN.cpp:3576, 3675), so the rate that transmits is the
     * rate standing now. */
    const uint8_t  duty_phy_len = (uint8_t)(ullen + LORAWAN_PHY_OVERHEAD +
                                            node.getMacUplinkLen());
    const uint32_t duty_own_ms  = node.uplinkToaMs(duty_phy_len);
    /* Feed the DR-aware cadence floor from the airtime of the frame that is
     * actually going out, at the rate that is actually in force. This is the
     * only number here that is true: see duty_floor_note(). */
    duty_floor_note(duty_own_ms);
    const uint8_t  duty_repeats = node.uplinkRepeats();

    /* RESERVE THE AIRTIME BEFORE TRANSMITTING, RECONCILE AFTER.
     *
     * sendReceive() transmits and then blocks through RX1 and RX2 - well over
     * a second at DR3 - and only returns afterwards. Charging on return meant a
     * watchdog, brownout or panic anywhere in that window discarded the airtime
     * of a frame that had ALREADY gone out, and the reset handed the band a
     * refund. Computing the cost at the right moment does not help if the
     * durable write happens at the wrong one.
     *
     * So the worst case for this attempt is written NOW, and corrected to the
     * measured value when the call returns. A reset in between leaves the
     * reservation standing, which errs toward silence. If nothing actually
     * flew, the reconcile restores the previous obligation instead. */
    uint32_t duty_prev_tx_s = 0, duty_prev_toa_ms = 0;
    rtc_duty_load(&duty_prev_tx_s, &duty_prev_toa_ms);
    (void)radio.take_air_ms(NULL);          /* start this call's count at zero */
    rtc_duty_store(rtc_uptime_s(),
                   duty_own_ms ? duty_own_ms * duty_repeats
                               : DUTY_UNKNOWN_TOA_MS);

    uint8_t  dl[MAX_FRAME];
    size_t   dllen = sizeof(dl);
    LoRaWANEvent_t ev_down = {};
    int st = node.sendReceive((uint8_t *)ul, ullen, ul_port, dl, &dllen,
                              false, NULL, &ev_down);
    if (st == RADIOLIB_ERR_UPLINK_UNAVAILABLE) {
        /* NOT an error: the duty-cycle budget for this sub-band is spent.
         * Named separately because "skipped, by law" and "the radio is
         * broken" are indistinguishable in a log otherwise, and they need
         * completely different responses from whoever reads it.
         *
         * A REPLY MUST GO BACK ON THE QUEUE. Its request's effects are already
         * committed; dropping the answer strands the host. A scheduled
         * measurement is different - it is regenerated next cycle, and
         * re-queueing it would replay a stale reading. */
        if (sending_reply) {
            reply_unpop(&rep);
            Serial.printf("reply on fPort %u held: duty-cycle budget spent, "
                          "it goes out at the next opportunity\n",
                          (unsigned)ul_port);
        } else {
            Serial.println("uplink skipped: 1 % duty-cycle budget spent");
        }
        return;
    }
    /* AN ERROR DOES NOT MEAN NOTHING WAS TRANSMITTED. RadioLib says so
     * explicitly and advances fCntUp BEFORE returning the error
     * (LoRaWAN.cpp:237), so a failure after the RF went out still consumed
     * both airtime and a frame counter. Recording the duty stamp and the
     * session only on success meant the next wake restored a stale timestamp
     * and a used counter: the node could transmit before its silence was up
     * and replay an fCnt the server has already seen - which reads from the
     * ground as coverage loss, not as a bug here.
     *
     * UPLINK_UNAVAILABLE is the one case that genuinely transmitted nothing,
     * and it returned above without reaching this. Everything else is
     * recorded, error or not. */
    /* RECONCILE THE RESERVATION AGAINST WHAT ACTUALLY FLEW.
     *
     * The radio counted every frame it was told to launch, so this is a
     * MEASUREMENT rather than an inference. It covers the cases no arithmetic
     * on getLastToA() could: a repeat sequence that stopped early because a
     * downlink arrived (nbTrans is a maximum, not a count), the MAC-only uplink
     * RadioLib sends with its own duty guard lifted, a frame whose completion
     * IRQ was missed, and - the one that defeated the previous rule outright -
     * a second frame that happens to have exactly the same airtime as ours.
     *
     * getLastToA() is kept only as a floor: if RadioLib ever reports more than
     * we counted, the larger number is the safe one to charge. */
    {
        uint32_t tx_count = 0;
        const uint32_t counted  = radio.take_air_ms(&tx_count);
        const uint32_t reported = (uint32_t)node.getLastToA();
        const uint32_t charge   = counted > reported ? counted : reported;

        if (charge == 0) {
            /* Nothing reached the radio: PACKET_TOO_LONG and every other
             * refusal decided before RF. Release the reservation and restore
             * the obligation that was standing before it, which is still the
             * real one - the previous transmission's silence has not been
             * served merely because this attempt was refused. */
            rtc_duty_store(duty_prev_tx_s, duty_prev_toa_ms);
        } else {
            rtc_duty_store(rtc_uptime_s(), charge);
            if (tx_count > duty_repeats)
                Serial.printf("duty: %lu frames left the radio inside one "
                              "sendReceive (%lu of RadioLib's own); "
                              "charging %lu ms\n",
                              (unsigned long)tx_count,
                              (unsigned long)(tx_count - duty_repeats),
                              (unsigned long)charge);
        }
    }
    rtc_save_session(node.getBufferSession(), RADIOLIB_LORAWAN_SESSION_BUF_SIZE);

    if (st < RADIOLIB_ERR_NONE) {
        /* PACKET_TOO_LONG is decided BEFORE the radio and can never succeed on
         * retry - the reply is longer than the live data rate allows - so it
         * is a permanently undeliverable answer, not a transient one. Say that
         * plainly rather than re-queueing it forever or logging it as a
         * generic radio fault.
         *
         * Every other negative result is AMBIGUOUS: the frame may or may not
         * have gone out. Restoring it could duplicate a reply the host already
         * has, which for a self-update status is worse than losing one, so a
         * reply is dropped here deliberately and the host resyncs on next_seq. */
        if (st == RADIOLIB_ERR_PACKET_TOO_LONG && sending_reply) {
            Serial.printf("!! reply on fPort %u DROPPED: %u bytes exceeds what "
                          "this data rate allows - it can never be sent\n",
                          (unsigned)ul_port, (unsigned)ullen);
        } else {
            Serial.printf("sendReceive error %d%s\n", st,
                          sending_reply ? " (reply dropped: it may already have "
                                          "been transmitted, and a duplicate is "
                                          "worse than a resync)" : "");
        }
        return;
    }

    Serial.printf("ul fPort=%u len=%u -> st=%d, dl len=%u fPort=%u\n",
                  (unsigned)ul_port, (unsigned)ullen, st,
                  (unsigned)dllen, (unsigned)ev_down.fPort);
    /* fCnt moved; persist so the next wake resumes instead of replaying.
     *
     * RTC only. It is the store the next wake actually reads, and it costs no
     * flash. There is no NVS mirror because a cold boot deliberately does not
     * use one: an NVS session's fCnt is always behind the server's, so the
     * node joins instead. See the note at the join. */
    rtc_save_session(node.getBufferSession(), RADIOLIB_LORAWAN_SESSION_BUF_SIZE);

    /* The one moment a sleeping node can be reached. See POST_UPLINK_LISTEN_MS
     * in config.h: RadioLib has already sat through RX1 and RX2 inside
     * sendReceive, so this covers the Class C case, where ChirpStack transmits
     * a queued downlink immediately and the timing depends on how fast the
     * server reacts to the uplink it has just received. */
    hold_awake(POST_UPLINK_LISTEN_MS);

    /* COMMIT selected the new slot but did not boot it -- see su_reboot_armed.
     * Reboot once the STAGED status has actually left the radio, so the host
     * knows the commit succeeded. The new image comes up in PENDING_VERIFY and
     * must earn su_confirm() by joining AND reaching the bus.
     *
     * ul_port MATTERS. This used to fire on ANY reply while armed, so a
     * queued relay reply that happened to go out first would reboot the node
     * before the STAGED status was ever sent - the host would see a node
     * vanish mid-exchange with no indication the commit had succeeded, which
     * is the exact ambiguity the deferred reboot exists to remove. */
    if (sending_reply && ul_port == O2P_SU_FPORT && su_reboot_armed()) {
        Serial.println("COMMIT acknowledged; rebooting into the new slot");
        /* Fold the elapsed time into the budget clock FIRST. RTC_DATA is wiped
         * by esp_restart(); the budgets are not, but their clock origin has to
         * be advanced here or the seconds since the last checkpoint would be
         * forgiven - and this restart happens 200 ms after a transmission. */
        rtc_clock_checkpoint();
        Serial.flush();
        delay(200);
        esp_restart();
    }

    if (st > 0 && dllen > 0) handle_downlink(dl, dllen, ev_down.fPort);

    /* ARMED BUT UNABLE TO SAY SO. If the STAGED status was popped from the
     * queue and then failed to transmit, it is gone: the host never learns the
     * commit succeeded, and the node is left armed - which blocks sleep
     * forever, because a scheduled sensor uplink is not a reply and so never
     * reaches the reboot above. A node that has staged a good image and then
     * quietly stops sleeping is the worst of both outcomes.
     *
     * So the reboot is bounded rather than conditional on an acknowledgement
     * that may never be sendable. The host discovers the new image from its
     * next status poll, which is a resync it already knows how to do. */
    if (su_reboot_armed()) {
        if (reboot_armed_at == 0) reboot_armed_at = millis();
        else if (millis() - reboot_armed_at > SU_REBOOT_GRACE_MS) {
            Serial.println("!! staged image armed but the STAGED status could "
                           "not be sent; rebooting anyway");
            Serial.flush();
            delay(200);
            esp_restart();
        }
    } else {
        reboot_armed_at = 0;
    }

    maybe_sleep();
}
