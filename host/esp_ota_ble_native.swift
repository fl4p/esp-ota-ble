// Native CoreBluetooth readiness queue. OTA byte credits remain owned by the host protocol.
import Foundation
import CoreBluetooth

final class Client: NSObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    let queue = DispatchQueue(label: "esp-ota-ble", qos: .userInteractive)
    let address: UUID
    let cmdID: CBUUID, notifyID: CBUUID, fwID: CBUUID
    var central: CBCentralManager!
    var peripheral: CBPeripheral?
    var cmd: CBCharacteristic?, notify: CBCharacteristic?, fw: CBCharacteristic?
    var jobs: [(Int, Data, Bool)] = []
    var head = 0, buffered = 0, writes = 0, callbacks = 0
    var pending: Int?
    var closed = false
    var connected = false
    var subscribing = false
    var activity: NSObjectProtocol?

    init(_ config: [String: String]) throws {
        guard let address = UUID(uuidString: config["address"] ?? ""),
              let cmd = config["cmd"], let notify = config["notify"], let fw = config["fw"] else {
            throw NSError(domain: "configuration", code: 1)
        }
        self.address = address
        cmdID = CBUUID(string: cmd); notifyID = CBUUID(string: notify); fwID = CBUUID(string: fw)
        super.init()
        activity = ProcessInfo.processInfo.beginActivity(options: [.userInitiated, .latencyCritical], reason: "BLE OTA")
        central = CBCentralManager(delegate: self, queue: queue)
        queue.asyncAfter(deadline: .now()+25) { if !self.connected { self.fail("connect deadline") } }
    }

    func emit(_ value: [String: Any]) {
        guard let data = try? JSONSerialization.data(withJSONObject: value, options: [.sortedKeys]) else { exit(2) }
        print(String(decoding: data, as: UTF8.self)); fflush(stdout)
    }
    func fail(_ message: String) {
        if closed { return }; closed = true
        emit(["event": "error", "message": message])
        if let p = peripheral { central.cancelPeripheralConnection(p) }
        queue.asyncAfter(deadline: .now()+0.2) { exit(1) }
    }
    func receive(_ input: String) {
        guard !closed else { return }
        guard let data = input.data(using: .utf8),
              let message = (try? JSONSerialization.jsonObject(with: data)) as? [String: Any],
              let op = message["op"] as? String else { fail("invalid IPC frame"); return }
        if op == "close" {
            closed = true
            if let p = peripheral { central.cancelPeripheralConnection(p) }
            queue.asyncAfter(deadline: .now()+0.2) { exit(0) }; return
        }
        guard connected, ["command", "data"].contains(op), let token = message["id"] as? Int,
              let encoded = message["data"] as? String, let bytes = Data(base64Encoded: encoded),
              !bytes.isEmpty, let p = peripheral else { fail("invalid write request"); return }
        let response = op == "command"
        guard bytes.count <= p.maximumWriteValueLength(for: response ? .withResponse : .withoutResponse),
              buffered + bytes.count <= 1024*1024 else { fail("write capacity/queue exceeded"); return }
        jobs.append((token, bytes, response)); buffered += bytes.count
        pump()
    }
    func pump() {
        guard !closed, pending == nil, let p = peripheral, let command = cmd, let firmware = fw else { return }
        while head < jobs.count {
            let (token, bytes, response) = jobs[head]
            if !response && !p.canSendWriteWithoutResponse { return }
            head += 1; buffered -= bytes.count
            if response { pending = token }
            p.writeValue(bytes, for: response ? command : firmware, type: response ? .withResponse : .withoutResponse)
            if !response { writes += 1 }
            if head == jobs.count { jobs.removeAll(keepingCapacity: true); head = 0 }
            else if head > 256 { jobs.removeFirst(head); head = 0 }
            if response { return }
        }
    }
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            guard let p = central.retrievePeripherals(withIdentifiers: [address]).first else {
                fail("device identifier not found; discover it first"); return
            }
            peripheral = p; p.delegate = self; central.connect(p)
        } else if central.state != .unknown && central.state != .resetting { fail("Bluetooth unavailable") }
    }
    func centralManager(_ central: CBCentralManager, didConnect p: CBPeripheral) { p.discoverServices(nil) }
    func centralManager(_ central: CBCentralManager, didFailToConnect p: CBPeripheral, error: Error?) { fail("connection failed") }
    func peripheral(_ p: CBPeripheral, didDiscoverServices error: Error?) {
        guard error == nil, let services = p.services, !services.isEmpty else { fail("service discovery"); return }
        for service in services { p.discoverCharacteristics([cmdID, notifyID, fwID], for: service) }
    }
    func peripheral(_ p: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        guard error == nil else { fail("characteristic discovery"); return }
        for c in service.characteristics ?? [] {
            if c.uuid == cmdID { cmd = c }
            if c.uuid == notifyID { notify = c }
            if c.uuid == fwID { fw = c }
        }
        if let c = cmd, let n = notify, let f = fw, !subscribing {
            guard c.properties.contains(.write), n.properties.contains(.notify),
                  f.properties.contains(.writeWithoutResponse) else { fail("characteristic properties"); return }
            subscribing = true; p.setNotifyValue(true, for: n)
        }
    }
    func peripheral(_ p: CBPeripheral, didUpdateNotificationStateFor c: CBCharacteristic, error: Error?) {
        guard error == nil, c.isNotifying else { fail("notification subscription"); return }
        if connected { return }; connected = true
        emit(["id": 0, "max_write": p.maximumWriteValueLength(for: .withoutResponse)])
    }
    func peripheral(_ p: CBPeripheral, didUpdateValueFor c: CBCharacteristic, error: Error?) {
        guard error == nil, let data = c.value else { fail("notification read"); return }
        emit(["event": "notify", "data": data.base64EncodedString()])
    }
    func peripheral(_ p: CBPeripheral, didWriteValueFor c: CBCharacteristic, error: Error?) {
        guard error == nil, let token = pending else { fail("command acknowledgement"); return }
        pending = nil; emit(["id": token]); pump()
    }
    func peripheralIsReady(toSendWriteWithoutResponse p: CBPeripheral) { callbacks += 1; pump() }
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral p: CBPeripheral, error: Error?) {
        emit(["event": "disconnected", "writes": writes, "ready_callbacks": callbacks])
        closed = true; exit(0)
    }
}

do {
    guard CommandLine.arguments.count == 2, let data = CommandLine.arguments[1].data(using: .utf8),
          let config = (try JSONSerialization.jsonObject(with: data)) as? [String: String] else { exit(2) }
    let client = try Client(config)
    DispatchQueue.global(qos: .userInitiated).async {
        // Bound IPC dispatch too: do not accumulate an unbounded list of
        // closures outside the checked firmware queue.
        while let line = readLine() { client.queue.sync { client.receive(line) } }
        client.queue.async { client.receive("{\"op\":\"close\"}") }
    }
    withExtendedLifetime(client) { RunLoop.main.run() }
} catch { exit(2) }
