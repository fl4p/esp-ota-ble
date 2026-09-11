// Private raw-OTA experiment: CoreBluetooth delegate feeds the queue directly.
// Every data write still checks canSendWriteWithoutResponse and receiver credit.
import Foundation
import CoreBluetooth
import CryptoKit

final class Push: NSObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    let service = CBUUID(string: "e8308d3d-c3b4-45ff-ba58-9c0fb99d0ecb")
    let controlID = CBUUID(string: "b0e0d1a4-7f52-4a3e-9c61-2d8f5b3ae741")
    let dataID = CBUUID(string: "b0e0d1a5-7f52-4a3e-9c61-2d8f5b3ae741")
    let bytes: Data
    let expectedBase: String
    let digest: String
    var central: CBCentralManager!
    var peripheral: CBPeripheral?
    var control: CBCharacteristic?
    var dataChar: CBCharacteristic?
    var pendingCommand: String?
    var stage = "scan"
    var receive = Data()
    var lines = [[Any]]()
    var before = [String: Any]()
    var started: UInt64 = 0
    var sent = 0
    var grant = 0
    var ready = false
    var endSent = false
    var ok = false
    var callbacks = 0
    var writes = 0
    var finished = false

    init(path: String, base: String) throws {
        bytes = try Data(contentsOf: URL(fileURLWithPath: path))
        expectedBase = base.lowercased()
        digest = SHA256.hash(data: bytes).map { String(format:"%02x", $0) }.joined()
        super.init()
        guard bytes.count > 0, expectedBase.count == 64 else {
            throw NSError(domain:"native_push",code:1)
        }
        central = CBCentralManager(delegate: self, queue: .main,
            options: [CBCentralManagerOptionShowPowerAlertKey:false])
        DispatchQueue.main.asyncAfter(deadline: .now()+180) { self.fail("global deadline") }
    }
    func elapsed() -> Double {
        started == 0 ? 0 : Double(DispatchTime.now().uptimeNanoseconds-started)/1e9
    }
    func fail(_ reason: String) {
        if finished { return }; finished=true
        print("NATIVE FAIL \(reason)"); fflush(stdout)
        if let p=peripheral { central.cancelPeripheralConnection(p) }
        DispatchQueue.main.asyncAfter(deadline:.now()+1) { exit(1) }
    }
    func command(_ text: String) {
        guard !finished, pendingCommand == nil, let p=peripheral, let c=control else {
            fail("command overlap or missing characteristic: \(text)");return
        }
        pendingCommand=text
        p.writeValue(Data((text+"\n").utf8),for:c,type:.withResponse)
    }
    func later(_ seconds: Double, _ body: @escaping () -> Void) {
        DispatchQueue.main.asyncAfter(deadline:.now()+seconds) { if !self.finished { body() } }
    }
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            central.scanForPeripherals(withServices:[service])
        } else if central.state != .unknown && central.state != .resetting {
            fail("Bluetooth unavailable: \(central.state.rawValue)")
        }
    }
    func centralManager(_ central: CBCentralManager, didDiscover p: CBPeripheral,
                        advertisementData: [String:Any], rssi RSSI: NSNumber) {
        let name=(advertisementData[CBAdvertisementDataLocalNameKey] as? String) ?? p.name ?? ""
        guard name == "farmnode-202A29", peripheral == nil else { return }
        peripheral=p;p.delegate=self;central.stopScan();central.connect(p)
        print("NATIVE DISCOVER rssi=\(RSSI)");fflush(stdout)
    }
    func centralManager(_ central: CBCentralManager, didConnect p: CBPeripheral) {
        p.discoverServices([service])
    }
    func centralManager(_ central: CBCentralManager, didFailToConnect p: CBPeripheral,error: Error?) {
        fail("connect: \(String(describing:error))")
    }
    func peripheral(_ p: CBPeripheral, didDiscoverServices error: Error?) {
        guard error == nil, let s=p.services?.first(where:{$0.uuid==service}) else {
            fail("service discovery");return
        }
        p.discoverCharacteristics([controlID,dataID],for:s)
    }
    func peripheral(_ p: CBPeripheral, didDiscoverCharacteristicsFor s: CBService,error: Error?) {
        guard error == nil else { fail("characteristic discovery");return }
        control=s.characteristics?.first(where:{$0.uuid==controlID})
        dataChar=s.characteristics?.first(where:{$0.uuid==dataID})
        guard let c=control, dataChar != nil,
              p.maximumWriteValueLength(for:.withoutResponse)>=495 else {
            fail("missing characteristic or insufficient write capacity");return
        }
        p.setNotifyValue(true,for:c)
    }
    func peripheral(_ p: CBPeripheral, didUpdateNotificationStateFor c: CBCharacteristic,error: Error?) {
        guard error == nil, c.isNotifying else { fail("subscribe failed");return }
        if stage == "scan" { stage="auth";command("auth BEEF") }
    }
    func peripheral(_ p: CBPeripheral, didWriteValueFor c: CBCharacteristic,error: Error?) {
        guard error == nil, let cmd=pendingCommand else { fail("command acknowledgement");return }
        pendingCommand=nil
        if cmd == "auth BEEF" { later(1.5) { self.stage="setup"; self.command("interval 8") } }
        else if cmd == "interval 8" { later(2) { self.command("link") } }
        else if cmd == "link" { later(2) { self.command("buffers") } }
        else if cmd == "buffers" { later(2) { self.stage="info";self.command("info") } }
        else if cmd == "info" { later(0.5) { self.begin() } }
        else if cmd.hasPrefix("begin ") { pump() }
    }
    func begin() {
        guard stage == "info", before["base"] as? String == expectedBase,
              before["run"] != nil, let slot=before["slot"] as? Int, slot>=bytes.count else {
            fail("receiver identity/slot unverified");return
        }
        stage="push"; started=DispatchTime.now().uptimeNanoseconds
        command("begin \(bytes.count) \(digest)")
    }
    func pump() {
        guard stage == "push", ready, !finished, let p=peripheral, let c=dataChar else { return }
        while sent < bytes.count && sent < grant && p.canSendWriteWithoutResponse {
            let count=min(495,min(bytes.count-sent,grant-sent))
            p.writeValue(bytes.subdata(in:sent..<(sent+count)),for:c,type:.withoutResponse)
            sent+=count;writes+=1
        }
    }
    func peripheralIsReady(toSendWriteWithoutResponse peripheral: CBPeripheral) {
        callbacks+=1;pump()
    }
    func peripheral(_ p: CBPeripheral, didUpdateValueFor c: CBCharacteristic,error: Error?) {
        guard error == nil, let value=c.value else { fail("notification error");return }
        receive.append(value)
        if receive.count>65536 { fail("unterminated reply");return }
        while let idx=receive.firstIndex(of:10) {
            let part=receive.prefix(upTo:idx);receive.removeSubrange(...idx)
            guard let line=String(data:part,encoding:.utf8) else { fail("invalid UTF-8 reply");return }
            handle(line.trimmingCharacters(in:.whitespacesAndNewlines))
        }
    }
    func handle(_ line: String) {
        lines.append([elapsed(),line])
        if !line.hasPrefix("OTAB CRED") && !line.hasPrefix("OTAB PROG") {
            print(String(format:"[%.3f] %@",elapsed(),line));fflush(stdout)
        }
        if line.hasPrefix("OTAB FAIL") { fail(line);return }
        let words=line.split(separator:" ").map(String.init)
        if stage == "info" {
            if words.count==3 && words[1]=="BASE" { before["base"]=words[2].lowercased() }
            if words.count>=4 && words[1]=="INFO" {
                for item in words.dropFirst(2) {
                    let parts=item.split(separator:"=",maxSplits:1).map(String.init)
                    if parts.count==2 {
                        if parts[0]=="run" { before["run"]=parts[1] }
                        if parts[0]=="slot", let size=Int(parts[1]) { before["slot"]=size }
                    }
                }
            }
        }
        if line.hasPrefix("OTAB READY ") && stage == "push" { ready=true;pump() }
        if words.count==3 && words[0]=="OTAB" && words[1]=="CRED" && stage == "push" {
            guard let n=Int(words[2]),n>=grant,n<=bytes.count else { fail("invalid credit");return }
            grant=n;pump()
        }
        if words.count==3 && words[1]=="PROG" && words[2]=="\(bytes.count)/\(bytes.count)" {
            guard sent==bytes.count, stage == "push", !endSent else { fail("invalid final progress");return }
            endSent=true;stage="reboot";command("end")
        }
        if line == "OTAB OK rebooting" { ok=true }
    }
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral p: CBPeripheral,error: Error?) {
        if finished { return }
        guard stage=="reboot", endSent, sent==bytes.count else { fail("premature disconnect");return }
        finished=true
        let record:[String:Any] = ["elapsed":elapsed(),"bytes":bytes.count,"before":before,
            "lines":lines,"transfer_hint":ok,"sent":sent,"ready_callbacks":callbacks,
            "writes":writes,"payload_sha256":digest,"backend":"native-CoreBluetooth"]
        do {
            let data=try JSONSerialization.data(withJSONObject:record,options:[.sortedKeys])
            print("NATIVE_RESULT "+String(decoding:data,as:UTF8.self));fflush(stdout);exit(0)
        } catch { print("NATIVE FAIL serializing result");exit(1) }
    }
}

guard CommandLine.arguments.count==3 else {
    print("usage: native_push IMAGE EXPECTED_RECEIVER_IMAGE_SHA");exit(2)
}
do {
    let push=try Push(path:CommandLine.arguments[1],base:CommandLine.arguments[2])
    withExtendedLifetime(push) { RunLoop.main.run() }
} catch { print("NATIVE FAIL \(error)");exit(1) }
