import CoreBluetooth
import Foundation

/// Connection to the robot's motor controller. Same JSON messages over two transports
/// (robot/PROTOCOL.md):
/// * Wi-Fi: one JSON object per WebSocket text message (URLSessionWebSocketTask).
/// * Bluetooth LE: newline-terminated JSON over the Nordic UART Service (NUS), which ESP32, nRF52 and
///   HM-10-style modules all offer.
/// `send` may be called from any thread.
final class RobotLink: NSObject, ObservableObject {
    enum State: Equatable {
        case off
        case connecting(String)
        case connected(String)
        case failed(String)

        var label: String {
            switch self {
            case .off: return "robot: off"
            case .connecting(let s): return "connecting \(s)"
            case .connected(let s): return s
            case .failed(let s): return s
            }
        }
        var isConnected: Bool { if case .connected = self { return true } else { return false } }
    }

    @Published private(set) var state: State = .off
    @Published private(set) var rttMs: Double?
    @Published private(set) var robotStatus = ""     // last "status" message, condensed
    @Published private(set) var sent = 0
    @Published private(set) var received = 0

    /// Called (on the main queue) for every message from the robot.
    var onMessage: (([String: Any]) -> Void)?

    private var kind: RobotLinkKind = .none
    private var ws: URLSessionWebSocketTask?
    private var urlSession: URLSession?
    private var pingTimer: Timer?
    private var seq = 0
    private let seqLock = NSLock()
    private var ble: BLEUART?
    private let flagLock = NSLock()
    private var connectedFlag = false

    /// Thread-safe "the robot is answering" flag (the @Published state is main-thread only).
    var isConnectedNow: Bool {
        flagLock.lock()
        defer { flagLock.unlock() }
        return connectedFlag
    }

    private func setConnectedFlag(_ v: Bool) {
        flagLock.lock()
        connectedFlag = v
        flagLock.unlock()
    }

    func nextSeq() -> Int {
        seqLock.lock()
        defer { seqLock.unlock() }
        seq += 1
        return seq
    }

    func connect(_ s: NavSettings) {
        disconnect()
        kind = s.link
        switch s.link {
        case .none:
            setState(.off)
        case .websocket:
            guard let url = URL(string: s.wsURL), url.scheme == "ws" || url.scheme == "wss" else {
                setState(.failed("bad URL \(s.wsURL)"))
                return
            }
            setState(.connecting(url.host ?? s.wsURL))
            let session = URLSession(configuration: .default)
            let task = session.webSocketTask(with: url)
            urlSession = session
            ws = task
            task.resume()
            receiveLoop(task)
            sendRaw(RobotProtocol.hello(name: "R2S Capture"))
        case .ble:
            setState(.connecting("BLE \(s.bleName)"))
            let b = BLEUART(namePrefix: s.bleName)
            b.onState = { [weak self] st in self?.setState(st) }
            b.onLine = { [weak self] line in self?.handle(line) }
            ble = b
            b.start()
        }
        DispatchQueue.main.async {
            self.pingTimer?.invalidate()
            self.pingTimer = Timer.scheduledTimer(withTimeInterval: 1.0, repeats: true) { [weak self] _ in
                guard let self = self else { return }
                self.sendRaw(RobotProtocol.ping(seq: self.nextSeq(), t: ProcessInfo.processInfo.systemUptime))
            }
        }
    }

    func disconnect() {
        DispatchQueue.main.async {
            self.pingTimer?.invalidate()
            self.pingTimer = nil
        }
        if let ws = ws {
            if let s = RobotProtocol.encode(RobotProtocol.stop(seq: nextSeq())) {
                ws.send(.string(s)) { _ in }
            }
            ws.cancel(with: .goingAway, reason: nil)
        }
        ws = nil
        urlSession?.invalidateAndCancel()
        urlSession = nil
        ble?.sendLine(RobotProtocol.encode(RobotProtocol.stop(seq: nextSeq())) ?? "")
        ble?.stop()
        ble = nil
        setState(.off)
    }

    @discardableResult
    func send(_ obj: [String: Any]) -> Bool {
        guard isConnectedNow else { return false }
        return sendRaw(obj)
    }

    @discardableResult
    private func sendRaw(_ obj: [String: Any]) -> Bool {
        guard let s = RobotProtocol.encode(obj) else { return false }
        switch kind {
        case .websocket:
            guard let ws = ws else { return false }
            ws.send(.string(s)) { [weak self] err in
                if let err = err { self?.setState(.failed("send: \(err.localizedDescription)")) }
            }
        case .ble:
            guard let b = ble else { return false }
            b.sendLine(s)
        case .none:
            return false
        }
        DispatchQueue.main.async { self.sent += 1 }
        return true
    }

    private func receiveLoop(_ task: URLSessionWebSocketTask) {
        task.receive { [weak self] result in
            guard let self = self, self.ws === task else { return }
            switch result {
            case .failure(let err):
                self.setState(.failed("Wi-Fi: \(err.localizedDescription)"))
            case .success(let msg):
                switch msg {
                case .string(let s): self.handle(s)
                case .data(let d): self.handle(String(decoding: d, as: UTF8.self))
                @unknown default: break
                }
                self.receiveLoop(task)
            }
        }
    }

    private func handle(_ line: String) {
        guard let obj = RobotProtocol.decode(line) else { return }
        let type = obj["type"] as? String ?? ""
        DispatchQueue.main.async {
            self.received += 1
            switch type {
            case "hello":
                let name = obj["name"] as? String ?? "robot"
                self.state = .connected(name)
                self.setConnectedFlag(true)
            case "pong":
                if let t = obj["t"] as? Double { self.rttMs = (ProcessInfo.processInfo.systemUptime - t) * 1000 }
                if !self.state.isConnected { self.state = .connected("robot") }
                self.setConnectedFlag(true)
            case "status":
                var parts: [String] = []
                if let v = obj["battery_v"] as? Double { parts.append(String(format: "%.1f V", v)) }
                if let l = obj["left"] as? Double, let r = obj["right"] as? Double {
                    parts.append(String(format: "L %.2f R %.2f", l, r))
                }
                if let b = obj["busy"] as? Bool, b { parts.append("busy") }
                if let e = obj["estop"] as? Bool, e { parts.append("E-STOP") }
                self.robotStatus = parts.joined(separator: " · ")
            default:
                break
            }
            self.onMessage?(obj)
        }
    }

    private func setState(_ s: State) {
        // A WebSocket counts as connected once the robot answers (hello or pong); BLE once the UART
        // characteristics are found.
        setConnectedFlag(s.isConnected)
        DispatchQueue.main.async { self.state = s }
    }
}

// MARK: - BLE Nordic UART client

/// Minimal NUS central: scans for the UART service (optionally filtered by name prefix), connects to the
/// first match, subscribes to TX notifications and writes to RX in MTU-sized chunks.
final class BLEUART: NSObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    static let service = CBUUID(string: "6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
    static let rxChar = CBUUID(string: "6E400002-B5A3-F393-E0A9-E50E24DCCA9E")   // phone -> robot (write)
    static let txChar = CBUUID(string: "6E400003-B5A3-F393-E0A9-E50E24DCCA9E")   // robot -> phone (notify)

    var onState: ((RobotLink.State) -> Void)?
    var onLine: ((String) -> Void)?

    private let namePrefix: String
    private let queue = DispatchQueue(label: "r2s.ble")
    private var central: CBCentralManager?
    private var peripheral: CBPeripheral?
    private var rx: CBCharacteristic?
    private var inbox = Data()

    init(namePrefix: String) {
        self.namePrefix = namePrefix
        super.init()
    }

    func start() {
        queue.async {
            self.central = CBCentralManager(delegate: self, queue: self.queue)
        }
    }

    func stop() {
        queue.async {
            if let p = self.peripheral { self.central?.cancelPeripheralConnection(p) }
            self.central?.stopScan()
            self.peripheral = nil
            self.rx = nil
            self.central = nil
        }
    }

    func sendLine(_ line: String) {
        guard !line.isEmpty else { return }
        queue.async {
            guard let p = self.peripheral, let rx = self.rx else { return }
            let data = Data((line + "\n").utf8)
            let type: CBCharacteristicWriteType = rx.properties.contains(.writeWithoutResponse) ? .withoutResponse : .withResponse
            let chunk = max(20, p.maximumWriteValueLength(for: type))
            var i = 0
            while i < data.count {
                let end = min(data.count, i + chunk)
                p.writeValue(data.subdata(in: i..<end), for: rx, type: type)
                i = end
            }
        }
    }

    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .poweredOn:
            onState?(.connecting("BLE scanning"))
            central.scanForPeripherals(withServices: [BLEUART.service], options: nil)
        case .unauthorized: onState?(.failed("Bluetooth permission denied"))
        case .poweredOff: onState?(.failed("Bluetooth is off"))
        case .unsupported: onState?(.failed("BLE unsupported"))
        default: break
        }
    }

    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any], rssi RSSI: NSNumber) {
        let name = peripheral.name ?? (advertisementData[CBAdvertisementDataLocalNameKey] as? String) ?? ""
        guard namePrefix.isEmpty || name.hasPrefix(namePrefix) else { return }
        central.stopScan()
        self.peripheral = peripheral
        peripheral.delegate = self
        onState?(.connecting("BLE \(name)"))
        central.connect(peripheral, options: nil)
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        peripheral.discoverServices([BLEUART.service])
    }

    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        onState?(.failed("BLE connect failed"))
    }

    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        rx = nil
        onState?(.connecting("BLE reconnecting"))
        central.connect(peripheral, options: nil)
    }

    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        for s in peripheral.services ?? [] where s.uuid == BLEUART.service {
            peripheral.discoverCharacteristics([BLEUART.rxChar, BLEUART.txChar], for: s)
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        for c in service.characteristics ?? [] {
            if c.uuid == BLEUART.rxChar { rx = c }
            if c.uuid == BLEUART.txChar { peripheral.setNotifyValue(true, for: c) }
        }
        if rx != nil {
            onState?(.connected("BLE \(peripheral.name ?? "robot")"))
            sendLine(RobotProtocol.encode(RobotProtocol.hello(name: "R2S Capture")) ?? "")
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        guard let d = characteristic.value else { return }
        inbox.append(d)
        while let nl = inbox.firstIndex(of: 0x0A) {
            let line = String(decoding: inbox[inbox.startIndex..<nl], as: UTF8.self)
            inbox.removeSubrange(inbox.startIndex...nl)
            if !line.isEmpty { onLine?(line) }
        }
        if inbox.count > 8192 { inbox.removeAll() }
    }
}
