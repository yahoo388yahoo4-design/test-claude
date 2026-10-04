import ARKit
import AVFoundation
import Foundation
import SceneKit
import UIKit

/// Everything the navigation screen shows, published ~10 times a second.
struct NavHUD {
    var tracking = "starting"
    var trackingOK = false
    var mapStatus = "new map"
    var mapping = "-"
    var pose: Pose2?
    var cameraHeight = 0.0           // m above the floor
    var floorFound = false
    var floorY: Float = 0
    var pointingDown = false
    var speed = 0.0                  // measured, m/s along the heading
    var turnRate = 0.0               // measured, deg/s, + left
    var cmd = DriveCommand.zero      // last command sent to the robot
    var sectorBearings: [Double] = NavEngine.sectorBearings
    var sectors: [Double] = Array(repeating: .infinity, count: NavEngine.sectorBearings.count) // m from robot edge
    var markers: [SIMD3<Float>?] = Array(repeating: nil, count: NavEngine.sectorBearings.count)
    var scan: [Double] = []          // 72-ray virtual lidar from the map, m from robot centre
    var scanMax = 4.0
    var nearest = Double.infinity    // m from robot edge
    var nearestBearing = 0.0         // deg, + left
    var front = Double.infinity, left = Double.infinity, right = Double.infinity, rear = Double.infinity
    var frontGap = Double.infinity
    var goal: P2?
    var goalLabel = ""
    var path: [P2] = []
    var pathVersion = 0
    var lookahead: P2?
    var remaining = 0.0
    var eta = 0.0
    var headingError = 0.0           // deg, + = turn left
    var instruction = "Tap the floor or the map to pick a goal"
    var state = "Idle"
    var depthImage: CGImage?
    var mapImage: CGImage?
    var mapCenter = P2(0, 0)
    var mapSpan = 8.0
    var fps = 0.0
    var depthPoints = 0
    var obstaclePoints = 0
    var thermal = "nominal"
    var logName = ""
}

/// Navigation mode engine: its own ARSession (world tracking + LiDAR depth, optionally relocalised in a
/// saved ARWorldMap), a live occupancy grid, A* planning, path following, obstacle safety, the robot link,
/// voice / beep guidance and a log written as a session folder. All per-frame work runs on `queue`.
final class NavEngine: NSObject, ObservableObject, ARSessionDelegate {
    static let sectorBearings: [Double] = stride(from: 40.0, through: -40.0, by: -10.0).map { $0 } // left -> right

    let session = ARSession()
    let link = RobotLink()
    let voice = VoiceGuide()

    @Published private(set) var hud = NavHUD()
    @Published private(set) var roomItems: [RoomItem] = []
    @Published private(set) var mapChoices: [URL] = []
    @Published var toast = ""
    @Published var settings = NavSettings.load() {
        didSet {
            settings.save()
            let s = settings
            queue.async { self.cfg = s }
        }
    }
    @Published var mode: NavDriveMode = .guide {
        didSet {
            let m = mode
            queue.async { self.setMode(m) }
        }
    }
    @Published private(set) var isLogging = false

    private let queue = DispatchQueue(label: "r2s.nav", qos: .userInitiated)

    // Queue-confined state.
    private var cfg = NavSettings.load()
    private var driveMode: NavDriveMode = .guide
    private var grid = OccupancyGrid(cell: 0.05, sizeMeters: 30)
    private var h = NavHUD()
    private var pose: Pose2?
    private var lastPose: Pose2?
    private var lastPoseT: TimeInterval = 0
    private var floorY: Float?
    private var goal: P2?
    private var path: [P2] = []
    private var task: NavTask = .idle
    private var lastTick: TimeInterval = 0
    private var lastDepth: TimeInterval = 0
    private var lastDepthImage: TimeInterval = 0
    private var lastPlan: TimeInterval = 0
    private var lastMapImage: TimeInterval = 0
    private var lastScan: TimeInterval = 0
    private var lastPublish: TimeInterval = 0
    private var lastSend: TimeInterval = 0
    private var lastSpokenKey = ""
    private var blockedSince: TimeInterval?
    private var wasTrackingOK = false
    private var loadedMapName = ""
    private var relocalising = false
    private var frameTimes: [TimeInterval] = []
    private var liveFrontGap = Double.infinity
    private var liveFrontT: TimeInterval = 0
    private var logger: NavLogger?
    private var lastLog: TimeInterval = 0
    private var stoppedSent = true
    private var lastObstacleWarning: TimeInterval = 0
    private var beepTimer: Timer?

    enum NavTask {
        case idle
        case follow
        case move(start: P2, theta: Double, dist: Double, speed: Double, began: TimeInterval)
        case turn(lastTheta: Double, turned: Double, target: Double, rate: Double, began: TimeInterval)
        case joystick(DriveCommand, at: TimeInterval)
        case backup(until: TimeInterval)

        var label: String {
            switch self {
            case .idle: return "Idle"
            case .follow: return "Following path"
            case .move(_, _, let d, _, _): return String(format: "Moving %.0f cm", d * 100)
            case .turn(_, _, let t, _, _): return String(format: "Turning %.0f°", deg(t))
            case .joystick: return "Manual"
            case .backup: return "Backing up"
            }
        }
    }

    override init() {
        super.init()
        session.delegateQueue = queue
        session.delegate = self
        link.onMessage = { [weak self] msg in self?.robotMessage(msg) }
    }

    // MARK: lifecycle

    func start() {
        refreshMapChoices()
        restart()
        if settings.link != .none { link.connect(settings) }
        UIApplication.shared.isIdleTimerDisabled = true
        beepTimer?.invalidate()
        beepTimer = Timer.scheduledTimer(withTimeInterval: 0.05, repeats: true) { [weak self] _ in
            guard let self = self, self.settings.beeps else { return }
            let s = self.settings
            self.voice.proximity(gap: self.hud.frontGap, stop: s.stopDistance, slow: s.slowDistance)
        }
    }

    func shutdown() {
        queue.async {
            self.task = .idle
            self.sendStop()
            self.logger?.close(hud: self.h)
            self.logger = nil
        }
        beepTimer?.invalidate()
        beepTimer = nil
        link.disconnect()
        session.pause()
        voice.stopAll()
        UIApplication.shared.isIdleTimerDisabled = false
        DispatchQueue.main.async { self.isLogging = false }
    }

    func refreshMapChoices() {
        mapChoices = SessionStorage.listSessions().filter {
            FileManager.default.fileExists(atPath: $0.appendingPathComponent("worldmap.arworldmap").path)
        }
    }

    /// (Re)starts tracking with the selected map: loads its ARWorldMap for relocalisation and its mesh,
    /// RoomPlan room and saved grid as the static map layer.
    func restart() {
        let s = settings
        queue.async {
            self.task = .idle
            self.sendStop()
            self.grid = OccupancyGrid(cell: 0.05, sizeMeters: 30)
            self.goal = nil
            self.path = []
            self.floorY = nil
            self.pose = nil
            self.lastPose = nil
            var worldMap: ARWorldMap?
            var items: [RoomItem] = []
            var status = "new map (nothing loaded)"
            if !s.mapSession.isEmpty {
                let dir = SessionStorage.root.appendingPathComponent(s.mapSession, isDirectory: true)
                if let data = try? Data(contentsOf: dir.appendingPathComponent("worldmap.arworldmap")) {
                    worldMap = try? NSKeyedUnarchiver.unarchivedObject(ofClass: ARWorldMap.self, from: data)
                }
                let rep = MapPrior.load(dir: dir, into: self.grid, robotHeight: s.robotHeight)
                items = rep.roomItems
                status = worldMap == nil ? "\(s.mapSession): no world map, using \(rep.summary)" : "relocalising in \(s.mapSession) · \(rep.summary)"
            }
            self.loadedMapName = s.mapSession
            self.relocalising = worldMap != nil
            self.h.mapStatus = status
            self.h.goal = nil
            self.h.path = []
            self.h.pathVersion += 1
            DispatchQueue.main.async { self.roomItems = items }
            let c = ARWorldTrackingConfiguration()
            c.worldAlignment = .gravity
            c.planeDetection = [.horizontal]
            if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) { c.frameSemantics.insert(.sceneDepth) }
            if ARWorldTrackingConfiguration.supportsFrameSemantics(.smoothedSceneDepth) { c.frameSemantics.insert(.smoothedSceneDepth) }
            if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) { c.sceneReconstruction = .mesh }
            c.initialWorldMap = worldMap
            self.session.run(c, options: [.resetTracking, .removeExistingAnchors])
        }
    }

    // MARK: user actions (main thread)

    func setGoal(_ p: P2, label: String = "") {
        queue.async {
            self.goal = p
            self.h.goal = p
            self.h.goalLabel = label
            self.lastPlan = 0
            self.lastSpokenKey = ""
            if self.driveMode == .auto { self.task = .follow }
            self.logger?.event(["event": "goal", "x": p.x, "z": p.z, "label": label])
        }
    }

    func setGoal(item: RoomItem) {
        queue.async {
            let from = self.pose?.p ?? item.center
            let p = item.approachPoint(from: from, clearance: self.cfg.robotRadius + 0.25)
            DispatchQueue.main.async { self.setGoal(p, label: item.label) }
        }
    }

    func clearGoal() {
        queue.async {
            self.goal = nil
            self.path = []
            self.h.goal = nil
            self.h.goalLabel = ""
            self.h.path = []
            self.h.pathVersion += 1
            if case .follow = self.task { self.task = .idle }
            self.sendStop()
        }
    }

    /// Auto mode: start (or resume) driving to the goal.
    func go() {
        queue.async {
            guard self.goal != nil else { self.say("Pick a goal first", key: "nogoal"); return }
            self.task = .follow
            self.blockedSince = nil
            self.lastPlan = 0
        }
    }

    func stop() {
        queue.async {
            self.task = .idle
            self.sendStop()
            self.logger?.event(["event": "stop"])
        }
        Haptics.warning()
    }

    func joystick(v: Double, w: Double) {
        queue.async {
            self.task = .joystick(DriveCommand(v: v, w: w), at: ProcessInfo.processInfo.systemUptime)
        }
    }

    /// Move `cm` centimetres (negative = backwards) at `speed` cm/s.
    func move(cm: Double, speed: Double) {
        queue.async {
            if !self.cfg.closedLoopMoves {
                self.link.send(RobotProtocol.move(cm: cm, speedCmS: speed, seq: self.link.nextSeq()))
                self.logger?.event(["event": "move_open_loop", "cm": cm, "speed": speed])
                return
            }
            guard let p = self.pose else { return }
            self.task = .move(start: p.p, theta: p.theta, dist: cm / 100, speed: speed / 100, began: ProcessInfo.processInfo.systemUptime)
            self.logger?.event(["event": "move", "cm": cm, "speed": speed])
        }
    }

    /// Turn `degrees` (+ left) at `speed` deg/s.
    func turn(degrees: Double, speed: Double) {
        queue.async {
            if !self.cfg.closedLoopMoves {
                self.link.send(RobotProtocol.turn(deg: degrees, speedDegS: speed, seq: self.link.nextSeq()))
                self.logger?.event(["event": "turn_open_loop", "deg": degrees, "speed": speed])
                return
            }
            guard let p = self.pose else { return }
            self.task = .turn(lastTheta: p.theta, turned: 0, target: rad(degrees), rate: rad(speed), began: ProcessInfo.processInfo.systemUptime)
            self.logger?.event(["event": "turn", "deg": degrees, "speed": speed])
        }
    }

    func clearMap() {
        queue.async {
            self.grid.reset()
            self.grid.clearAllStatic()
        }
    }

    func reconnect() {
        link.connect(settings)
    }

    func toggleLogging() {
        queue.async {
            if let l = self.logger {
                l.close(hud: self.h)
                self.logger = nil
                DispatchQueue.main.async { self.isLogging = false; self.toast = "Log saved: \(l.name)" }
            } else if let l = NavLogger(settings: self.cfg, mapName: self.loadedMapName) {
                self.logger = l
                self.h.logName = l.name
                DispatchQueue.main.async { self.isLogging = true }
            }
        }
    }

    /// Saves the current ARWorldMap, scene mesh and occupancy grid as a new session folder, which then
    /// shows up as a map choice (and converts like a mode A session).
    func saveMap() {
        DispatchQueue.main.async { self.toast = "Saving map…" }
        session.getCurrentWorldMap { [weak self] map, error in
            guard let self = self else { return }
            self.queue.async {
                guard let dir = NavLogger.newSessionDir(suffix: "navmap") else { return }
                var saved: [String] = []
                if let map = map, let data = try? NSKeyedArchiver.archivedData(withRootObject: map, requiringSecureCoding: true) {
                    try? data.write(to: dir.appendingPathComponent("worldmap.arworldmap"))
                    saved.append("world map")
                }
                try? self.grid.pgm().write(to: dir.appendingPathComponent("map.pgm"))
                try? self.grid.rosYAML(imageName: "map.pgm").write(to: dir.appendingPathComponent("map.yaml"), atomically: true, encoding: .utf8)
                saved.append("grid")
                if let anchors = self.session.currentFrame?.anchors {
                    MeshExporter.writeMesh(anchors: anchors, to: dir.appendingPathComponent("mesh.ply"))
                    MeshExporter.writePlanes(anchors: anchors, to: dir.appendingPathComponent("planes.json"))
                    saved.append("mesh")
                }
                // Keep the RoomPlan room of the map this one was built on, so the go-to menu survives.
                if !self.loadedMapName.isEmpty {
                    let src = SessionStorage.root.appendingPathComponent(self.loadedMapName).appendingPathComponent("roomplan")
                    if FileManager.default.fileExists(atPath: src.path) {
                        try? FileManager.default.copyItem(at: src, to: dir.appendingPathComponent("roomplan"))
                    }
                }
                var meta: [String: Any] = [
                    "format": "r2s-capture", "version": 1, "mode": "nav_map", "platform": "ios",
                    "device": SessionStorage.deviceModel, "app_version": SessionStorage.appVersion,
                    "start_unix": Date().timeIntervalSince1970, "complete": true,
                    "based_on": self.loadedMapName,
                    "grid": ["cell": self.grid.cell, "size": self.grid.size, "origin_xz": [self.grid.origin.x, self.grid.origin.z]],
                ]
                if let error = error { meta["world_map_error"] = error.localizedDescription }
                SessionStorage.writeJSON(meta, to: dir.appendingPathComponent("session.json"))
                DispatchQueue.main.async {
                    self.toast = "Saved \(dir.lastPathComponent): " + saved.joined(separator: ", ")
                    self.refreshMapChoices()
                }
            }
        }
    }

    // MARK: ARSessionDelegate (queue)

    func session(_ session: ARSession, didFailWithError error: Error) {
        DispatchQueue.main.async { self.toast = "AR error: \(error.localizedDescription)" }
    }

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let now = frame.timestamp
        frameTimes.append(now)
        if frameTimes.count > 30 { frameTimes.removeFirst(frameTimes.count - 30) }
        guard now - lastTick >= 1.0 / 15.0 else { return }
        lastTick = now

        // Tracking.
        var ok = false
        switch frame.camera.trackingState {
        case .normal:
            ok = true
            h.tracking = "tracking OK"
        case .notAvailable:
            h.tracking = "tracking not available"
        case .limited(let reason):
            switch reason {
            case .initializing: h.tracking = "initialising: move the phone a little"
            case .relocalizing: h.tracking = "relocalising: show the mapped area"
            case .excessiveMotion: h.tracking = "moving too fast"
            case .insufficientFeatures: h.tracking = "too few features (blank walls or dark)"
            @unknown default: h.tracking = "limited"
            }
        }
        h.trackingOK = ok
        h.mapping = ARRecorder.mappingString(frame.worldMappingStatus)
        if ok && relocalising {
            relocalising = false
            h.mapStatus = "relocalised in \(loadedMapName)"
            say("Map found", key: "reloc")
        }
        if !ok && wasTrackingOK && driveMode != .guide {
            sendStop()
            say("Tracking lost, stopping", key: "lost")
        }
        wasTrackingOK = ok

        // Pose of the robot from the camera.
        let T = frame.camera.transform
        let c = T.columns.3
        let fwd = -T.columns.2
        let fl = (fwd.x * fwd.x + fwd.z * fwd.z).squareRoot()
        h.pointingDown = fl < 0.35
        let theta = fl > 0.25 ? atan2(Double(-fwd.z), Double(fwd.x)) : (pose?.theta ?? 0)
        let robotP = P2(Double(c.x), Double(c.z)) - P2(cos(theta), -sin(theta)) * cfg.mountForward
        let newPose = Pose2(p: robotP, theta: theta)
        if let lp = lastPose, now - lastPoseT > 0 {
            let ddt = now - lastPoseT
            let d = newPose.p - lp.p
            let v = (d.x * cos(theta) - d.z * sin(theta)) / ddt
            let w = deg(wrapAngle(theta - lp.theta)) / ddt
            h.speed = 0.7 * h.speed + 0.3 * v
            h.turnRate = 0.7 * h.turnRate + 0.3 * w
        }
        lastPose = newPose
        lastPoseT = now
        pose = newPose
        h.pose = newPose

        // Floor height: the lowest floor plane below the camera, else the configured mount height.
        var planeFloor: Float?
        for a in frame.anchors {
            guard let p = a as? ARPlaneAnchor, p.alignment == .horizontal else { continue }
            let y = (p.transform * simd_float4(p.center, 1)).y
            guard y < c.y - 0.05 else { continue }
            var isFloor = false
            if case .floor = p.classification { isFloor = true }
            if ARPlaneAnchor.isClassificationSupported, !isFloor, y > c.y - 0.15 { continue }
            planeFloor = min(planeFloor ?? y, y)
        }
        if let pf = planeFloor {
            floorY = floorY.map { 0.9 * $0 + 0.1 * pf } ?? pf
            h.floorFound = true
        } else if !h.floorFound {
            floorY = c.y - Float(cfg.mountHeight)
        }
        let fy = floorY ?? (c.y - Float(cfg.mountHeight))
        h.floorY = fy
        h.cameraHeight = Double(c.y - fy)

        // Depth -> obstacles, sectors, grid (10 Hz).
        if ok, now - lastDepth >= 0.1, let depth = frame.smoothedSceneDepth ?? frame.sceneDepth {
            lastDepth = now
            processDepth(depth, frame: frame, floorY: fy, pose: newPose, now: now)
        }
        if now - lastDepthImage >= 0.2, let depth = frame.sceneDepth ?? frame.smoothedSceneDepth {
            lastDepthImage = now
            h.depthImage = DepthColormap.image(depth.depthMap, maxMeters: 4)
        }
        if now - liveFrontT > 0.5 { liveFrontGap = .infinity }

        // Virtual 360 lidar from the map (5 Hz).
        if now - lastScan >= 0.2 {
            lastScan = now
            let scan = grid.scan(pose: newPose, count: 72, maxRange: h.scanMax)
            h.scan = scan
            func window(_ centerDeg: Double, _ half: Double) -> Double {
                var m = Double.infinity
                for (k, r) in scan.enumerated() {
                    let b = Double(k) * 5
                    if abs(wrapAngle(rad(b - centerDeg))) <= rad(half), r < h.scanMax { m = min(m, r) }
                }
                return max(0, m - cfg.robotRadius)
            }
            h.left = window(90, 30)
            h.right = window(-90, 30)
            h.rear = window(180, 30)
            let mapFront = window(0, 20)
            h.front = min(mapFront, h.sectors.min() ?? .infinity)
            var nearest = h.front
            var nb = 0.0
            for (k, r) in scan.enumerated() where r - cfg.robotRadius < nearest {
                nearest = max(0, r - cfg.robotRadius)
                nb = wrapAngle(rad(Double(k) * 5))
            }
            for (k, g) in h.sectors.enumerated() where g < nearest {
                nearest = g
                nb = rad(NavEngine.sectorBearings[k])
            }
            h.nearest = nearest
            h.nearestBearing = deg(nb)
        }
        h.frontGap = min(liveFrontGap, corridorGap(newPose))

        // Plan (2 Hz).
        if ok, let g = goal, now - lastPlan >= 0.5 {
            lastPlan = now
            replan(from: newPose.p, to: g)
        }

        control(now: now, ok: ok, pose: newPose)

        if now - lastMapImage >= 0.33 {
            lastMapImage = now
            let span = cfg.minimapSpan
            h.mapImage = MapImage.render(grid: grid, center: newPose.p, span: span)
            h.mapCenter = newPose.p
            h.mapSpan = span
        }

        if now - lastLog >= 0.1, let l = logger {
            lastLog = now
            l.sample(t: now, hud: h, task: task.label, mode: driveMode.rawValue)
        }

        if now - lastPublish >= 0.1 {
            lastPublish = now
            if let first = frameTimes.first, let last = frameTimes.last, last > first {
                h.fps = Double(frameTimes.count - 1) / (last - first)
            }
            h.thermal = NavEngine.thermalString(ProcessInfo.processInfo.thermalState)
            let snapshot = h
            DispatchQueue.main.async { self.hud = snapshot }
        }
    }

    private func processDepth(_ depth: ARDepthData, frame: ARFrame, floorY fy: Float, pose: Pose2, now: TimeInterval) {
        let dm = depth.depthMap
        CVPixelBufferLockBaseAddress(dm, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(dm, .readOnly) }
        guard let base = CVPixelBufferGetBaseAddress(dm) else { return }
        let w = CVPixelBufferGetWidth(dm), hgt = CVPixelBufferGetHeight(dm)
        let rowBytes = CVPixelBufferGetBytesPerRow(dm)
        var confBase: UnsafeMutableRawPointer?
        var confRow = 0
        if let cm = depth.confidenceMap {
            CVPixelBufferLockBaseAddress(cm, .readOnly)
            confBase = CVPixelBufferGetBaseAddress(cm)
            confRow = CVPixelBufferGetBytesPerRow(cm)
        }
        defer { if let cm = depth.confidenceMap { CVPixelBufferUnlockBaseAddress(cm, .readOnly) } }

        let K = frame.camera.intrinsics
        let res = frame.camera.imageResolution
        let sx = Float(w) / Float(res.width), sy = Float(hgt) / Float(res.height)
        let fx = K[0][0] * sx, fyy = K[1][1] * sy, cx = K[2][0] * sx, cy = K[2][1] * sy
        let T = frame.camera.transform
        let robotH = Float(cfg.robotHeight)
        let radius = cfg.robotRadius
        let sensor = P2(Double(T.columns.3.x), Double(T.columns.3.z))
        let cosT = cos(pose.theta), sinT = sin(pose.theta)

        var hits: [P2] = []
        var floorPts: [P2] = []
        hits.reserveCapacity(4096)
        floorPts.reserveCapacity(4096)
        var sectors = [Double](repeating: .infinity, count: NavEngine.sectorBearings.count)
        var markers = [SIMD3<Float>?](repeating: nil, count: NavEngine.sectorBearings.count)
        var frontGap = Double.infinity
        var used = 0
        let step = 3
        for v in stride(from: 0, to: hgt, by: step) {
            let row = base.advanced(by: v * rowBytes).assumingMemoryBound(to: Float32.self)
            let crow = confBase?.advanced(by: v * confRow).assumingMemoryBound(to: UInt8.self)
            for u in stride(from: 0, to: w, by: step) {
                let d = row[u]
                guard d > 0.15, d < 5 else { continue }
                if let crow = crow, crow[u] < 1 { continue }
                used += 1
                let pc = simd_float4((Float(u) - cx) * d / fx, -(Float(v) - cy) * d / fyy, -d, 1)
                let wp = T * pc
                let height = wp.y - fy
                let q = P2(Double(wp.x), Double(wp.z))
                let dx = q.x - pose.p.x, dz = q.z - pose.p.z
                let lf = dx * cosT - dz * sinT
                let ll = -dx * sinT - dz * cosT
                let dist = (lf * lf + ll * ll).squareRoot()
                if dist < radius + 0.03 { continue }               // the robot's own body
                if height < 0.04 {
                    if height > -0.08 { floorPts.append(q); continue }
                    if !h.floorFound || height > -0.15 { continue }  // a drop (stairs down, a hole): obstacle
                } else if height > robotH {
                    continue
                }
                hits.append(q)
                let gap = dist - radius
                let b = deg(atan2(ll, lf))
                let idx = Int(((40 - b) / 10).rounded())
                if idx >= 0, idx < sectors.count, abs(b) <= 45, gap < sectors[idx] {
                    sectors[idx] = gap
                    markers[idx] = simd_float3(wp.x, wp.y, wp.z)
                }
                if lf > 0, abs(ll) < radius + 0.03 { frontGap = min(frontGap, lf - radius) }
            }
        }
        grid.integrate(sensor: sensor, hits: hits, floor: floorPts, maxRange: 4.5)
        h.sectors = sectors.map { max(0, $0) }
        h.markers = markers
        h.depthPoints = used
        h.obstaclePoints = hits.count
        liveFrontGap = max(0, frontGap)
        liveFrontT = now
    }

    /// Forward gap from the map: first occupied cell in the robot's corridor, up to 2 m.
    private func corridorGap(_ p: Pose2) -> Double {
        let r = cfg.robotRadius
        let fwd = p.forward
        let left = P2(-sin(p.theta), -cos(p.theta))
        var d = r
        while d < r + 2 {
            for lat in [-r, -r / 2, 0, r / 2, r] {
                if grid.isOccupied(p.p + fwd * d + left * lat) { return max(0, d - r) }
            }
            d += grid.cell
        }
        return .infinity
    }

    private func replan(from: P2, to: P2) {
        switch Planner.plan(grid: grid, start: from, goal: to, cfg: cfg.plannerConfig) {
        case .ok(let p):
            path = p
            h.state = h.state == "No path" ? "Idle" : h.state
        case .noPath:
            path = []
            h.state = "No path"
            say("No path to the goal", key: "nopath")
        case .goalBlocked:
            path = []
            h.state = "Goal blocked"
            say("The goal is inside an obstacle", key: "goalblocked")
        case .startBlocked:
            path = []
            h.state = "Outside the map"
        }
        h.path = path
        h.pathVersion += 1
    }

    // MARK: control loop (queue, 15 Hz)

    private func control(now: TimeInterval, ok: Bool, pose: Pose2) {
        let fc = cfg.followerConfig
        let gap = h.frontGap
        var cmd = DriveCommand.zero
        var state = task.label

        // Guidance for every mode with a path (Guide mode speaks it; Auto shows it).
        if let g = goal, !path.isEmpty {
            let out = Follower.step(path: path, pose: pose, frontGap: gap, cfg: fc)
            h.remaining = out.remaining
            h.headingError = deg(out.headingError)
            h.lookahead = out.target
            let speed = max(0.05, driveMode == .guide ? 0.8 : max(0.05, abs(h.speed)))
            h.eta = out.remaining / speed
            let ins = Guidance.instruction(path: path, pose: pose, frontGap: gap, cfg: fc)
            h.instruction = ins.text
            if driveMode == .guide || driveMode == .auto { say(ins.text, key: ins.key) }
            if out.state == .arrived {
                if case .follow = task { task = .idle; sendStop() }
                if driveMode == .guide || h.state != "Arrived" { Haptics.success() }
                goal = nil
                path = []
                h.goal = nil
                h.path = []
                h.pathVersion += 1
                h.state = "Arrived"
                logger?.event(["event": "arrived", "x": g.x, "z": g.z])
                publishNow()
                return
            }
            if driveMode == .auto, case .follow = task, ok {
                cmd = out.cmd
                state = out.state.rawValue
                if out.state == .blocked {
                    if blockedSince == nil { blockedSince = now; Haptics.warning() }
                    if let b = blockedSince, now - b > 3, h.rear > 0.25 {
                        task = .backup(until: now + 1.0)
                        blockedSince = nil
                        lastPlan = 0
                        say("Blocked, backing up", key: "backup")
                    }
                } else {
                    blockedSince = nil
                }
            }
        } else if goal == nil {
            h.instruction = driveMode == .manual ? "Joystick or move / turn buttons" : "Tap the floor or the map to pick a goal"
            h.remaining = 0
            h.eta = 0
            h.lookahead = nil
        }

        switch task {
        case .idle, .follow:
            break
        case .backup(let until):
            if now < until && h.rear > 0.1 {
                cmd = DriveCommand(v: -0.08, w: 0)
            } else {
                task = goal != nil ? .follow : .idle
            }
        case .joystick(let c, let at):
            if now - at > 0.5 { task = .idle } else { cmd = Follower.safety(c, frontGap: gap, cfg: fc) }
        case .move(let start, let theta, let dist, let speed, let began):
            let d = pose.p - start
            let travelled = d.x * cos(theta) - d.z * sin(theta)
            let remaining = abs(dist) - abs(travelled)
            if remaining <= 0.01 || now - began > abs(dist) / max(0.02, speed) * 3 + 3 {
                task = .idle
                say("Done", key: "done-\(began)")
            } else {
                let v = (dist < 0 ? -1 : 1) * min(speed, max(0.05, 1.5 * remaining))
                // Hold the heading the move started with.
                let w = max(-0.5, min(0.5, 2.0 * wrapAngle(theta - pose.theta)))
                cmd = Follower.safety(DriveCommand(v: v, w: w), frontGap: gap, cfg: fc)
                if dist > 0 && cmd.v == 0 { state = "Blocked" }
            }
        case .turn(let lastTheta, let turned, let target, let rate, let began):
            let t2 = turned + wrapAngle(pose.theta - lastTheta)
            let remaining = abs(target) - abs(t2)
            if remaining <= rad(1.5) || now - began > abs(target) / max(0.1, rate) * 3 + 3 {
                task = .idle
                say("Done", key: "done-\(began)")
            } else {
                let w = (target < 0 ? -1 : 1) * min(rate, max(rad(12), 2.0 * remaining))
                cmd = DriveCommand(v: 0, w: w)
                task = .turn(lastTheta: pose.theta, turned: t2, target: target, rate: rate, began: began)
            }
        }

        if !ok { cmd = .zero }
        if driveMode == .guide { cmd = .zero }
        h.state = (h.state == "No path" || h.state == "Goal blocked") && goal != nil ? h.state : state
        sendCommand(cmd, now: now)

        // Obstacle warnings in every mode, at most every 3 s.
        if gap < fc.stopDistance + 0.05 && (driveMode == .guide || abs(h.speed) > 0.05 || cmd.v > 0),
           now - lastObstacleWarning > 3 {
            lastObstacleWarning = now
            say("Obstacle ahead, \(Guidance.cm(gap))", key: "obst-\(now)")
        }
    }

    private func sendCommand(_ c: DriveCommand, now: TimeInterval) {
        let active = c.v != 0 || c.w != 0
        h.cmd = c
        guard link.isConnectedNow else { return }
        if active {
            if now - lastSend >= 0.1 {
                lastSend = now
                link.send(RobotProtocol.vel(c, seq: link.nextSeq()))
                stoppedSent = false
            }
        } else if !stoppedSent {
            sendStop()
        }
    }

    private func sendStop() {
        link.send(RobotProtocol.stop(seq: link.nextSeq()))
        stoppedSent = true
        h.cmd = .zero
    }

    private func setMode(_ m: NavDriveMode) {
        driveMode = m
        task = .idle
        sendStop()
        lastSpokenKey = ""
        logger?.event(["event": "mode", "mode": m.rawValue])
    }

    private func publishNow() {
        let snapshot = h
        DispatchQueue.main.async { self.hud = snapshot }
    }

    private func say(_ text: String, key: String) {
        guard key != lastSpokenKey else { return }
        lastSpokenKey = key
        let s = cfg
        DispatchQueue.main.async {
            if s.voice { self.voice.speak(text) }
        }
    }

    private func robotMessage(_ msg: [String: Any]) {
        queue.async {
            self.logger?.event(["event": "robot", "msg": msg])
            if (msg["type"] as? String) == "estop" {
                self.task = .idle
                self.say("Robot emergency stop", key: "estop")
            }
        }
    }

    static func thermalString(_ s: ProcessInfo.ThermalState) -> String {
        switch s {
        case .nominal: return "nominal"
        case .fair: return "fair"
        case .serious: return "serious"
        case .critical: return "critical"
        @unknown default: return "?"
        }
    }

    // MARK: AR view tap

    /// World floor point under a screen point of `view` (ARKit raycast, else the ray hitting the floor height).
    func floorPoint(in view: ARSCNView, at pt: CGPoint) -> P2? {
        if let q = view.raycastQuery(from: pt, allowing: .estimatedPlane, alignment: .horizontal),
           let r = session.raycast(q).first {
            let c = r.worldTransform.columns.3
            return P2(Double(c.x), Double(c.z))
        }
        let near = view.unprojectPoint(SCNVector3(Float(pt.x), Float(pt.y), 0))
        let far = view.unprojectPoint(SCNVector3(Float(pt.x), Float(pt.y), 1))
        let fy = hud.floorY
        let dy = far.y - near.y
        guard abs(dy) > 1e-6 else { return nil }
        let t = (fy - near.y) / dy
        guard t > 0 else { return nil }
        return P2(Double(near.x + (far.x - near.x) * t), Double(near.z + (far.z - near.z) * t))
    }
}

// MARK: - logging

/// Writes a navigation run as a session folder: session.json + nav.jsonl (10 Hz samples and events).
final class NavLogger {
    let dir: URL
    let name: String
    private let writer: LineWriter
    private let startUnix = Date().timeIntervalSince1970
    private let startUptime = ProcessInfo.processInfo.systemUptime
    private let settings: NavSettings
    private let mapName: String

    static func newSessionDir(suffix: String) -> URL? {
        let f = DateFormatter()
        f.locale = Locale(identifier: "en_US_POSIX")
        f.dateFormat = "yyyyMMdd_HHmmss"
        let url = SessionStorage.root.appendingPathComponent("\(f.string(from: Date()))_\(suffix)", isDirectory: true)
        do {
            try FileManager.default.createDirectory(at: url, withIntermediateDirectories: true)
            return url
        } catch {
            return nil
        }
    }

    init?(settings: NavSettings, mapName: String) {
        guard let dir = NavLogger.newSessionDir(suffix: "nav"),
              let w = try? LineWriter(url: dir.appendingPathComponent("nav.jsonl")) else { return nil }
        self.dir = dir
        name = dir.lastPathComponent
        writer = w
        self.settings = settings
        self.mapName = mapName
        writeSession(final: false, hud: nil)
    }

    func sample(t: TimeInterval, hud h: NavHUD, task: String, mode: String) {
        var o: [String: Any] = [
            "t": t, "mode": mode, "task": task, "state": h.state,
            "v_meas": h.speed, "w_meas_dps": h.turnRate, "v_cmd": h.cmd.v, "w_cmd": h.cmd.w,
            "front_gap": h.frontGap.isFinite ? h.frontGap : -1,
            "nearest": h.nearest.isFinite ? h.nearest : -1, "nearest_bearing": h.nearestBearing,
            "sectors": h.sectors.map { $0.isFinite ? $0 : -1 },
            "tracking": h.tracking, "floor_y": Double(h.floorY),
        ]
        if let p = h.pose { o["x"] = p.p.x; o["z"] = p.p.z; o["theta"] = p.theta }
        if let g = h.goal { o["goal"] = [g.x, g.z]; o["remaining"] = h.remaining }
        writer.writeJSON(o)
    }

    func event(_ e: [String: Any]) {
        var o = e
        o["t"] = ProcessInfo.processInfo.systemUptime
        writer.writeJSON(o)
    }

    func close(hud: NavHUD) {
        writer.close()
        writeSession(final: true, hud: hud)
    }

    private func writeSession(final: Bool, hud: NavHUD?) {
        var o: [String: Any] = [
            "format": "r2s-capture", "version": 1, "mode": "nav", "platform": "ios",
            "device": SessionStorage.deviceModel,
            "os": "\(UIDevice.current.systemName) \(UIDevice.current.systemVersion)",
            "app_version": SessionStorage.appVersion,
            "start_unix": startUnix, "start_uptime": startUptime,
            "nav_settings": settings.asDictionary, "map_session": mapName,
            "complete": final,
            "notes": "nav.jsonl: 10 Hz samples (t = ARKit timestamp = uptime) and events; see NAVIGATION.md",
        ]
        if final {
            o["end_unix"] = Date().timeIntervalSince1970
            o["samples"] = writer.lines
        }
        SessionStorage.writeJSON(o, to: dir.appendingPathComponent("session.json"))
    }
}

// MARK: - images for the HUD

enum DepthColormap {
    /// False-colour LiDAR depth (near = red, far = blue, invalid = black), in sensor orientation.
    static func image(_ buf: CVPixelBuffer, maxMeters: Float) -> CGImage? {
        CVPixelBufferLockBaseAddress(buf, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(buf, .readOnly) }
        guard let base = CVPixelBufferGetBaseAddress(buf) else { return nil }
        let w = CVPixelBufferGetWidth(buf), h = CVPixelBufferGetHeight(buf)
        let rb = CVPixelBufferGetBytesPerRow(buf)
        var px = [UInt8](repeating: 0, count: w * h * 4)
        for y in 0..<h {
            let row = base.advanced(by: y * rb).assumingMemoryBound(to: Float32.self)
            for x in 0..<w {
                let d = row[x]
                let o = (y * w + x) * 4
                guard d > 0, d.isFinite else { px[o + 3] = 255; continue }
                let t = min(1, d / maxMeters)
                let (r, g, b) = turbo(t)
                px[o] = r; px[o + 1] = g; px[o + 2] = b; px[o + 3] = 255
            }
        }
        return rgbaImage(px, width: w, height: h)
    }

    /// Cheap red -> yellow -> green -> cyan -> blue ramp.
    static func turbo(_ t: Float) -> (UInt8, UInt8, UInt8) {
        let r = max(0, min(1, 1.5 - abs(4 * t - 0.5)))
        let g = max(0, min(1, 1.5 - abs(4 * t - 1.75)))
        let b = max(0, min(1, 1.5 - abs(4 * t - 3.25)))
        return (UInt8(r * 255), UInt8(g * 255), UInt8(b * 255))
    }

    static func rgbaImage(_ px: [UInt8], width: Int, height: Int) -> CGImage? {
        let data = Data(px) as CFData
        guard let provider = CGDataProvider(data: data) else { return nil }
        return CGImage(width: width, height: height, bitsPerComponent: 8, bitsPerPixel: 32, bytesPerRow: width * 4,
                       space: CGColorSpaceCreateDeviceRGB(),
                       bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.premultipliedLast.rawValue),
                       provider: provider, decode: nil, shouldInterpolate: false, intent: .defaultIntent)
    }
}

enum MapImage {
    /// Top-down crop of the grid, `span` metres wide, centred on `center`. +x is right, -z is up.
    static func render(grid: OccupancyGrid, center: P2, span: Double) -> CGImage? {
        let n = max(8, Int(span / grid.cell))
        var px = [UInt8](repeating: 0, count: n * n * 4)
        let x0 = center.x - span / 2, z0 = center.z - span / 2
        for r in 0..<n {
            let z = z0 + (Double(r) + 0.5) * grid.cell
            for col in 0..<n {
                let x = x0 + (Double(col) + 0.5) * grid.cell
                let o = (r * n + col) * 4
                guard let (i, j) = grid.cellOf(P2(x, z)) else { continue }
                let k = j * grid.size + i
                let l = grid.logOdds[k]
                if grid.occupied(k) {
                    if grid.staticOcc[k] != 0 && l <= OccupancyGrid.occupiedAbove {
                        px[o] = 70; px[o + 1] = 140; px[o + 2] = 255; px[o + 3] = 255     // from the saved map
                    } else {
                        px[o] = 255; px[o + 1] = 80; px[o + 2] = 60; px[o + 3] = 255      // seen now
                    }
                } else if l < OccupancyGrid.freeBelow {
                    px[o] = 35; px[o + 1] = 75; px[o + 2] = 55; px[o + 3] = 230          // free
                } else if l != 0 {
                    px[o] = 50; px[o + 1] = 55; px[o + 2] = 60; px[o + 3] = 200
                } else {
                    px[o + 3] = 120                                                       // unknown
                }
            }
        }
        return DepthColormap.rgbaImage(px, width: n, height: n)
    }
}

// MARK: - voice, beeps, haptics

/// Spoken instructions plus a parking-sensor style proximity beep. Main thread only.
final class VoiceGuide {
    private let synth = AVSpeechSynthesizer()
    private var player: AVAudioPlayer?
    private var lastBeep: TimeInterval = 0
    private var lastSpeech: TimeInterval = 0

    init() {
        try? AVAudioSession.sharedInstance().setCategory(.playback, mode: .voicePrompt, options: [.duckOthers, .mixWithOthers])
        try? AVAudioSession.sharedInstance().setActive(true)
        player = try? AVAudioPlayer(data: VoiceGuide.beepWAV())
        player?.prepareToPlay()
    }

    func speak(_ text: String) {
        let now = ProcessInfo.processInfo.systemUptime
        // Obstacle warnings interrupt; other instructions wait for the current one.
        if text.hasPrefix("Obstacle") || text.hasPrefix("Tracking") {
            synth.stopSpeaking(at: .immediate)
        } else if synth.isSpeaking && now - lastSpeech < 4 {
            return
        }
        lastSpeech = now
        let u = AVSpeechUtterance(string: text)
        u.rate = 0.52
        synth.speak(u)
    }

    /// Call often (e.g. 20 Hz); beeps faster the closer `gap` is. No beep beyond `far`.
    func proximity(gap: Double, stop: Double, slow: Double, far: Double = 1.0) {
        guard gap.isFinite, gap < far else { return }
        let interval: Double = gap <= stop ? 0.12 : gap <= slow ? 0.3 : 0.6
        let now = ProcessInfo.processInfo.systemUptime
        guard now - lastBeep >= interval else { return }
        lastBeep = now
        player?.currentTime = 0
        player?.play()
    }

    func stopAll() {
        synth.stopSpeaking(at: .immediate)
        player?.stop()
    }

    /// 70 ms, 1.4 kHz sine with a short fade, 16-bit mono WAV.
    static func beepWAV() -> Data {
        let rate = 22050, n = rate * 70 / 1000
        var samples = Data(capacity: n * 2)
        for i in 0..<n {
            let env = min(1, Double(min(i, n - i)) / 200)
            var s = Int16(sin(2 * Double.pi * 1400 * Double(i) / Double(rate)) * 12000 * env).littleEndian
            samples.append(Data(bytes: &s, count: 2))
        }
        func u32(_ v: Int) -> Data { var x = UInt32(v).littleEndian; return Data(bytes: &x, count: 4) }
        func u16(_ v: Int) -> Data { var x = UInt16(v).littleEndian; return Data(bytes: &x, count: 2) }
        var d = Data("RIFF".utf8)
        d.append(u32(36 + samples.count))
        d.append(Data("WAVEfmt ".utf8))
        d.append(u32(16)); d.append(u16(1)); d.append(u16(1)); d.append(u32(rate)); d.append(u32(rate * 2))
        d.append(u16(2)); d.append(u16(16))
        d.append(Data("data".utf8))
        d.append(u32(samples.count))
        d.append(samples)
        return d
    }
}

enum Haptics {
    static func warning() {
        DispatchQueue.main.async { UINotificationFeedbackGenerator().notificationOccurred(.warning) }
    }
    static func success() {
        DispatchQueue.main.async { UINotificationFeedbackGenerator().notificationOccurred(.success) }
    }
}
