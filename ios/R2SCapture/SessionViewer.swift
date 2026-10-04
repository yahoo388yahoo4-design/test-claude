import AVFoundation
import Charts
import Compression
import QuickLook
import SceneKit
import SwiftUI
import UIKit

/// In-app playback of a recorded session (any mode):
/// * Video: the recording with a LiDAR depth overlay (modes A/B), or every camera side by side with a
///   depth inset (mode C).
/// * 3D: the ARKit mesh, the camera trajectory with a marker that follows playback, and the RoomPlan
///   room (mode B). AR Quick Look for room.usdz.
/// * Sensors: IMU, magnetometer, barometer, GPS, thermal/battery plots with a playback cursor.
/// * Info: session.json and the file list.
struct SessionViewer: View {
    let url: URL
    @StateObject private var data: SessionData
    @StateObject private var playback = Playback()
    @State private var tab = Tab.video

    enum Tab: String, CaseIterable, Identifiable {
        case video = "Video", scene = "3D", sensors = "Sensors", info = "Info"
        var id: String { rawValue }
    }

    init(url: URL) {
        self.url = url
        _data = StateObject(wrappedValue: SessionData(url: url))
    }

    var body: some View {
        VStack(spacing: 8) {
            Picker("View", selection: $tab) {
                ForEach(Tab.allCases) { Text($0.rawValue).tag($0) }
            }
            .pickerStyle(.segmented)
            .padding(.horizontal)
            Group {
                if !data.loaded {
                    ProgressView("Loading…").frame(maxHeight: .infinity)
                } else {
                    switch tab {
                    case .video: VideoTab(data: data, playback: playback)
                    case .scene: SceneTab(data: data, playback: playback)
                    case .sensors: SensorsTab(data: data, playback: playback)
                    case .info: InfoTab(data: data)
                    }
                }
            }
            .frame(maxHeight: .infinity)
            if data.loaded && !playback.players.isEmpty {
                TransportBar(playback: playback)
            }
        }
        .navigationTitle(url.lastPathComponent)
        .navigationBarTitleDisplayMode(.inline)
        .task {
            await data.load()
            playback.setUp(videos: data.videos)
        }
        .onDisappear { playback.pause() }
    }
}

// MARK: - session data

/// One depth record: byte range in a .zlib.bin blob, size, timestamp.
struct DepthRef {
    let t: Double
    let offset: UInt64
    let length: Int
    let w: Int
    let h: Int
}

struct CSVTable {
    var columns: [String] = []
    var rows: [[Double]] = []
    func column(_ name: String) -> [Double]? {
        guard let i = columns.firstIndex(of: name) else { return nil }
        return rows.map { i < $0.count ? $0[i] : .nan }
    }
}

final class SessionData: ObservableObject {
    let url: URL
    @Published private(set) var loaded = false
    private(set) var json: [String: Any] = [:]
    private(set) var mode = "?"
    /// Videos to play (mode A/B: one; mode C: one per camera), with their first frame time.
    private(set) var videos: [(name: String, url: URL, aspect: CGFloat)] = []
    /// Uptime of the first video frame: playback time 0.
    private(set) var t0: Double = 0
    private(set) var depth: [DepthRef] = []
    private(set) var depthBlob: URL?
    /// Camera positions (ARKit world) with times, for the trajectory.
    private(set) var poses: [(t: Double, p: SIMD3<Float>)] = []
    private(set) var tables: [String: CSVTable] = [:]
    private(set) var files: [(String, Int64)] = []
    private(set) var meshURL: URL?
    private(set) var roomURL: URL?

    init(url: URL) { self.url = url }

    @MainActor
    func load() async {
        let u = url
        let result = await Task.detached(priority: .userInitiated) { SessionData.read(u) }.value
        json = result.json
        mode = result.json["mode"] as? String ?? "?"
        videos = result.videos
        t0 = result.t0
        depth = result.depth
        depthBlob = result.depthBlob
        poses = result.poses
        tables = result.tables
        files = result.files
        meshURL = result.mesh
        roomURL = result.room
        loaded = true
    }

    private struct ReadResult {
        var json: [String: Any] = [:]
        var videos: [(name: String, url: URL, aspect: CGFloat)] = []
        var t0: Double = 0
        var depth: [DepthRef] = []
        var depthBlob: URL?
        var poses: [(t: Double, p: SIMD3<Float>)] = []
        var tables: [String: CSVTable] = [:]
        var files: [(String, Int64)] = []
        var mesh: URL?
        var room: URL?
    }

    private static func read(_ url: URL) -> ReadResult {
        var r = ReadResult()
        let fm = FileManager.default
        if let d = try? Data(contentsOf: url.appendingPathComponent("session.json")),
           let j = try? JSONSerialization.jsonObject(with: d) as? [String: Any] {
            r.json = j
        }
        let mode = r.json["mode"] as? String ?? ""

        if mode == "multicam" {
            let cams = url.appendingPathComponent("cams")
            let names = ((try? fm.contentsOfDirectory(atPath: cams.path)) ?? [])
                .filter { $0.hasSuffix(".mov") }.map { String($0.dropLast(4)) }.sorted()
            var firstT: Double?
            for n in names {
                let lines = jsonLines(cams.appendingPathComponent("\(n).jsonl"), limit: 1)
                var aspect: CGFloat = 4 / 3
                if let l = lines.first, let w = l["w"] as? Double, let h = l["h"] as? Double, h > 0 {
                    aspect = CGFloat(w / h)
                    if let t = l["t"] as? Double, firstT == nil { firstT = t }
                }
                r.videos.append((n, cams.appendingPathComponent("\(n).mov"), aspect))
            }
            r.t0 = firstT ?? 0
            for l in jsonLines(cams.appendingPathComponent("lidar_depth.jsonl")) {
                if let ref = depthRef(l, key: "d", wKey: "w", hKey: "h") { r.depth.append(ref) }
            }
            r.depthBlob = cams.appendingPathComponent("lidar_depth.zlib.bin")
        } else {
            let frames = jsonLines(url.appendingPathComponent("frames.jsonl"))
            var aspect: CGFloat = 4 / 3
            if let f = frames.first {
                r.t0 = f["t"] as? Double ?? 0
                if let w = f["w"] as? Double, let h = f["h"] as? Double, h > 0 { aspect = CGFloat(w / h) }
            }
            let video = url.appendingPathComponent("video.mov")
            if fm.fileExists(atPath: video.path) { r.videos.append(("video", video, aspect)) }
            for (k, f) in frames.enumerated() {
                if let ref = depthRef(f, key: "d", wKey: "dw", hKey: "dh") { r.depth.append(ref) }
                if k % 3 == 0, let T = f["T"] as? [Double], T.count == 16, let t = f["t"] as? Double {
                    r.poses.append((t, SIMD3(Float(T[3]), Float(T[7]), Float(T[11]))))
                }
            }
            r.depthBlob = url.appendingPathComponent("depth.zlib.bin")
        }

        for name in ["imu", "altimeter", "location", "status"] {
            let f = url.appendingPathComponent("\(name).csv")
            if fm.fileExists(atPath: f.path) { r.tables[name] = readCSV(f) }
        }
        let mesh = url.appendingPathComponent("mesh.ply")
        if fm.fileExists(atPath: mesh.path) { r.mesh = mesh }
        let room = url.appendingPathComponent("roomplan/room.usdz")
        if fm.fileExists(atPath: room.path) { r.room = room }

        if let e = fm.enumerator(at: url, includingPropertiesForKeys: [.fileSizeKey]) {
            for case let f as URL in e where !f.hasDirectoryPath {
                let size = Int64((try? f.resourceValues(forKeys: [.fileSizeKey]).fileSize) ?? 0)
                r.files.append((String(f.path.dropFirst(url.path.count + 1)), size))
            }
        }
        r.files.sort { $0.0 < $1.0 }
        return r
    }

    private static func depthRef(_ l: [String: Any], key: String, wKey: String, hKey: String) -> DepthRef? {
        guard let d = l[key] as? [Double], d.count == 2, let t = l["t"] as? Double,
              let w = l[wKey] as? Double, let h = l[hKey] as? Double else { return nil }
        return DepthRef(t: t, offset: UInt64(d[0]), length: Int(d[1]), w: Int(w), h: Int(h))
    }

    static func jsonLines(_ url: URL, limit: Int = .max) -> [[String: Any]] {
        guard let text = try? String(contentsOf: url, encoding: .utf8) else { return [] }
        var out: [[String: Any]] = []
        for line in text.split(separator: "\n") {
            if out.count >= limit { break }
            if let d = line.data(using: .utf8), let o = try? JSONSerialization.jsonObject(with: d) as? [String: Any] {
                out.append(o)
            }
        }
        return out
    }

    static func readCSV(_ url: URL) -> CSVTable {
        guard let text = try? String(contentsOf: url, encoding: .utf8) else { return CSVTable() }
        var lines = text.split(separator: "\n")
        guard !lines.isEmpty else { return CSVTable() }
        var t = CSVTable()
        t.columns = lines.removeFirst().split(separator: ",").map { $0.trimmingCharacters(in: .whitespaces) }
        t.rows.reserveCapacity(lines.count)
        for l in lines {
            t.rows.append(l.split(separator: ",", omittingEmptySubsequences: false).map { Double($0) ?? .nan })
        }
        return t
    }

    /// Nearest depth record to an uptime, decoded to millimetres.
    func depthImage(at t: Double) -> (CGImage, Int)? {
        guard !depth.isEmpty, let blob = depthBlob else { return nil }
        var lo = 0, hi = depth.count - 1
        while lo < hi {
            let mid = (lo + hi) / 2
            if depth[mid].t < t { lo = mid + 1 } else { hi = mid }
        }
        if lo > 0, abs(depth[lo - 1].t - t) < abs(depth[lo].t - t) { lo -= 1 }
        let ref = depth[lo]
        guard let h = try? FileHandle(forReadingFrom: blob) else { return nil }
        defer { try? h.close() }
        guard (try? h.seek(toOffset: ref.offset)) != nil,
              let packed = try? h.read(upToCount: ref.length),
              let raw = Inflate.raw(packed, expected: ref.w * ref.h * 2),
              let img = DepthColor.image(millimetres: raw, width: ref.w, height: ref.h) else { return nil }
        return (img, lo)
    }
}

enum Inflate {
    /// Raw deflate (zlib -15) of a known output size.
    static func raw(_ data: Data, expected: Int) -> Data? {
        guard !data.isEmpty, expected > 0 else { return nil }
        var out = Data(count: expected)
        let n = out.withUnsafeMutableBytes { (dst: UnsafeMutableRawBufferPointer) -> Int in
            data.withUnsafeBytes { (src: UnsafeRawBufferPointer) -> Int in
                guard let d = dst.bindMemory(to: UInt8.self).baseAddress,
                      let s = src.bindMemory(to: UInt8.self).baseAddress else { return 0 }
                return compression_decode_buffer(d, expected, s, data.count, nil, COMPRESSION_ZLIB)
            }
        }
        return n == expected ? out : nil
    }
}

extension DepthColor {
    /// uint16 millimetres (little-endian) -> colour image, same palette as the live inset.
    static func image(millimetres raw: Data, width w: Int, height h: Int, maxDepth: Float = 5) -> CGImage? {
        guard raw.count >= w * h * 2 else { return nil }
        var rgba = [UInt8](repeating: 255, count: w * h * 4)
        raw.withUnsafeBytes { (p: UnsafeRawBufferPointer) in
            let mm = p.bindMemory(to: UInt16.self)
            for i in 0..<(w * h) {
                let v = UInt16(littleEndian: mm[i])
                let j = i * 4
                if v == 0 { rgba[j] = 0; rgba[j + 1] = 0; rgba[j + 2] = 0; rgba[j + 3] = 0; continue }
                let t = min(Float(v) / 1000 / maxDepth, 1)
                rgba[j] = UInt8(255 * min(max(1.5 - abs(4 * t - 1), 0), 1))
                rgba[j + 1] = UInt8(255 * min(max(1.5 - abs(4 * t - 2), 0), 1))
                rgba[j + 2] = UInt8(255 * min(max(1.5 - abs(4 * t - 3), 0), 1))
            }
        }
        guard let provider = CGDataProvider(data: Data(rgba) as CFData) else { return nil }
        return CGImage(width: w, height: h, bitsPerComponent: 8, bitsPerPixel: 32, bytesPerRow: w * 4,
                       space: CGColorSpaceCreateDeviceRGB(),
                       bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.last.rawValue),
                       provider: provider, decode: nil, shouldInterpolate: true, intent: .defaultIntent)
    }
}

// MARK: - playback

final class Playback: ObservableObject {
    @Published private(set) var players: [AVPlayer] = []
    @Published var time: Double = 0
    @Published private(set) var duration: Double = 0
    @Published private(set) var isPlaying = false
    private var observer: Any?

    func setUp(videos: [(name: String, url: URL, aspect: CGFloat)]) {
        guard players.isEmpty else { return }
        players = videos.map { v in
            let p = AVPlayer(url: v.url)
            p.isMuted = true
            p.automaticallyWaitsToMinimizeStalling = false   // required by setRate(_:time:atHostTime:)
            p.actionAtItemEnd = .pause
            return p
        }
        guard let first = players.first else { return }
        observer = first.addPeriodicTimeObserver(forInterval: CMTime(value: 1, timescale: 15), queue: .main) { [weak self] t in
            guard let self = self else { return }
            self.time = t.seconds.isFinite ? t.seconds : 0
            if let d = first.currentItem?.duration.seconds, d.isFinite { self.duration = d }
            self.isPlaying = first.rate != 0
        }
    }

    func toggle() { isPlaying ? pause() : play() }

    func play() {
        if duration > 0, time >= duration - 0.05 { seek(0) }
        let now = CMClockGetTime(CMClockGetHostTimeClock()) + CMTime(value: 1, timescale: 10)
        for p in players { p.setRate(1, time: .invalid, atHostTime: now) }
        isPlaying = true
    }

    func pause() {
        for p in players { p.pause() }
        isPlaying = false
    }

    func seek(_ seconds: Double) {
        time = seconds
        let t = CMTime(seconds: seconds, preferredTimescale: 600)
        for p in players { p.seek(to: t, toleranceBefore: .zero, toleranceAfter: .zero) }
    }

    deinit {
        if let o = observer { players.first?.removeTimeObserver(o) }
    }
}

struct TransportBar: View {
    @ObservedObject var playback: Playback
    @State private var scrubbing = false
    @State private var scrub: Double = 0

    var body: some View {
        HStack(spacing: 12) {
            Button { playback.toggle() } label: {
                Image(systemName: playback.isPlaying ? "pause.fill" : "play.fill").font(.title2)
            }
            Slider(value: Binding(get: { scrubbing ? scrub : playback.time },
                                  set: { scrub = $0; playback.seek($0) }),
                   in: 0...max(playback.duration, 0.1),
                   onEditingChanged: { editing in
                       scrubbing = editing
                       if editing { playback.pause() }
                   })
            Text(String(format: "%.1f / %.1f s", playback.time, playback.duration))
                .font(.caption.monospacedDigit())
        }
        .padding(.horizontal)
        .padding(.bottom, 8)
    }
}

final class PlayerHostView: UIView {
    override class var layerClass: AnyClass { AVPlayerLayer.self }
    var playerLayer: AVPlayerLayer { layer as! AVPlayerLayer }
}

struct PlayerView: UIViewRepresentable {
    let player: AVPlayer
    func makeUIView(context: Context) -> PlayerHostView {
        let v = PlayerHostView()
        v.playerLayer.videoGravity = .resizeAspect
        v.playerLayer.player = player
        v.backgroundColor = .black
        return v
    }
    func updateUIView(_ uiView: PlayerHostView, context: Context) {
        if uiView.playerLayer.player !== player { uiView.playerLayer.player = player }
    }
}

// MARK: - video tab

struct VideoTab: View {
    @ObservedObject var data: SessionData
    @ObservedObject var playback: Playback
    @State private var showDepth = true
    @State private var depthOpacity = 0.5
    @State private var depthImage: CGImage?
    @State private var depthIndex = -1

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 10) {
                if data.videos.isEmpty {
                    Text("No video in this session.").foregroundStyle(.secondary)
                } else if data.mode == "multicam" {
                    LazyVGrid(columns: [GridItem(.flexible()), GridItem(.flexible())], spacing: 6) {
                        ForEach(Array(zip(data.videos.indices, playback.players)), id: \.0) { i, p in
                            VStack(spacing: 2) {
                                PlayerView(player: p).aspectRatio(data.videos[i].aspect, contentMode: .fit)
                                Text(data.videos[i].name).font(.caption2)
                            }
                        }
                        if let img = depthImage {
                            VStack(spacing: 2) {
                                Image(decorative: img, scale: 1).resizable().scaledToFit()
                                Text("lidar_depth").font(.caption2)
                            }
                        }
                    }
                } else if let p = playback.players.first {
                    ZStack {
                        PlayerView(player: p)
                        if showDepth, let img = depthImage {
                            Image(decorative: img, scale: 1).resizable().opacity(depthOpacity)
                        }
                    }
                    .aspectRatio(data.videos[0].aspect, contentMode: .fit)
                    if !data.depth.isEmpty {
                        Toggle("LiDAR depth overlay", isOn: $showDepth)
                        if showDepth {
                            HStack {
                                Text("Opacity").font(.caption)
                                Slider(value: $depthOpacity, in: 0...1)
                            }
                        }
                    }
                }
                LegendRow()
                if let i = depthIndex >= 0 ? depthIndex : nil {
                    Text("depth frame \(i + 1) / \(data.depth.count) · \(data.depth[i].w)×\(data.depth[i].h)")
                        .font(.caption.monospacedDigit()).foregroundStyle(.secondary)
                }
                if let p = data.poses.last(where: { $0.t <= data.t0 + playback.time }) {
                    Text(String(format: "camera position  x %.2f  y %.2f  z %.2f m", p.p.x, p.p.y, p.p.z))
                        .font(.caption.monospacedDigit()).foregroundStyle(.secondary)
                }
            }
            .padding(.horizontal)
        }
        .onAppear { updateDepth() }
        .onChange(of: playback.time) { _, _ in updateDepth() }
    }

    private func updateDepth() {
        guard !data.depth.isEmpty else { return }
        let t = data.t0 + playback.time
        let d = data
        Task.detached(priority: .userInitiated) {
            let r = d.depthImage(at: t)
            await MainActor.run {
                if let r = r, r.1 != depthIndex {
                    depthImage = r.0
                    depthIndex = r.1
                }
            }
        }
    }
}

struct LegendRow: View {
    var body: some View {
        HStack(spacing: 6) {
            Text("depth").font(.caption2)
            LinearGradient(colors: [.red, .yellow, .green, .cyan, .blue], startPoint: .leading, endPoint: .trailing)
                .frame(height: 6).clipShape(Capsule())
            Text("0 → 5 m").font(.caption2)
        }
    }
}

// MARK: - 3D tab

struct SceneTab: View {
    @ObservedObject var data: SessionData
    @ObservedObject var playback: Playback
    @State private var showQuickLook = false

    var body: some View {
        VStack(spacing: 6) {
            if data.meshURL == nil && data.roomURL == nil && data.poses.isEmpty {
                Text(data.mode == "multicam"
                     ? "MultiCam recordings have no camera poses or mesh. Run tools/recover_poses.py on a computer to get them."
                     : "No mesh, room or trajectory in this session.")
                    .foregroundStyle(.secondary).padding()
                Spacer()
            } else {
                SessionSceneView(data: data, time: data.t0 + playback.time)
                HStack {
                    Label("trajectory", systemImage: "point.topleft.down.curvedto.point.bottomright.up").foregroundStyle(.yellow)
                    Label("camera", systemImage: "circle.fill").foregroundStyle(.red)
                    if data.roomURL != nil {
                        Spacer()
                        Button("AR Quick Look") { showQuickLook = true }
                    }
                }
                .font(.caption)
                .padding(.horizontal)
                Text("Drag to orbit, pinch to zoom, two fingers to pan.").font(.caption2).foregroundStyle(.secondary)
            }
        }
        .sheet(isPresented: $showQuickLook) {
            if let u = data.roomURL { QuickLookView(url: u).ignoresSafeArea() }
        }
    }
}

struct SessionSceneView: UIViewRepresentable {
    let data: SessionData
    let time: Double

    final class Coordinator {
        let marker = SCNNode(geometry: SCNSphere(radius: 0.05))
        var built = false
    }

    func makeCoordinator() -> Coordinator { Coordinator() }

    func makeUIView(context: Context) -> SCNView {
        let v = SCNView()
        v.backgroundColor = .black
        v.allowsCameraControl = true
        v.autoenablesDefaultLighting = true
        v.antialiasingMode = .multisampling4X
        let scene = SCNScene()
        v.scene = scene
        let marker = context.coordinator.marker
        let m = SCNMaterial()
        m.diffuse.contents = UIColor.red
        m.lightingModel = .constant
        marker.geometry?.materials = [m]
        scene.rootNode.addChildNode(marker)
        let d = data
        DispatchQueue.global(qos: .userInitiated).async {
            let root = SceneBuilder.build(d)
            DispatchQueue.main.async {
                scene.rootNode.addChildNode(root)
                let (minB, maxB) = root.boundingBox
                let c = SCNVector3((minB.x + maxB.x) / 2, (minB.y + maxB.y) / 2, (minB.z + maxB.z) / 2)
                let size = max(maxB.x - minB.x, maxB.z - minB.z, 1)
                let cam = SCNNode()
                cam.camera = SCNCamera()
                cam.camera?.zFar = 500
                cam.position = SCNVector3(c.x, c.y + size * 0.9, c.z + size * 0.9)
                cam.look(at: c)
                scene.rootNode.addChildNode(cam)
                v.pointOfView = cam
            }
        }
        return v
    }

    func updateUIView(_ uiView: SCNView, context: Context) {
        if let p = data.poses.last(where: { $0.t <= time }) ?? data.poses.first {
            context.coordinator.marker.simdPosition = p.p
            context.coordinator.marker.isHidden = false
        } else {
            context.coordinator.marker.isHidden = true
        }
    }
}

enum SceneBuilder {
    static func build(_ data: SessionData) -> SCNNode {
        let root = SCNNode()
        if let u = data.meshURL, let g = mesh(u) { root.addChildNode(SCNNode(geometry: g)) }
        if data.poses.count > 1 {
            var idx: [Int32] = []
            for i in 0..<(data.poses.count - 1) { idx.append(Int32(i)); idx.append(Int32(i + 1)) }
            let src = SCNGeometrySource(vertices: data.poses.map { SCNVector3($0.p.x, $0.p.y, $0.p.z) })
            let g = SCNGeometry(sources: [src], elements: [SCNGeometryElement(indices: idx, primitiveType: .line)])
            let m = SCNMaterial()
            m.diffuse.contents = UIColor.yellow
            m.lightingModel = .constant
            g.materials = [m]
            root.addChildNode(SCNNode(geometry: g))
        }
        if let u = data.roomURL, let room = try? SCNScene(url: u, options: nil) {
            let n = SCNNode()
            for c in room.rootNode.childNodes { n.addChildNode(c) }
            n.enumerateHierarchy { node, _ in
                node.geometry?.materials.forEach { $0.transparency = 0.6 }
            }
            root.addChildNode(n)
        }
        return root
    }

    /// Colours for ARMeshClassification 0...7.
    static let classColors: [SIMD3<Float>] = [
        [0.6, 0.6, 0.6], [0.85, 0.75, 0.55], [0.35, 0.55, 0.85], [0.9, 0.9, 0.95],
        [0.85, 0.45, 0.25], [0.55, 0.8, 0.35], [0.4, 0.85, 0.9], [0.75, 0.35, 0.75],
    ]

    /// Reads the app's own binary PLY (see MeshExporter / FORMAT.md).
    static func mesh(_ url: URL) -> SCNGeometry? {
        guard let d = try? Data(contentsOf: url, options: .mappedIfSafe),
              let end = d.range(of: Data("end_header\n".utf8)) else { return nil }
        let header = String(decoding: d[..<end.lowerBound], as: UTF8.self)
        func count(_ el: String) -> Int {
            for l in header.split(separator: "\n") where l.hasPrefix("element \(el) ") {
                return Int(l.split(separator: " ").last ?? "") ?? 0
            }
            return 0
        }
        let nv = count("vertex"), nf = count("face")
        let body = end.upperBound
        guard nv > 0, nf > 0, d.count >= body + nv * 24 + nf * 14 else { return nil }
        var pos = [SIMD3<Float>](repeating: .zero, count: nv)
        var nor = [SIMD3<Float>](repeating: .zero, count: nv)
        var col = [SIMD3<Float>](repeating: classColors[0], count: nv)
        var idx = [Int32](); idx.reserveCapacity(nf * 3)
        d.withUnsafeBytes { (raw: UnsafeRawBufferPointer) in
            for i in 0..<nv {
                let o = body + i * 24
                func f(_ k: Int) -> Float { raw.loadUnaligned(fromByteOffset: o + 4 * k, as: Float.self) }
                pos[i] = SIMD3(f(0), f(1), f(2))
                nor[i] = SIMD3(f(3), f(4), f(5))
            }
            let fb = body + nv * 24
            for i in 0..<nf {
                let o = fb + i * 14
                let a = raw.loadUnaligned(fromByteOffset: o + 1, as: Int32.self)
                let b = raw.loadUnaligned(fromByteOffset: o + 5, as: Int32.self)
                let c = raw.loadUnaligned(fromByteOffset: o + 9, as: Int32.self)
                let cls = Int(raw[o + 13])
                guard a >= 0, b >= 0, c >= 0, Int(a) < nv, Int(b) < nv, Int(c) < nv else { continue }
                idx.append(a); idx.append(b); idx.append(c)
                let colour = classColors[min(cls, classColors.count - 1)]
                col[Int(a)] = colour; col[Int(b)] = colour; col[Int(c)] = colour
            }
        }
        let vs = SCNGeometrySource(vertices: pos.map { SCNVector3($0.x, $0.y, $0.z) })
        let ns = SCNGeometrySource(normals: nor.map { SCNVector3($0.x, $0.y, $0.z) })
        let cdata = col.withUnsafeBufferPointer { Data(buffer: $0) }
        let cs = SCNGeometrySource(data: cdata, semantic: .color, vectorCount: col.count, usesFloatComponents: true,
                                   componentsPerVector: 3, bytesPerComponent: 4, dataOffset: 0,
                                   dataStride: MemoryLayout<SIMD3<Float>>.stride)
        let g = SCNGeometry(sources: [vs, ns, cs], elements: [SCNGeometryElement(indices: idx, primitiveType: .triangles)])
        let m = SCNMaterial()
        m.isDoubleSided = true
        m.lightingModel = .lambert
        g.materials = [m]
        return g
    }
}

struct QuickLookView: UIViewControllerRepresentable {
    let url: URL
    func makeCoordinator() -> Coordinator { Coordinator(url: url) }
    func makeUIViewController(context: Context) -> QLPreviewController {
        let c = QLPreviewController()
        c.dataSource = context.coordinator
        return c
    }
    func updateUIViewController(_ uiViewController: QLPreviewController, context: Context) {}
    final class Coordinator: NSObject, QLPreviewControllerDataSource {
        let url: URL
        init(url: URL) { self.url = url }
        func numberOfPreviewItems(in controller: QLPreviewController) -> Int { 1 }
        func previewController(_ controller: QLPreviewController, previewItemAt index: Int) -> QLPreviewItem {
            url as NSURL
        }
    }
}

// MARK: - sensors tab

struct SensorsTab: View {
    @ObservedObject var data: SessionData
    @ObservedObject var playback: Playback

    private struct Plot: Identifiable {
        let id: String
        let title: String
        let table: String
        let columns: [String]
        let unit: String
    }

    private let plots: [Plot] = [
        Plot(id: "acc", title: "User acceleration", table: "imu", columns: ["ax", "ay", "az"], unit: "g"),
        Plot(id: "gyr", title: "Rotation rate", table: "imu", columns: ["gx", "gy", "gz"], unit: "rad/s"),
        Plot(id: "grav", title: "Gravity", table: "imu", columns: ["grx", "gry", "grz"], unit: "g"),
        Plot(id: "mag", title: "Magnetic field", table: "imu", columns: ["mx", "my", "mz"], unit: "µT"),
        Plot(id: "head", title: "Heading", table: "imu", columns: ["heading"], unit: "deg"),
        Plot(id: "alt", title: "Relative altitude", table: "altimeter", columns: ["rel_alt_m"], unit: "m"),
        Plot(id: "press", title: "Air pressure", table: "altimeter", columns: ["pressure_kpa"], unit: "kPa"),
        Plot(id: "speed", title: "GPS speed / accuracy", table: "location", columns: ["speed", "hacc"], unit: "m/s, m"),
        Plot(id: "therm", title: "Thermal state / battery", table: "status", columns: ["thermal", "battery"], unit: "0–3, 0–1"),
    ]

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 18) {
                let available = plots.filter { p in
                    guard let t = data.tables[p.table], !t.rows.isEmpty else { return false }
                    return p.columns.contains { t.columns.contains($0) }
                }
                if available.isEmpty {
                    Text("No sensor logs in this session.").foregroundStyle(.secondary)
                }
                ForEach(available) { p in
                    SensorChart(title: "\(p.title) (\(p.unit))", series: series(p), cursor: playback.time)
                }
                if let loc = data.tables["location"], let lat = loc.column("lat"), let lon = loc.column("lon"),
                   let la = lat.last(where: { $0.isFinite }), let lo = lon.last(where: { $0.isFinite }) {
                    Text(String(format: "Last GPS fix: %.6f, %.6f (%d fixes)", la, lo, loc.rows.count))
                        .font(.caption.monospacedDigit())
                }
            }
            .padding()
        }
    }

    /// (series name, [(time since first video frame, value)]), downsampled to <= 600 points.
    private func series(_ p: Plot) -> [(String, [(Double, Double)])] {
        guard let t = data.tables[p.table], let ts = t.column("t") else { return [] }
        let step = max(1, ts.count / 600)
        return p.columns.compactMap { c in
            guard let v = t.column(c) else { return nil }
            var pts: [(Double, Double)] = []
            var i = 0
            while i < ts.count {
                if ts[i].isFinite, v[i].isFinite { pts.append((ts[i] - data.t0, v[i])) }
                i += step
            }
            return (c, pts)
        }
    }
}

struct SensorChart: View {
    let title: String
    let series: [(String, [(Double, Double)])]
    let cursor: Double

    private struct Pt: Identifiable {
        let id: Int
        let s: String
        let x: Double
        let y: Double
    }

    var body: some View {
        let pts: [Pt] = series.enumerated().flatMap { (k, s) in
            s.1.enumerated().map { (i, p) in Pt(id: k * 1_000_000 + i, s: s.0, x: p.0, y: p.1) }
        }
        VStack(alignment: .leading, spacing: 4) {
            Text(title).font(.caption.weight(.semibold))
            Chart {
                ForEach(pts) { p in
                    LineMark(x: .value("t (s)", p.x), y: .value("value", p.y))
                        .foregroundStyle(by: .value("series", p.s))
                        .lineStyle(StrokeStyle(lineWidth: 1))
                }
                RuleMark(x: .value("now", cursor)).foregroundStyle(.white.opacity(0.6))
            }
            .chartLegend(position: .top, alignment: .leading)
            .frame(height: 140)
        }
    }
}

// MARK: - info tab

struct InfoTab: View {
    @ObservedObject var data: SessionData

    var body: some View {
        List {
            Section("session.json") {
                Text(pretty).font(.caption2.monospaced()).textSelection(.enabled)
            }
            Section("Files") {
                ForEach(data.files, id: \.0) { f in
                    HStack {
                        Text(f.0).font(.caption.monospaced())
                        Spacer()
                        Text(ByteCountFormatter.string(fromByteCount: f.1, countStyle: .file)).font(.caption)
                    }
                }
            }
        }
    }

    private var pretty: String {
        guard JSONSerialization.isValidJSONObject(data.json),
              let d = try? JSONSerialization.data(withJSONObject: data.json, options: [.prettyPrinted, .sortedKeys]) else {
            return "(no session.json)"
        }
        return String(decoding: d, as: UTF8.self)
    }
}
