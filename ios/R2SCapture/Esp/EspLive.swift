import ARKit
import SwiftUI
import UIKit

/// Marks the ESP32 sensing nodes in the scan while recording (modes A and B):
///  * automatically (mode A): the printed node tags (sensing/esp32/tags, bundled from Esp/Tags) are ARKit
///    reference images; every tracked sighting of tag N updates node N (median of the sightings);
///  * by tapping the ESP on screen: LiDAR depth under the tap, else an ARKit raycast; when neither hits,
///    the tap's ray is kept and two taps from different angles are intersected.
/// Results go to <session>/esp_nodes.json in the session's world frame. `observe` runs on the AR queue.
final class EspLive: ObservableObject {
    static let shared = EspLive()

    struct Mark: Identifiable {
        let id: Int
        let point: CGPoint
        let manual: Bool
        let distance: Float
    }

    @Published private(set) var marks: [Mark] = []
    @Published private(set) var placed: [Int: String] = [:]
    @Published var selected = 1
    @Published var marking = false
    @Published private(set) var message = "Film each ESP's tag for a second, or tap Mark ESP and tap the board."

    private let lock = NSLock()
    private var store: EspNodeStore?
    private weak var session: ARSession?
    private var taps: [(CGPoint, CGSize, Int)] = []
    private var viewport = CGSize(width: 390, height: 844)
    private var sightings: [Int: [SIMD3<Double>]] = [:]
    private var rays: [Int: [(SIMD3<Double>, SIMD3<Double>)]] = [:]
    private var lastPublish: TimeInterval = 0
    private var lastSave: TimeInterval = 0
    private var frameIndex = 0

    // MARK: configuration

    static let referenceImages: Set<ARReferenceImage> = {
        var width = EspMath.tagSizeM
        if let u = Bundle.main.url(forResource: "esp_tags", withExtension: "json"),
           let d = try? Data(contentsOf: u),
           let o = try? JSONSerialization.jsonObject(with: d) as? [String: Any],
           let w = o["tag_size_m"] as? Double { width = w }
        var out = Set<ARReferenceImage>()
        for i in 1...EspMath.nodeCount {
            guard let u = Bundle.main.url(forResource: "esp_\(i)", withExtension: "png"),
                  let img = UIImage(contentsOfFile: u.path)?.cgImage else { continue }
            let ref = ARReferenceImage(img, orientation: .up, physicalWidth: CGFloat(width))
            ref.name = "esp_\(i)"
            out.insert(ref)
        }
        return out
    }()

    /// Adds the tag images to a mode A configuration.
    static func configure(_ c: ARWorldTrackingConfiguration) {
        let refs = referenceImages
        guard !refs.isEmpty else { return }
        c.detectionImages = refs
        c.maximumNumberOfTrackedImages = 2
    }

    // MARK: recording lifecycle

    func begin(dir: URL, session: ARSession) {
        lock.lock()
        store = EspNodeStore(sessionDir: dir)
        self.session = session
        taps.removeAll()
        sightings.removeAll()
        rays.removeAll()
        frameIndex = 0
        lock.unlock()
        DispatchQueue.main.async { self.marks = []; self.placed = [:] }
    }

    func finish() {
        lock.lock()
        let s = store
        store = nil
        lock.unlock()
        if s?.dirty == true { s?.save() }
        DispatchQueue.main.async { self.marks = []; self.marking = false }
    }

    func setViewport(_ s: CGSize) { lock.lock(); viewport = s; lock.unlock() }

    func tap(at p: CGPoint, viewport size: CGSize) {
        lock.lock(); taps.append((p, size, selected)); lock.unlock()
    }

    // MARK: per frame (AR queue)

    func observe(_ frame: ARFrame) {
        lock.lock()
        guard let store = store else { lock.unlock(); return }
        let pending = taps
        taps.removeAll()
        let vp = viewport
        lock.unlock()
        frameIndex += 1
        var msg: String?

        for case let a as ARImageAnchor in frame.anchors where a.isTracked {
            guard let name = a.referenceImage.name, name.hasPrefix("esp_"), let id = Int(name.dropFirst(4)) else { continue }
            let c = a.transform.columns
            var list = sightings[id] ?? []
            list.append(SIMD3(Double(c.3.x), Double(c.3.y), Double(c.3.z)))
            if list.count > 60 { list.removeFirst() }
            sightings[id] = list
            let n = SIMD3(Double(c.1.x), Double(c.1.y), Double(c.1.z))   // the image's normal
            let node = EspNode(id: id, position: EspMath.median(list), normal: n, method: "auto_tag_live",
                               frameIndex: frameIndex, t: frame.timestamp, confidence: min(1, Double(list.count) / 10),
                               observations: list.count)
            if store.put(node), list.count == 1 { msg = "ESP \(id) found from its tag" }
        }

        for (p, size, id) in pending { msg = place(frame, p, size, id, store) }

        let now = CACurrentMediaTime()
        if store.dirty && now - lastSave > 1 { store.save(); lastSave = now }
        guard now - lastPublish > 0.08 || msg != nil else { return }
        lastPublish = now
        let toCam = frame.camera.transform.inverse
        var marks: [Mark] = []
        var placed: [Int: String] = [:]
        for n in store.all {
            placed[n.id] = n.method
            let w = SIMD3<Float>(Float(n.position.x), Float(n.position.y), Float(n.position.z))
            let pc = toCam * SIMD4<Float>(w, 1)
            guard pc.z < -0.05 else { continue }
            let pt = frame.camera.projectPoint(w, orientation: .portrait, viewportSize: vp)
            marks.append(Mark(id: n.id, point: pt, manual: EspMath.rank(n.method) >= 4, distance: -pc.z))
        }
        DispatchQueue.main.async {
            self.marks = marks
            self.placed = placed
            if let m = msg { self.message = m }
        }
    }

    private func place(_ frame: ARFrame, _ p: CGPoint, _ size: CGSize, _ id: Int, _ store: EspNodeStore) -> String {
        guard case .normal = frame.camera.trackingState else { return "Hold still: tracking is not ready" }
        // view point -> normalised image point (captured image, landscape)
        let toImage = frame.displayTransform(for: .portrait, viewportSize: size).inverted()
        let n = CGPoint(x: p.x / size.width, y: p.y / size.height).applying(toImage)
        let res = frame.camera.imageResolution
        let k = frame.camera.intrinsics
        let K = [Double(k[0][0]), Double(k[1][1]), Double(k[2][0]), Double(k[2][1])]
        let T = EspLive.rowMajor(frame.camera.transform)
        let u = Double(n.x) * Double(res.width), v = Double(n.y) * Double(res.height)
        var pos: SIMD3<Double>?
        var how = ""
        if let depth = (frame.sceneDepth ?? frame.smoothedSceneDepth)?.depthMap, let z = EspLive.depth(depth, nx: n.x, ny: n.y) {
            pos = EspMath.unproject(K: K, T: T, u: u, v: v, depth: z)
            how = String(format: "LiDAR, %.2f m away", z)
        } else if let s = session,
                  let hit = s.raycast(frame.raycastQuery(from: n, allowing: .estimatedPlane, alignment: .any)).first {
            let c = hit.worldTransform.columns.3
            pos = SIMD3(Double(c.x), Double(c.y), Double(c.z))
            how = "surface estimate"
        }
        if let pos = pos {
            store.put(EspNode(id: id, position: pos, normal: nil, method: "tap_live", frameIndex: frameIndex,
                              t: frame.timestamp, confidence: 0.8, observations: 1), force: true)
            rays[id] = nil
            DispatchQueue.main.async { if self.selected == id && id < EspMath.nodeCount { self.selected = id + 1 } }
            return "ESP \(id) placed (\(how))"
        }
        var list = rays[id] ?? []
        list.append(EspMath.ray(K: K, T: T, u: u, v: v))
        rays[id] = list
        if let tri = EspMath.triangulate(list), tri.2 >= 5, tri.1 < 0.1 {
            store.put(EspNode(id: id, position: tri.0, normal: nil, method: "triangulated", frameIndex: frameIndex,
                              t: frame.timestamp, confidence: exp(-tri.1 / 0.03), observations: list.count), force: true)
            return String(format: "ESP %d placed from %d taps (%.0f° apart)", id, list.count, tri.2)
        }
        return "ESP \(id): no surface there yet. Step to the side and tap it again"
    }

    /// Median of valid depth (m) in a 3x3 patch at a normalised image point.
    static func depth(_ buf: CVPixelBuffer, nx: CGFloat, ny: CGFloat) -> Double? {
        guard CVPixelBufferGetPixelFormatType(buf) == kCVPixelFormatType_DepthFloat32 else { return nil }
        CVPixelBufferLockBaseAddress(buf, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(buf, .readOnly) }
        let w = CVPixelBufferGetWidth(buf), h = CVPixelBufferGetHeight(buf)
        let row = CVPixelBufferGetBytesPerRow(buf)
        guard let base = CVPixelBufferGetBaseAddress(buf) else { return nil }
        let cx = Int((nx * CGFloat(w)).rounded()), cy = Int((ny * CGFloat(h)).rounded())
        guard cx >= 0, cx < w, cy >= 0, cy < h else { return nil }
        var vals: [Float] = []
        for y in max(0, cy - 1)...min(h - 1, cy + 1) {
            let r = base.advanced(by: y * row).assumingMemoryBound(to: Float32.self)
            for x in max(0, cx - 1)...min(w - 1, cx + 1) where r[x].isFinite && r[x] > 0.05 { vals.append(r[x]) }
        }
        guard !vals.isEmpty else { return nil }
        vals.sort()
        return Double(vals[vals.count / 2])
    }

    static func rowMajor(_ m: simd_float4x4) -> [Double] {
        var out = [Double]()
        for r in 0..<4 { for c in 0..<4 { out.append(Double(m[c][r])) } }
        return out
    }
}

/// Live overlay over the AR preview while recording: node buttons, rings on placed nodes, tap-to-place.
struct EspOverlay: View {
    @ObservedObject var live = EspLive.shared

    var body: some View {
        GeometryReader { geo in
            ZStack(alignment: .top) {
                if live.marking {
                    Color.clear
                        .contentShape(Rectangle())
                        .gesture(SpatialTapGesture().onEnded { v in live.tap(at: v.location, viewport: geo.size) })
                }
                ForEach(live.marks) { m in
                    let r = CGFloat(min(70, max(16, 40 / max(0.3, m.distance))))
                    ZStack {
                        Circle().stroke(m.manual ? Color.green : Color.cyan, lineWidth: 3)
                            .frame(width: 2 * r, height: 2 * r)
                            .position(m.point)
                        Text("ESP \(m.id)").font(.caption.bold()).foregroundStyle(.white).shadow(radius: 3)
                            .position(x: m.point.x + r + 28, y: m.point.y)
                    }
                    .allowsHitTesting(false)
                }
                VStack(alignment: .leading, spacing: 6) {
                    ScrollView(.horizontal, showsIndicators: false) {
                        HStack(spacing: 6) {
                            Button(live.marking ? "Tap ESP \(live.selected)" : "Mark ESP") { live.marking.toggle() }
                                .buttonStyle(.borderedProminent).tint(live.marking ? .orange : .gray)
                            ForEach(1...EspMath.nodeCount, id: \.self) { i in
                                Button("\(i)") { live.selected = i; live.marking = true }
                                    .buttonStyle(.borderedProminent)
                                    .tint(color(i))
                            }
                        }
                    }
                    Text(live.message).font(.caption).foregroundStyle(.white).shadow(radius: 3)
                }
                .padding(.horizontal, 12)
                .padding(.top, 120)
            }
            .onAppear { live.setViewport(geo.size) }
            .onChange(of: geo.size) { _, s in live.setViewport(s) }
        }
    }

    private func color(_ i: Int) -> Color {
        if live.marking && live.selected == i { return .orange }
        guard let m = live.placed[i] else { return Color(white: 0.3) }
        return EspMath.rank(m) >= 4 ? .green : .blue
    }
}
