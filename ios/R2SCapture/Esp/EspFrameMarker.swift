import AVFoundation
import SwiftUI

/// Tap the ESP32 board in a recorded video frame to put it into the scan (sensing/esp32/README.md).
/// With LiDAR depth at that frame one tap is enough; without depth, tap it in two frames seen from
/// different angles and the rays are intersected. Writes <session>/esp_nodes.json; placed nodes are
/// drawn back into every frame so you can scrub and check them.
struct EspFrameMarker: View {
    @StateObject private var m: EspFrameModel
    @State private var zoom: CGFloat = 1
    @State private var center = CGPoint(x: 0.5, y: 0.5)
    @State private var dragStart: CGPoint?

    init(url: URL) { _m = StateObject(wrappedValue: EspFrameModel(url: url)) }

    var body: some View {
        VStack(spacing: 8) {
            ScrollView(.horizontal, showsIndicators: false) {
                HStack(spacing: 6) {
                    ForEach(1...EspMath.nodeCount, id: \.self) { i in
                        Button("ESP \(i)") { m.selected = i }
                            .buttonStyle(.borderedProminent).tint(color(i))
                    }
                    Button("Clear") { m.clearSelected() }.buttonStyle(.bordered)
                }
                .padding(.horizontal)
            }
            GeometryReader { geo in
                let iw = CGFloat(max(1, m.imageSize.width)), ih = CGFloat(max(1, m.imageSize.height))
                let s = min(geo.size.width / iw, geo.size.height / ih) * zoom
                let ox = geo.size.width / 2 - center.x * iw * s, oy = geo.size.height / 2 - center.y * ih * s
                ZStack(alignment: .topLeading) {
                    Color.black
                    if let img = m.image {
                        Image(decorative: img, scale: 1).resizable()
                            .frame(width: iw * s, height: ih * s)
                            .position(x: ox + iw * s / 2, y: oy + ih * s / 2)
                    }
                    ForEach(m.marks) { mk in
                        let p = CGPoint(x: ox + mk.point.x * s, y: oy + mk.point.y * s)
                        Circle().stroke(mk.manual ? Color.green : Color.cyan, lineWidth: 3).frame(width: 44, height: 44).position(p)
                        Text("ESP \(mk.id)").font(.caption.bold()).foregroundStyle(.white).shadow(radius: 3)
                            .position(x: p.x + 50, y: p.y)
                    }
                }
                .clipped()
                .contentShape(Rectangle())
                .gesture(DragGesture(minimumDistance: 8)
                    .onChanged { v in
                        guard zoom > 1 else { return }
                        let start = dragStart ?? center
                        if dragStart == nil { dragStart = center }
                        center = CGPoint(x: min(1, max(0, start.x - v.translation.width / (iw * s))),
                                         y: min(1, max(0, start.y - v.translation.height / (ih * s))))
                    }
                    .onEnded { _ in dragStart = nil })
                .simultaneousGesture(SpatialTapGesture().onEnded { v in
                    let u = (v.location.x - ox) / s, vv = (v.location.y - oy) / s
                    if u >= 0, u < iw, vv >= 0, vv < ih { m.tap(u: Double(u), v: Double(vv)) }
                })
            }
            if m.count > 1 {
                Slider(value: Binding(get: { Double(m.index) }, set: { m.show(Int($0.rounded())) }),
                       in: 0...Double(m.count - 1))
                    .padding(.horizontal)
            }
            HStack(spacing: 10) {
                ForEach([-30, -1, 1, 30], id: \.self) { d in
                    Button(d > 0 ? "+\(d)" : "\(d)") { m.show(m.index + d) }.buttonStyle(.bordered)
                }
                Button("Zoom \(Int(zoom))×") {
                    zoom = zoom >= 4 ? 1 : zoom * 2
                    if zoom == 1 { center = CGPoint(x: 0.5, y: 0.5) }
                }.buttonStyle(.bordered)
            }
            Text(m.status).font(.caption.monospacedDigit()).foregroundStyle(.secondary)
                .frame(maxWidth: .infinity, alignment: .leading).padding(.horizontal)
        }
        .navigationTitle("ESP nodes")
        .navigationBarTitleDisplayMode(.inline)
        .task { await m.load() }
    }

    private func color(_ i: Int) -> Color {
        if m.selected == i { return .orange }
        guard let n = m.placed[i] else { return Color(white: 0.3) }
        return EspMath.rank(n) >= 4 ? .green : .blue
    }
}

@MainActor
final class EspFrameModel: ObservableObject {
    struct F {
        let i: Int, t: Double, w: Int, h: Int
        let K: [Double], T: [Double]?
        let d: [Int]?, dw: Int, dh: Int
    }

    let url: URL
    @Published var index = 0
    @Published var image: CGImage?
    @Published var imageSize = CGSize(width: 1920, height: 1440)
    @Published var selected = 1 { didSet { refresh() } }
    @Published var status = "Loading…"
    struct Mark: Identifiable { let id: Int; let point: CGPoint; let manual: Bool }
    @Published var marks: [Mark] = []
    @Published var placed: [Int: String] = [:]
    private(set) var count = 0
    private var frames: [F] = []
    private var t0 = 0.0
    private var generator: AVAssetImageGenerator?
    private var store: EspNodeStore
    private var rays: [Int: [(SIMD3<Double>, SIMD3<Double>)]] = [:]
    private var loading = false
    private var queued: Int?

    init(url: URL) {
        self.url = url
        store = EspNodeStore(sessionDir: url)
    }

    func load() async {
        let u = url
        let lines = await Task.detached { SessionData.jsonLines(u.appendingPathComponent("frames.jsonl")) }.value
        frames = lines.enumerated().compactMap { k, l -> F? in
            guard let t = l["t"] as? Double, let K = l["K"] as? [Double], K.count >= 4 else { return nil }
            let T = (l["T"] as? [Double]).flatMap { $0.count == 16 ? $0 : nil }
            return F(i: l["i"] as? Int ?? k, t: t, w: l["w"] as? Int ?? 1920, h: l["h"] as? Int ?? 1440, K: K, T: T,
                     d: l["d"] as? [Int], dw: l["dw"] as? Int ?? 0, dh: l["dh"] as? Int ?? 0)
        }
        count = frames.count
        let video = url.appendingPathComponent("video.mov")
        guard !frames.isEmpty, frames.contains(where: { $0.T != nil }), FileManager.default.fileExists(atPath: video.path) else {
            status = "This session has no video with camera poses (record in mode A or B to mark ESPs)."
            return
        }
        t0 = frames[0].t
        let g = AVAssetImageGenerator(asset: AVURLAsset(url: video))
        g.appliesPreferredTrackTransform = false      // keep the sensor's landscape orientation, like K
        g.requestedTimeToleranceBefore = .zero
        g.requestedTimeToleranceAfter = .zero
        generator = g
        let start = store.node(selected)?.frameIndex.map { min(max(0, $0), count - 1) } ?? count / 2
        show(start)
    }

    func show(_ k: Int) {
        guard !frames.isEmpty else { return }
        let k = min(max(0, k), frames.count - 1)
        if loading { queued = k; return }
        loading = true
        index = k
        let f = frames[k]
        let time = CMTime(seconds: f.t - t0, preferredTimescale: 600_000)
        let g = generator
        Task.detached {
            let img = try? await g?.image(at: time).image
            await MainActor.run {
                self.loading = false
                if let img = img { self.image = img }
                self.imageSize = CGSize(width: f.w, height: f.h)
                self.refresh()
                if let q = self.queued, q != k { self.queued = nil; self.show(q) } else { self.queued = nil }
            }
        }
    }

    func refresh() {
        let all = store.all
        placed = Dictionary(uniqueKeysWithValues: all.map { ($0.id, $0.method) })
        guard index < frames.count else { return }
        let f = frames[index]
        if let T = f.T {
            marks = all.compactMap { n in
                EspMath.project(K: f.K, T: T, p: n.position).map { Mark(id: n.id, point: CGPoint(x: $0.x, y: $0.y), manual: EspMath.rank(n.method) >= 4) }
            }
        } else { marks = [] }
        var s = String(format: "frame %d / %d  t=%.2f s%@\nTap ESP %d in the picture. ", index, frames.count - 1, f.t - t0,
                       f.T == nil ? "  (no pose)" : "", selected)
        if let n = store.node(selected) {
            s += String(format: "Placed (%@): %.2f, %.2f, %.2f m. ", n.method, n.position.x, n.position.y, n.position.z)
        }
        if let r = rays[selected], !r.isEmpty { s += "Views so far: \(r.count). " }
        status = s
    }

    func clearSelected() {
        store.remove(selected)
        rays[selected] = nil
        store.save()
        refresh()
    }

    /// (u, v) in native image pixels.
    func tap(u: Double, v: Double) {
        guard index < frames.count else { return }
        let f = frames[index]
        guard let T = f.T else { status = "No camera pose in this frame: pick another one."; return }
        let id = selected
        var msg: String
        if let z = depthAt(f, u: u, v: v) {
            store.put(EspNode(id: id, position: EspMath.unproject(K: f.K, T: T, u: u, v: v, depth: z), normal: nil,
                              method: "tap_frame", frameIndex: index, t: f.t, confidence: 0.8, observations: 1), force: true)
            rays[id] = nil
            msg = String(format: "ESP %d placed %.2f m from the camera.", id, z)
        } else {
            var list = rays[id] ?? []
            list.append(EspMath.ray(K: f.K, T: T, u: u, v: v))
            rays[id] = list
            if let tri = EspMath.triangulate(list), tri.2 >= 5, tri.1 < 0.1 {
                store.put(EspNode(id: id, position: tri.0, normal: nil, method: "triangulated", frameIndex: index, t: f.t,
                                  confidence: exp(-tri.1 / 0.03), observations: list.count), force: true)
                msg = String(format: "ESP %d placed from %d views (%.0f° apart).", id, list.count, tri.2)
            } else {
                msg = "No depth here: scrub to a frame that sees ESP \(id) from another angle and tap it again."
            }
        }
        store.save()
        refresh()
        status = msg + "\n" + status
    }

    private func depthAt(_ f: F, u: Double, v: Double) -> Double? {
        guard let r = f.d, r.count == 2, f.dw > 0, f.dh > 0,
              let h = try? FileHandle(forReadingFrom: url.appendingPathComponent("depth.zlib.bin")) else { return nil }
        defer { try? h.close() }
        guard (try? h.seek(toOffset: UInt64(r[0]))) != nil, let packed = try? h.read(upToCount: r[1]),
              let raw = Inflate.raw(packed, expected: f.dw * f.dh * 2) else { return nil }
        let mm = raw.withUnsafeBytes { Array($0.bindMemory(to: UInt16.self)) }.map { UInt16(littleEndian: $0) }
        return EspMath.depthAt(mm: mm, dw: f.dw, dh: f.dh, w: f.w, h: f.h, u: u, v: v)
    }
}
