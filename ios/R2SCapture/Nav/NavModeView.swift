import ARKit
import SceneKit
import SwiftUI
import UIKit

/// Full-screen navigation mode: AR view with the path, goal and nearest obstacles drawn on the floor,
/// HUD (instruction, speed, distances, LiDAR depth, radar, minimap), drive controls and settings.
struct NavModeView: View {
    @StateObject private var engine = NavEngine()
    @Environment(\.dismiss) private var dismiss
    @State private var showSettings = false
    @State private var showManual = true

    var body: some View {
        ZStack {
            NavARView(engine: engine).ignoresSafeArea()
            VStack(spacing: 6) {
                NavTopBar(engine: engine, link: engine.link, close: { dismiss() }, settings: { showSettings = true })
                InstructionBanner(hud: engine.hud)
                HStack(alignment: .top) {
                    VStack(alignment: .leading, spacing: 6) {
                        SpeedGauge(hud: engine.hud, maxSpeed: engine.settings.maxSpeed)
                        TurnRateBar(rate: engine.hud.turnRate, cmd: deg(engine.hud.cmd.w), maxRate: engine.settings.maxTurnRateDeg)
                        SensorStats(hud: engine.hud)
                    }
                    Spacer()
                    DepthInset(image: engine.hud.depthImage)
                }
                .padding(.horizontal, 10)
                Spacer()
                if !engine.toast.isEmpty {
                    Text(engine.toast)
                        .font(.caption)
                        .padding(6)
                        .background(.ultraThinMaterial, in: Capsule())
                        .onTapGesture { engine.toast = "" }
                }
                BottomPanel(engine: engine, showManual: $showManual)
            }
        }
        .preferredColorScheme(.dark)
        .onAppear { engine.start() }
        .onDisappear { engine.shutdown() }
        .sheet(isPresented: $showSettings) {
            NavSettingsView(engine: engine)
        }
    }
}

// MARK: - AR view with SceneKit overlays

struct NavARView: UIViewRepresentable {
    @ObservedObject var engine: NavEngine

    func makeCoordinator() -> Coordinator { Coordinator(engine: engine) }

    func makeUIView(context: Context) -> ARSCNView {
        let v = ARSCNView(frame: .zero)
        v.session = engine.session
        v.automaticallyUpdatesLighting = true
        v.rendersCameraGrain = false
        v.scene.rootNode.addChildNode(context.coordinator.root)
        let tap = UITapGestureRecognizer(target: context.coordinator, action: #selector(Coordinator.tapped(_:)))
        v.addGestureRecognizer(tap)
        context.coordinator.view = v
        return v
    }

    func updateUIView(_ uiView: ARSCNView, context: Context) {
        context.coordinator.update(hud: engine.hud)
    }

    final class Coordinator: NSObject {
        let engine: NavEngine
        weak var view: ARSCNView?
        let root = SCNNode()
        private let pathNode = SCNNode()
        private let goalNode = SCNNode()
        private let carrotNode = SCNNode()
        private var markerNodes: [SCNNode] = []
        private var pathVersion = -1

        init(engine: NavEngine) {
            self.engine = engine
            super.init()
            root.addChildNode(pathNode)
            // Goal: a green pole with a ball on top.
            let pole = SCNNode(geometry: SCNCylinder(radius: 0.015, height: 0.6))
            pole.geometry?.firstMaterial?.diffuse.contents = UIColor.systemGreen
            pole.geometry?.firstMaterial?.lightingModel = .constant
            pole.position = SCNVector3(0, 0.3, 0)
            let ball = SCNNode(geometry: SCNSphere(radius: 0.06))
            ball.geometry?.firstMaterial?.diffuse.contents = UIColor.systemGreen
            ball.geometry?.firstMaterial?.lightingModel = .constant
            ball.position = SCNVector3(0, 0.62, 0)
            let ring = SCNNode(geometry: SCNTorus(ringRadius: 0.15, pipeRadius: 0.012))
            ring.geometry?.firstMaterial?.diffuse.contents = UIColor.systemGreen
            ring.geometry?.firstMaterial?.lightingModel = .constant
            goalNode.addChildNode(pole)
            goalNode.addChildNode(ball)
            goalNode.addChildNode(ring)
            goalNode.isHidden = true
            root.addChildNode(goalNode)
            let carrot = SCNSphere(radius: 0.04)
            carrot.firstMaterial?.diffuse.contents = UIColor.systemYellow
            carrot.firstMaterial?.lightingModel = .constant
            carrotNode.geometry = carrot
            carrotNode.isHidden = true
            root.addChildNode(carrotNode)
            for _ in NavEngine.sectorBearings {
                let s = SCNNode(geometry: SCNSphere(radius: 0.035))
                s.geometry?.firstMaterial?.lightingModel = .constant
                s.isHidden = true
                root.addChildNode(s)
                markerNodes.append(s)
            }
        }

        @objc func tapped(_ g: UITapGestureRecognizer) {
            guard let v = view else { return }
            if let p = engine.floorPoint(in: v, at: g.location(in: v)) {
                engine.setGoal(p, label: "tapped point")
            }
        }

        func update(hud: NavHUD) {
            let fy = hud.floorY + 0.01
            if let g = hud.goal {
                goalNode.isHidden = false
                goalNode.position = SCNVector3(Float(g.x), hud.floorY, Float(g.z))
            } else {
                goalNode.isHidden = true
            }
            if let c = hud.lookahead {
                carrotNode.isHidden = false
                carrotNode.position = SCNVector3(Float(c.x), fy + 0.03, Float(c.z))
            } else {
                carrotNode.isHidden = true
            }
            if hud.pathVersion != pathVersion {
                pathVersion = hud.pathVersion
                pathNode.childNodes.forEach { $0.removeFromParentNode() }
                let pts = hud.path
                if pts.count >= 2 {
                    for k in 0..<(pts.count - 1) {
                        pathNode.addChildNode(Coordinator.segment(pts[k], pts[k + 1], y: fy))
                    }
                    // Chevrons every 30 cm show the direction of travel.
                    var acc = 0.0
                    for k in 0..<(pts.count - 1) {
                        let a = pts[k], b = pts[k + 1]
                        let L = a.distance(to: b)
                        var s = 0.3 - acc
                        while s < L {
                            let q = a + (b - a) * (s / L)
                            pathNode.addChildNode(Coordinator.chevron(at: q, heading: Pose2.heading(from: a, to: b), y: fy + 0.005))
                            s += 0.3
                        }
                        acc = (acc + L).truncatingRemainder(dividingBy: 0.3)
                    }
                }
            }
            for (k, node) in markerNodes.enumerated() {
                if k < hud.markers.count, let m = hud.markers[k], hud.sectors[k] < 1.5 {
                    node.isHidden = false
                    node.position = SCNVector3(m.x, m.y, m.z)
                    node.geometry?.firstMaterial?.diffuse.contents = UIColor(gapColor(hud.sectors[k], stop: engine.settings.stopDistance, slow: engine.settings.slowDistance))
                } else {
                    node.isHidden = true
                }
            }
        }

        static func segment(_ a: P2, _ b: P2, y: Float) -> SCNNode {
            let L = a.distance(to: b)
            let box = SCNBox(width: CGFloat(L), height: 0.004, length: 0.06, chamferRadius: 0)
            box.firstMaterial?.diffuse.contents = UIColor.systemCyan.withAlphaComponent(0.8)
            box.firstMaterial?.lightingModel = .constant
            let n = SCNNode(geometry: box)
            n.position = SCNVector3(Float((a.x + b.x) / 2), y, Float((a.z + b.z) / 2))
            // Box width runs along local +x; rotate about +y by the travel heading (CCW from +x towards -z).
            n.eulerAngles = SCNVector3(0, Float(Pose2.heading(from: a, to: b)), 0)
            return n
        }

        static func chevron(at p: P2, heading: Double, y: Float) -> SCNNode {
            let path = UIBezierPath()
            path.move(to: CGPoint(x: -0.05, y: -0.05))
            path.addLine(to: CGPoint(x: 0.05, y: 0))
            path.addLine(to: CGPoint(x: -0.05, y: 0.05))
            path.addLine(to: CGPoint(x: -0.03, y: 0))
            path.close()
            let shape = SCNShape(path: path, extrusionDepth: 0.003)
            shape.firstMaterial?.diffuse.contents = UIColor.white
            shape.firstMaterial?.lightingModel = .constant
            // SCNShape lies in its local x/y plane: lay it flat (x stays forward, tip at +x), then turn the
            // parent to the travel heading (rotation about +y, CCW from +x towards -z).
            let flat = SCNNode(geometry: shape)
            flat.eulerAngles = SCNVector3(-Float.pi / 2, 0, 0)
            let n = SCNNode()
            n.addChildNode(flat)
            n.eulerAngles = SCNVector3(0, Float(heading), 0)
            n.position = SCNVector3(Float(p.x), y, Float(p.z))
            return n
        }
    }
}

func gapColor(_ gap: Double, stop: Double, slow: Double) -> Color {
    if gap <= stop { return .red }
    if gap <= slow { return .orange }
    if gap <= slow + 0.6 { return .yellow }
    return .green
}

func fmtDist(_ m: Double) -> String {
    guard m.isFinite else { return "–" }
    return m < 1 ? String(format: "%.0f cm", m * 100) : String(format: "%.2f m", m)
}

// MARK: - top bar and banner

struct NavTopBar: View {
    @ObservedObject var engine: NavEngine
    @ObservedObject var link: RobotLink
    let close: () -> Void
    let settings: () -> Void

    var body: some View {
        HStack(spacing: 8) {
            Button(action: close) { Image(systemName: "xmark.circle.fill").font(.title2) }
            VStack(alignment: .leading, spacing: 2) {
                HStack(spacing: 6) {
                    Circle().fill(engine.hud.trackingOK ? Color.green : Color.orange).frame(width: 8, height: 8)
                    Text(engine.hud.tracking).font(.caption2.weight(.semibold)).lineLimit(1)
                }
                Text(engine.hud.mapStatus).font(.caption2).foregroundStyle(.secondary).lineLimit(1)
                HStack(spacing: 6) {
                    Circle().fill(linkColor).frame(width: 8, height: 8)
                    Text(link.state.label).font(.caption2).lineLimit(1)
                    if let rtt = link.rttMs { Text(String(format: "%.0f ms", rtt)).font(.caption2.monospacedDigit()) }
                    if !link.robotStatus.isEmpty { Text(link.robotStatus).font(.caption2).lineLimit(1) }
                }
            }
            Spacer()
            Button(action: settings) { Image(systemName: "gearshape.fill").font(.title2) }
        }
        .padding(8)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12))
        .padding(.horizontal, 10)
        .foregroundStyle(.white)
    }

    private var linkColor: Color {
        switch link.state {
        case .connected: return .green
        case .connecting: return .yellow
        case .failed: return .red
        case .off: return .gray
        }
    }
}

struct InstructionBanner: View {
    let hud: NavHUD

    var body: some View {
        HStack(spacing: 12) {
            Image(systemName: hud.goal == nil ? "mappin.and.ellipse" : "arrow.up")
                .font(.system(size: 34, weight: .bold))
                .rotationEffect(.degrees(hud.goal == nil ? 0 : -hud.headingError))
                .foregroundStyle(hud.state == "Blocked" ? Color.red : Color.cyan)
                .frame(width: 44)
            VStack(alignment: .leading, spacing: 2) {
                Text(hud.instruction).font(.headline).lineLimit(2).minimumScaleFactor(0.7)
                HStack(spacing: 10) {
                    Text(hud.state).font(.caption.weight(.semibold))
                    if hud.goal != nil {
                        Text("goal \(fmtDist(hud.remaining))").font(.caption.monospacedDigit())
                        Text(String(format: "ETA %.0f s", hud.eta)).font(.caption.monospacedDigit())
                        Text(String(format: "%+.0f°", hud.headingError)).font(.caption.monospacedDigit())
                    }
                    if !hud.goalLabel.isEmpty { Text(hud.goalLabel).font(.caption).foregroundStyle(.secondary) }
                }
            }
            Spacer(minLength: 0)
        }
        .padding(8)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12))
        .padding(.horizontal, 10)
    }
}

// MARK: - gauges

struct SpeedGauge: View {
    let hud: NavHUD
    let maxSpeed: Double

    var body: some View {
        let frac = min(1, abs(hud.speed) / max(0.05, maxSpeed))
        let cmdFrac = min(1, abs(hud.cmd.v) / max(0.05, maxSpeed))
        ZStack {
            Circle().trim(from: 0.15, to: 0.85).stroke(Color.white.opacity(0.2), style: StrokeStyle(lineWidth: 8, lineCap: .round))
                .rotationEffect(.degrees(90))
            Circle().trim(from: 0.15, to: 0.15 + 0.7 * frac)
                .stroke(hud.speed < 0 ? Color.orange : Color.cyan, style: StrokeStyle(lineWidth: 8, lineCap: .round))
                .rotationEffect(.degrees(90))
            // Commanded speed tick.
            Rectangle().fill(Color.white).frame(width: 3, height: 12).offset(y: -34)
                .rotationEffect(.degrees(-126 + 252 * cmdFrac))
            VStack(spacing: 0) {
                Text(String(format: "%.2f", hud.speed)).font(.system(size: 16, weight: .bold).monospacedDigit())
                Text("m/s").font(.system(size: 9))
                Text(String(format: "cmd %.2f", hud.cmd.v)).font(.system(size: 9).monospacedDigit()).foregroundStyle(.secondary)
            }
        }
        .frame(width: 84, height: 84)
        .padding(4)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12))
    }
}

struct TurnRateBar: View {
    let rate: Double       // deg/s, + left
    let cmd: Double
    let maxRate: Double

    var body: some View {
        VStack(spacing: 2) {
            GeometryReader { geo in
                let w = geo.size.width
                let f = max(-1, min(1, rate / max(1, maxRate)))
                let c = max(-1, min(1, cmd / max(1, maxRate)))
                ZStack(alignment: .center) {
                    Capsule().fill(Color.white.opacity(0.2))
                    Capsule().fill(Color.purple)
                        .frame(width: abs(f) * w / 2)
                        .offset(x: -f * w / 4)
                    Rectangle().fill(Color.white).frame(width: 2).offset(x: -c * w / 2)
                }
            }
            .frame(height: 8)
            Text(String(format: "turn %+.0f°/s", rate)).font(.system(size: 9).monospacedDigit())
        }
        .frame(width: 92)
        .padding(4)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 8))
    }
}

struct SensorStats: View {
    let hud: NavHUD
    var body: some View {
        VStack(alignment: .leading, spacing: 1) {
            Text(String(format: "%.0f fps · %@", hud.fps, hud.thermal))
            Text("LiDAR \(hud.depthPoints) px · obst \(hud.obstaclePoints)")
            Text(String(format: "cam %.2f m above floor%@", hud.cameraHeight, hud.floorFound ? "" : " (est.)"))
            Text("map \(hud.mapping)")
            if hud.pointingDown { Text("Point the back camera forward").foregroundStyle(.orange) }
        }
        .font(.system(size: 9).monospacedDigit())
        .padding(4)
        .frame(width: 140, alignment: .leading)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 8))
    }
}

struct DepthInset: View {
    let image: CGImage?
    var body: some View {
        if let img = image {
            // ARKit depth is in sensor (landscape) orientation; the UI is portrait.
            Image(uiImage: UIImage(cgImage: img, scale: 1, orientation: .right))
                .resizable()
                .interpolation(.none)
                .scaledToFit()
                .frame(width: 96)
                .clipShape(RoundedRectangle(cornerRadius: 8))
                .overlay(alignment: .bottom) { Text("LiDAR 0–4 m").font(.system(size: 9)).padding(2) }
                .allowsHitTesting(false)
        }
    }
}

// MARK: - radar (360 virtual lidar from the map + live LiDAR sectors)

struct RadarView: View {
    let hud: NavHUD
    let stop: Double
    let slow: Double
    let robotRadius: Double

    var body: some View {
        Canvas { ctx, size in
            let c = CGPoint(x: size.width / 2, y: size.height / 2)
            let R = min(size.width, size.height) / 2 - 4
            let maxR = 3.0
            let k = R / maxR
            for ring in [0.5, 1.0, 2.0, 3.0] {
                let r = ring * k
                ctx.stroke(Path(ellipseIn: CGRect(x: c.x - r, y: c.y - r, width: 2 * r, height: 2 * r)),
                           with: .color(.white.opacity(0.15)), lineWidth: 1)
            }
            // Screen: forward is up, left is left. Bearing b (+ left) -> (-sin b, -cos b).
            func pt(_ bDeg: Double, _ r: Double) -> CGPoint {
                let b = rad(bDeg)
                return CGPoint(x: c.x - CGFloat(sin(b) * r) * k, y: c.y - CGFloat(cos(b) * r) * k)
            }
            // Live LiDAR sectors as wedges.
            for (i, gap) in hud.sectors.enumerated() where i < hud.sectorBearings.count {
                let b = hud.sectorBearings[i]
                let r = gap.isFinite ? min(maxR, gap + robotRadius) : maxR
                var p = Path()
                p.move(to: c)
                p.addLine(to: pt(b - 5, r))
                p.addLine(to: pt(b + 5, r))
                p.closeSubpath()
                let col = gap.isFinite ? gapColor(gap, stop: stop, slow: slow) : Color.green
                ctx.fill(p, with: .color(col.opacity(0.35)))
            }
            // Virtual lidar points.
            for (i, r) in hud.scan.enumerated() where r < hud.scanMax {
                let p = pt(Double(i) * 5, min(r, maxR))
                let col = gapColor(r - robotRadius, stop: stop, slow: slow)
                ctx.fill(Path(ellipseIn: CGRect(x: p.x - 2, y: p.y - 2, width: 4, height: 4)), with: .color(col))
            }
            // Robot footprint and heading.
            let rr = robotRadius * k
            ctx.stroke(Path(ellipseIn: CGRect(x: c.x - rr, y: c.y - rr, width: 2 * rr, height: 2 * rr)), with: .color(.white), lineWidth: 1.5)
            var h = Path()
            h.move(to: c)
            h.addLine(to: CGPoint(x: c.x, y: c.y - rr - 6))
            ctx.stroke(h, with: .color(.white), lineWidth: 2)
            // Nearest obstacle line.
            if hud.nearest.isFinite && hud.nearest + robotRadius < maxR {
                var n = Path()
                n.move(to: c)
                n.addLine(to: pt(hud.nearestBearing, hud.nearest + robotRadius))
                ctx.stroke(n, with: .color(.red), style: StrokeStyle(lineWidth: 1, dash: [3, 3]))
            }
        }
        .overlay(alignment: .bottom) {
            Text("nearest \(fmtDist(hud.nearest)) @ \(String(format: "%+.0f°", hud.nearestBearing))")
                .font(.system(size: 9).monospacedDigit())
        }
        .overlay(alignment: .topLeading) { Text("radar 3 m").font(.system(size: 9)).padding(2) }
    }
}

// MARK: - minimap (tap to set a goal)

struct MiniMap: View {
    let hud: NavHUD
    let items: [RoomItem]
    let onTap: (P2) -> Void

    var body: some View {
        GeometryReader { geo in
            let size = geo.size
            Canvas { ctx, sz in
                let span = hud.mapSpan
                let s = Double(min(sz.width, sz.height)) / span
                let c = hud.mapCenter
                func scr(_ p: P2) -> CGPoint {
                    CGPoint(x: Double(sz.width) / 2 + (p.x - c.x) * s, y: Double(sz.height) / 2 + (p.z - c.z) * s)
                }
                if let img = hud.mapImage {
                    let side = span * s
                    ctx.draw(Image(decorative: img, scale: 1).interpolation(.none),
                             in: CGRect(x: Double(sz.width) / 2 - side / 2, y: Double(sz.height) / 2 - side / 2, width: side, height: side))
                }
                for it in items where it.footprint.count >= 2 {
                    var p = Path()
                    p.move(to: scr(it.footprint[0]))
                    for q in it.footprint.dropFirst() { p.addLine(to: scr(q)) }
                    p.closeSubpath()
                    let col: Color = it.kind == "wall" ? .white : it.kind == "door" ? .green : .blue
                    ctx.stroke(p, with: .color(col.opacity(0.7)), lineWidth: it.kind == "wall" ? 2 : 1)
                    if it.kind == "object" {
                        ctx.draw(Text(it.label).font(.system(size: 8)).foregroundColor(.white), at: scr(it.center))
                    }
                }
                if hud.path.count >= 2 {
                    var p = Path()
                    p.move(to: scr(hud.path[0]))
                    for q in hud.path.dropFirst() { p.addLine(to: scr(q)) }
                    ctx.stroke(p, with: .color(.cyan), lineWidth: 2.5)
                }
                if let g = hud.goal {
                    let q = scr(g)
                    ctx.fill(Path(ellipseIn: CGRect(x: q.x - 5, y: q.y - 5, width: 10, height: 10)), with: .color(.green))
                }
                if let pose = hud.pose {
                    let q = scr(pose.p)
                    let f = pose.forward
                    let l = P2(-sin(pose.theta), -cos(pose.theta))
                    let tip = scr(pose.p + f * 0.35)
                    let a = scr(pose.p - f * 0.15 + l * 0.15)
                    let b = scr(pose.p - f * 0.15 - l * 0.15)
                    var tri = Path()
                    tri.move(to: tip); tri.addLine(to: a); tri.addLine(to: q); tri.addLine(to: b); tri.closeSubpath()
                    ctx.fill(tri, with: .color(.yellow))
                }
            }
            .contentShape(Rectangle())
            .gesture(SpatialTapGesture().onEnded { v in
                let span = hud.mapSpan
                let s = Double(min(size.width, size.height)) / span
                let p = P2(hud.mapCenter.x + (Double(v.location.x) - Double(size.width) / 2) / s,
                           hud.mapCenter.z + (Double(v.location.y) - Double(size.height) / 2) / s)
                onTap(p)
            })
        }
        .overlay(alignment: .topLeading) { Text(String(format: "map %.0f m · tap = goal", hud.mapSpan)).font(.system(size: 9)).padding(2) }
    }
}

// MARK: - distance chips

struct DistanceRow: View {
    let hud: NavHUD
    let stop: Double
    let slow: Double

    var body: some View {
        HStack(spacing: 6) {
            chip("front", hud.frontGap)
            chip("left", hud.left)
            chip("right", hud.right)
            chip("rear", hud.rear)
            chip("nearest", hud.nearest)
        }
    }

    private func chip(_ name: String, _ d: Double) -> some View {
        VStack(spacing: 0) {
            Text(fmtDist(d)).font(.system(size: 12, weight: .bold).monospacedDigit())
            Text(name).font(.system(size: 8))
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, 3)
        .background((d.isFinite ? gapColor(d, stop: stop, slow: slow) : Color.green).opacity(0.35), in: RoundedRectangle(cornerRadius: 6))
    }
}

// MARK: - bottom panel

struct BottomPanel: View {
    @ObservedObject var engine: NavEngine
    @Binding var showManual: Bool

    var body: some View {
        VStack(spacing: 8) {
            HStack(spacing: 8) {
                RadarView(hud: engine.hud, stop: engine.settings.stopDistance, slow: engine.settings.slowDistance,
                          robotRadius: engine.settings.robotRadius)
                    .frame(height: 150)
                    .background(Color.black.opacity(0.45), in: RoundedRectangle(cornerRadius: 10))
                MiniMap(hud: engine.hud, items: engine.roomItems) { p in engine.setGoal(p, label: "map point") }
                    .frame(height: 150)
                    .clipShape(RoundedRectangle(cornerRadius: 10))
                    .background(Color.black.opacity(0.45), in: RoundedRectangle(cornerRadius: 10))
            }
            DistanceRow(hud: engine.hud, stop: engine.settings.stopDistance, slow: engine.settings.slowDistance)
            Picker("Mode", selection: $engine.mode) {
                ForEach(NavDriveMode.allCases) { Text($0.title).tag($0) }
            }
            .pickerStyle(.segmented)
            if engine.mode == .manual && showManual {
                ManualControls(engine: engine)
            }
            ActionRow(engine: engine, showManual: $showManual)
        }
        .padding(10)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 16))
        .padding(.horizontal, 6)
        .padding(.bottom, 6)
    }
}

struct ActionRow: View {
    @ObservedObject var engine: NavEngine
    @Binding var showManual: Bool

    var body: some View {
        HStack(spacing: 10) {
            Menu {
                if engine.roomItems.filter({ $0.kind == "object" }).isEmpty {
                    Text("Load a RoomPlan map in Settings to list objects")
                }
                ForEach(engine.roomItems.filter { $0.kind == "object" }) { it in
                    Button(it.label) { engine.setGoal(item: it) }
                }
                Button("Clear goal", role: .destructive) { engine.clearGoal() }
            } label: {
                Label("Go to", systemImage: "mappin.circle").labelStyle(.iconOnly).font(.title2)
            }
            if engine.mode == .auto {
                Button { engine.go() } label: {
                    Text("GO").font(.headline).frame(width: 56, height: 40)
                        .background(Color.green, in: RoundedRectangle(cornerRadius: 10))
                }
            }
            if engine.mode == .manual {
                Button { showManual.toggle() } label: {
                    Image(systemName: showManual ? "chevron.down.circle" : "gamecontroller").font(.title2)
                }
            }
            Button { engine.stop() } label: {
                Text("STOP").font(.headline.weight(.heavy)).frame(maxWidth: .infinity, minHeight: 40)
                    .background(Color.red, in: RoundedRectangle(cornerRadius: 10))
            }
            Button { engine.toggleLogging() } label: {
                Image(systemName: engine.isLogging ? "record.circle.fill" : "record.circle").font(.title2)
                    .foregroundStyle(engine.isLogging ? Color.red : Color.white)
            }
            Button { engine.saveMap() } label: {
                Image(systemName: "square.and.arrow.down").font(.title2)
            }
        }
        .foregroundStyle(.white)
    }
}

struct ManualControls: View {
    @ObservedObject var engine: NavEngine
    @State private var moveCm = 50.0
    @State private var speedCmS = 20.0
    @State private var turnDeg = 90.0
    @State private var turnDps = 45.0

    var body: some View {
        HStack(alignment: .center, spacing: 10) {
            Joystick(maxV: engine.settings.maxSpeed, maxW: rad(engine.settings.maxTurnRateDeg)) { v, w in
                engine.joystick(v: v, w: w)
            }
            .frame(width: 110, height: 110)
            VStack(spacing: 4) {
                Stepper(value: $moveCm, in: 5...500, step: 5) {
                    Text("\(Int(moveCm)) cm @ \(Int(speedCmS)) cm/s").font(.caption.monospacedDigit())
                }
                Slider(value: $speedCmS, in: 5...min(100, engine.settings.maxSpeed * 100), step: 5)
                HStack {
                    Button("▲ Fwd") { engine.move(cm: moveCm, speed: speedCmS) }
                    Spacer()
                    Button("▼ Back") { engine.move(cm: -moveCm, speed: speedCmS) }
                }
                .buttonStyle(.bordered)
                Stepper(value: $turnDeg, in: 5...360, step: 5) {
                    Text("\(Int(turnDeg))° @ \(Int(turnDps))°/s").font(.caption.monospacedDigit())
                }
                HStack {
                    Button("⟲ Left") { engine.turn(degrees: turnDeg, speed: turnDps) }
                    Spacer()
                    Button("Right ⟳") { engine.turn(degrees: -turnDeg, speed: turnDps) }
                }
                .buttonStyle(.bordered)
            }
            .font(.caption)
        }
    }
}

/// Drag inside the pad: up = forward, left = turn left. Sends at 10 Hz while held; release = stop.
struct Joystick: View {
    let maxV: Double
    let maxW: Double
    let onChange: (Double, Double) -> Void
    @State private var knob = CGSize.zero
    @State private var timer: Timer?

    var body: some View {
        GeometryReader { geo in
            let R = min(geo.size.width, geo.size.height) / 2
            ZStack {
                Circle().fill(Color.white.opacity(0.12))
                Circle().stroke(Color.white.opacity(0.4), lineWidth: 1)
                Circle().fill(Color.cyan).frame(width: 38, height: 38).offset(knob)
            }
            .gesture(DragGesture(minimumDistance: 0)
                .onChanged { g in
                    var d = CGSize(width: g.location.x - R, height: g.location.y - R)
                    let len = (d.width * d.width + d.height * d.height).squareRoot()
                    if len > R { d = CGSize(width: d.width * R / len, height: d.height * R / len) }
                    knob = d
                    if timer == nil {
                        send(R)
                        timer = Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { _ in send(R) }
                    }
                }
                .onEnded { _ in
                    timer?.invalidate()
                    timer = nil
                    knob = .zero
                    onChange(0, 0)
                })
        }
    }

    private func send(_ R: CGFloat) {
        let v = -Double(knob.height / R) * maxV
        let w = -Double(knob.width / R) * maxW
        onChange(abs(v) < 0.02 ? 0 : v, abs(w) < 0.05 ? 0 : w)
    }
}

// MARK: - settings

struct NavSettingsView: View {
    @ObservedObject var engine: NavEngine
    @Environment(\.dismiss) private var dismiss

    var body: some View {
        NavigationStack {
            Form {
                Section("Map (from modes A / B or a saved nav map)") {
                    Picker("Map", selection: $engine.settings.mapSession) {
                        Text("New map (explore)").tag("")
                        ForEach(engine.mapChoices, id: \.self) { u in
                            Text(u.lastPathComponent).tag(u.lastPathComponent)
                        }
                    }
                    Button("Load map and restart tracking") { engine.restart(); dismiss() }
                    Button("Clear obstacle map", role: .destructive) { engine.clearMap() }
                    Text("The map's ARWorldMap relocalises the phone; its mesh, RoomPlan room and saved grid become the static obstacle layer. Walk the phone past a mapped area until the status says \"relocalised\".")
                        .font(.caption).foregroundStyle(.secondary)
                }
                Section("Robot link") {
                    Picker("Link", selection: $engine.settings.link) {
                        ForEach(RobotLinkKind.allCases) { Text($0.title).tag($0) }
                    }
                    if engine.settings.link == .websocket {
                        TextField("ws://192.168.4.1:8777/robot", text: $engine.settings.wsURL)
                            .keyboardType(.URL).textInputAutocapitalization(.never).autocorrectionDisabled()
                    }
                    if engine.settings.link == .ble {
                        TextField("BLE name prefix", text: $engine.settings.bleName)
                            .textInputAutocapitalization(.never).autocorrectionDisabled()
                    }
                    Button("Connect") { engine.reconnect() }
                    Toggle("Moves / turns closed-loop with ARKit", isOn: $engine.settings.closedLoopMoves)
                    Text("Closed-loop: the phone measures the motion and streams velocities. Off: \"move\" and \"turn\" go to the robot as is (it needs encoders).")
                        .font(.caption).foregroundStyle(.secondary)
                }
                Section("Robot and mount") {
                    row("Robot radius", $engine.settings.robotRadius, 0.05...1.0, 0.01, "m")
                    row("Obstacle height up to", $engine.settings.robotHeight, 0.1...2.0, 0.05, "m")
                    row("Camera height (until floor found)", $engine.settings.mountHeight, 0.05...2.0, 0.01, "m")
                    row("Camera ahead of turning centre", $engine.settings.mountForward, -0.5...0.5, 0.01, "m")
                }
                Section("Motion and safety") {
                    row("Max speed", $engine.settings.maxSpeed, 0.05...1.5, 0.05, "m/s")
                    row("Max turn rate", $engine.settings.maxTurnRateDeg, 10...180, 5, "°/s")
                    row("Slow down below", $engine.settings.slowDistance, 0.1...2.0, 0.05, "m")
                    row("Stop below", $engine.settings.stopDistance, 0.05...1.0, 0.05, "m")
                    row("Goal tolerance", $engine.settings.goalTolerance, 0.05...0.5, 0.05, "m")
                }
                Section("Guidance and display") {
                    Toggle("Voice instructions", isOn: $engine.settings.voice)
                    Toggle("Proximity beeps", isOn: $engine.settings.beeps)
                    Toggle("Haptics", isOn: $engine.settings.haptics)
                    row("Minimap span", $engine.settings.minimapSpan, 2...30, 1, "m")
                }
            }
            .navigationTitle("Navigation")
            .toolbar { Button("Done") { dismiss() } }
            .onAppear { engine.refreshMapChoices() }
        }
    }

    private func row(_ title: String, _ value: Binding<Double>, _ range: ClosedRange<Double>, _ step: Double, _ unit: String) -> some View {
        Stepper(value: value, in: range, step: step) {
            HStack {
                Text(title)
                Spacer()
                Text(step < 1 ? String(format: "%.2f %@", value.wrappedValue, unit) : String(format: "%.0f %@", value.wrappedValue, unit))
                    .monospacedDigit().foregroundStyle(.secondary)
            }
        }
    }
}
