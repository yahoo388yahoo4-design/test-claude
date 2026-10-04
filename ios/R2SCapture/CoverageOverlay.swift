import ARKit
import SceneKit
import SwiftUI
import UIKit

/// Live "what has been captured" overlay for modes A and B: a transparent SceneKit view drawn over the
/// camera preview, with its camera driven by the ARSession's current frame.
/// * If the session produces scene-reconstruction meshes (mode A with "Scene mesh" on), the LiDAR mesh
///   is drawn as a translucent surface that grows as you scan.
/// * Otherwise (e.g. RoomPlan's own session), LiDAR depth is back-projected into a growing point cloud
///   (high-confidence pixels, 3 cm voxel de-duplication, coloured by height).
final class CoverageOverlayView: SCNView, SCNSceneRendererDelegate {
    weak var session: ARSession?
    private let cameraNode = SCNNode()
    private let meshRoot = SCNNode()
    private let pointsRoot = SCNNode()
    private var meshNodes: [UUID: (anchor: ARMeshAnchor, node: SCNNode)] = [:]
    private var seenMesh = false
    private var voxels = Set<Int64>()
    private var pointCount = 0
    private var lastSample: TimeInterval = 0
    private var lastMeshUpdate: TimeInterval = 0
    private var viewport = CGSize(width: 1, height: 1)
    private let lock = NSLock()
    private var needsReset = false

    static let maxPoints = 400_000

    init(session: ARSession) {
        self.session = session
        super.init(frame: .zero, options: nil)
        backgroundColor = .clear
        isOpaque = false
        isUserInteractionEnabled = false
        antialiasingMode = .none
        let scene = SCNScene()
        scene.background.contents = UIColor.clear
        let cam = SCNCamera()
        cam.zNear = 0.01
        cam.zFar = 100
        cameraNode.camera = cam
        scene.rootNode.addChildNode(cameraNode)
        scene.rootNode.addChildNode(meshRoot)
        scene.rootNode.addChildNode(pointsRoot)
        self.scene = scene
        pointOfView = cameraNode
        delegate = self
        rendersContinuously = true
        isPlaying = true
    }

    required init?(coder: NSCoder) { fatalError("init(coder:) is not supported") }

    override func layoutSubviews() {
        super.layoutSubviews()
        lock.lock()
        viewport = bounds.size
        lock.unlock()
    }

    /// Clears what has been drawn (called when a new recording starts).
    func reset() {
        lock.lock()
        needsReset = true
        lock.unlock()
    }

    // MARK: SCNSceneRendererDelegate (render thread)

    func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        guard let frame = session?.currentFrame else { return }
        lock.lock()
        let size = viewport
        let resetNow = needsReset
        needsReset = false
        lock.unlock()
        if resetNow {
            meshRoot.childNodes.forEach { $0.removeFromParentNode() }
            pointsRoot.childNodes.forEach { $0.removeFromParentNode() }
            meshNodes.removeAll()
            voxels.removeAll()
            pointCount = 0
        }
        guard size.width > 1, size.height > 1 else { return }
        let orientation: UIInterfaceOrientation = .portrait
        cameraNode.simdTransform = frame.camera.viewMatrix(for: orientation).inverse
        cameraNode.camera?.projectionTransform = SCNMatrix4(
            frame.camera.projectionMatrix(for: orientation, viewportSize: size, zNear: 0.01, zFar: 100))
        guard frame.camera.trackingState == .normal else { return }

        let meshes = frame.anchors.compactMap { $0 as? ARMeshAnchor }
        if !meshes.isEmpty {
            if !seenMesh {
                seenMesh = true
                pointsRoot.isHidden = true
            }
            if time - lastMeshUpdate > 0.5 {
                lastMeshUpdate = time
                updateMeshes(meshes)
            }
        } else if !seenMesh, time - lastSample > 0.4 {
            lastSample = time
            addDepthPoints(frame)
        }
    }

    // MARK: mesh

    private func updateMeshes(_ anchors: [ARMeshAnchor]) {
        for a in anchors {
            if let existing = meshNodes[a.identifier] {
                if existing.anchor !== a {
                    existing.node.geometry = CoverageOverlayView.geometry(a.geometry)
                    existing.node.simdTransform = a.transform
                    meshNodes[a.identifier] = (a, existing.node)
                }
            } else {
                let node = SCNNode(geometry: CoverageOverlayView.geometry(a.geometry))
                node.simdTransform = a.transform
                meshRoot.addChildNode(node)
                meshNodes[a.identifier] = (a, node)
            }
        }
    }

    private static let meshMaterial: SCNMaterial = {
        let m = SCNMaterial()
        m.diffuse.contents = UIColor(red: 0.2, green: 0.85, blue: 1.0, alpha: 1)
        m.lightingModel = .constant
        m.transparency = 0.35
        m.isDoubleSided = true
        m.writesToDepthBuffer = false
        return m
    }()

    /// Copies ARKit's mesh buffers (they are reused by ARKit) into an SCNGeometry.
    static func geometry(_ g: ARMeshGeometry) -> SCNGeometry {
        let v = g.vertices
        let vdata = Data(bytes: v.buffer.contents().advanced(by: v.offset), count: v.stride * v.count)
        let source = SCNGeometrySource(data: vdata, semantic: .vertex, vectorCount: v.count, usesFloatComponents: true,
                                       componentsPerVector: 3, bytesPerComponent: MemoryLayout<Float>.size,
                                       dataOffset: 0, dataStride: v.stride)
        let f = g.faces
        let fdata = Data(bytes: f.buffer.contents(), count: f.count * f.indexCountPerPrimitive * f.bytesPerIndex)
        let element = SCNGeometryElement(data: fdata, primitiveType: .triangles, primitiveCount: f.count,
                                         bytesPerIndex: f.bytesPerIndex)
        let geo = SCNGeometry(sources: [source], elements: [element])
        geo.materials = [meshMaterial]
        return geo
    }

    // MARK: depth points

    private func addDepthPoints(_ frame: ARFrame) {
        guard pointCount < CoverageOverlayView.maxPoints,
              let depth = frame.smoothedSceneDepth ?? frame.sceneDepth else { return }
        let map = depth.depthMap
        CVPixelBufferLockBaseAddress(map, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(map, .readOnly) }
        let w = CVPixelBufferGetWidth(map), h = CVPixelBufferGetHeight(map)
        guard let base = CVPixelBufferGetBaseAddress(map) else { return }
        let stride = CVPixelBufferGetBytesPerRow(map)
        var conf: UnsafeRawPointer?
        var confStride = 0
        if let c = depth.confidenceMap {
            CVPixelBufferLockBaseAddress(c, .readOnly)
            conf = UnsafeRawPointer(CVPixelBufferGetBaseAddress(c))
            confStride = CVPixelBufferGetBytesPerRow(c)
        }
        defer { if let c = depth.confidenceMap { CVPixelBufferUnlockBaseAddress(c, .readOnly) } }

        let K = frame.camera.intrinsics
        let sx = Float(w) / Float(frame.camera.imageResolution.width)
        let sy = Float(h) / Float(frame.camera.imageResolution.height)
        let fx = K[0][0] * sx, fy = K[1][1] * sy, cx = K[2][0] * sx, cy = K[2][1] * sy
        let T = frame.camera.transform
        let voxel: Float = 0.03

        var pts: [SIMD3<Float>] = []
        var cols: [SIMD3<Float>] = []
        let step = 6
        for y in Swift.stride(from: 0, to: h, by: step) {
            let row = (base + y * stride).assumingMemoryBound(to: Float32.self)
            for x in Swift.stride(from: 0, to: w, by: step) {
                if let conf = conf, conf.load(fromByteOffset: y * confStride + x, as: UInt8.self) < 2 { continue }
                let d = row[x]
                guard d.isFinite, d > 0.1, d < 6 else { continue }
                // ARKit camera space: x right, y up, looking down -z
                let pc = SIMD4<Float>((Float(x) - cx) / fx * d, -(Float(y) - cy) / fy * d, -d, 1)
                let pw = T * pc
                let key = (Int64((pw.x / voxel).rounded()) & 0x1FFFFF) << 42
                    | (Int64((pw.y / voxel).rounded()) & 0x1FFFFF) << 21
                    | (Int64((pw.z / voxel).rounded()) & 0x1FFFFF)
                guard voxels.insert(key).inserted else { continue }
                pts.append(SIMD3(pw.x, pw.y, pw.z))
                let t = min(max((pw.y + 1.5) / 3.0, 0), 1)      // height -> colour
                cols.append(SIMD3(t, 0.4 + 0.6 * (1 - abs(t - 0.5) * 2), 1 - t))
            }
        }
        guard !pts.isEmpty else { return }
        pointCount += pts.count
        let vsrc = SCNGeometrySource(vertices: pts.map { SCNVector3($0.x, $0.y, $0.z) })
        let cdata = cols.withUnsafeBufferPointer { Data(buffer: $0) }
        let csrc = SCNGeometrySource(data: cdata, semantic: .color, vectorCount: cols.count, usesFloatComponents: true,
                                     componentsPerVector: 3, bytesPerComponent: MemoryLayout<Float>.size,
                                     dataOffset: 0, dataStride: MemoryLayout<SIMD3<Float>>.stride)
        let element = SCNGeometryElement(indices: Array(0..<Int32(pts.count)), primitiveType: .point)
        element.pointSize = 4
        element.minimumPointScreenSpaceRadius = 1.5
        element.maximumPointScreenSpaceRadius = 4
        let geo = SCNGeometry(sources: [vsrc, csrc], elements: [element])
        let m = SCNMaterial()
        m.lightingModel = .constant
        geo.materials = [m]
        pointsRoot.addChildNode(SCNNode(geometry: geo))
    }
}

struct CoverageOverlay: UIViewRepresentable {
    let session: ARSession
    let resetToken: Int
    func makeUIView(context: Context) -> CoverageOverlayView { CoverageOverlayView(session: session) }
    func updateUIView(_ uiView: CoverageOverlayView, context: Context) {
        if context.coordinator.token != resetToken {
            context.coordinator.token = resetToken
            uiView.reset()
        }
    }
    func makeCoordinator() -> Coordinator { Coordinator(token: resetToken) }
    final class Coordinator {
        var token: Int
        init(token: Int) { self.token = token }
    }
}
