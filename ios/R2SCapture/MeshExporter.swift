import ARKit
import Foundation
import Metal

/// Writes ARKit scene reconstruction (ARMeshAnchor) as one binary PLY in world coordinates with
/// per-vertex normals and per-face ARMeshClassification, plus plane anchors as JSON.
enum MeshExporter {
    static func writeMesh(anchors: [ARAnchor], to url: URL) {
        let meshes = anchors.compactMap { $0 as? ARMeshAnchor }
        guard !meshes.isEmpty else { return }
        var vertexData = Data()
        var faceData = Data()
        var vertexCount = 0
        var faceCount = 0
        for anchor in meshes {
            let g = anchor.geometry
            let T = anchor.transform
            let R = simd_float3x3(simd_make_float3(T.columns.0), simd_make_float3(T.columns.1), simd_make_float3(T.columns.2))
            let base = vertexCount
            let vs = g.vertices
            let ns = g.normals
            let vptr = vs.buffer.contents()
            let nptr = ns.buffer.contents()
            for i in 0..<vs.count {
                let p = vptr.advanced(by: vs.offset + vs.stride * i).assumingMemoryBound(to: Float.self)
                let n = nptr.advanced(by: ns.offset + ns.stride * i).assumingMemoryBound(to: Float.self)
                let wp = T * simd_float4(p[0], p[1], p[2], 1)
                let wn = R * simd_float3(n[0], n[1], n[2])
                var rec: [Float] = [wp.x, wp.y, wp.z, wn.x, wn.y, wn.z]
                vertexData.append(Data(bytes: &rec, count: 24))
            }
            vertexCount += vs.count
            let f = g.faces
            let fptr = f.buffer.contents()
            let cls = g.classification
            for i in 0..<f.count {
                var idx = [Int32](repeating: 0, count: 3)
                for k in 0..<3 {
                    let off = (i * f.indexCountPerPrimitive + k) * f.bytesPerIndex
                    let v: Int
                    if f.bytesPerIndex == 2 {
                        v = Int(fptr.advanced(by: off).assumingMemoryBound(to: UInt16.self).pointee)
                    } else {
                        v = Int(fptr.advanced(by: off).assumingMemoryBound(to: UInt32.self).pointee)
                    }
                    idx[k] = Int32(base + v)
                }
                var c: UInt8 = 0
                if let cls = cls {
                    c = cls.buffer.contents().advanced(by: cls.offset + cls.stride * i).assumingMemoryBound(to: UInt8.self).pointee
                }
                var n: UInt8 = 3
                faceData.append(Data(bytes: &n, count: 1))
                idx.withUnsafeBytes { faceData.append(contentsOf: $0) }
                faceData.append(Data(bytes: &c, count: 1))
            }
            faceCount += f.count
        }
        let header = """
        ply
        format binary_little_endian 1.0
        comment r2s-capture ARKit scene mesh, ARKit world frame (y up, metres)
        comment face cls = ARMeshClassification 0 none 1 wall 2 floor 3 ceiling 4 table 5 seat 6 window 7 door
        element vertex \(vertexCount)
        property float x
        property float y
        property float z
        property float nx
        property float ny
        property float nz
        element face \(faceCount)
        property list uchar int vertex_indices
        property uchar cls
        end_header

        """
        var out = Data(header.utf8)
        out.append(vertexData)
        out.append(faceData)
        try? out.write(to: url)
    }

    static func writePlanes(anchors: [ARAnchor], to url: URL) {
        let planes: [[String: Any]] = anchors.compactMap { a in
            guard let p = a as? ARPlaneAnchor else { return nil }
            let boundary = p.geometry.boundaryVertices.map { [Double($0.x), Double($0.y), Double($0.z)] }
            return [
                "id": p.identifier.uuidString,
                "alignment": p.alignment == .horizontal ? "horizontal" : "vertical",
                "classification": classificationString(p.classification),
                "T": ARRecorder.rowMajor(p.transform),
                "center": [Double(p.center.x), Double(p.center.y), Double(p.center.z)],
                "extent": [Double(p.planeExtent.width), Double(p.planeExtent.height)],
                "rotation_y": Double(p.planeExtent.rotationOnYAxis),
                "boundary": boundary,
            ]
        }
        SessionStorage.writeJSON(planes, to: url)
    }

    static func classificationString(_ c: ARPlaneAnchor.Classification) -> String {
        switch c {
        case .wall: return "wall"
        case .floor: return "floor"
        case .ceiling: return "ceiling"
        case .table: return "table"
        case .seat: return "seat"
        case .window: return "window"
        case .door: return "door"
        case .none: return "none"
        @unknown default: return "unknown"
        }
    }
}
