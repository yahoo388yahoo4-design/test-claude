import Foundation

enum CaptureMode: String, CaseIterable, Identifiable, Codable {
    case arkitRGBD = "arkit_rgbd"
    case arkitRoomPlan = "arkit_roomplan"
    case multiCam = "multicam"

    var id: String { rawValue }

    var title: String {
        switch self {
        case .arkitRGBD: return "A · RGB-D + poses (ARKitScenes)"
        case .arkitRoomPlan: return "B · RGB-D + RoomPlan (LiteReality)"
        case .multiCam: return "C · Multi-camera + LiDAR (no poses)"
        }
    }

    var shortTitle: String {
        switch self {
        case .arkitRGBD: return "RGB-D"
        case .arkitRoomPlan: return "RoomPlan"
        case .multiCam: return "MultiCam"
        }
    }
}

enum VideoPreset: String, CaseIterable, Identifiable, Codable {
    case hd60 = "1920x1440 @ 60 fps"
    case uhd30 = "3840x2880 @ 30 fps (4K)"
    var id: String { rawValue }
}

struct CaptureSettings: Codable, Equatable {
    var mode: CaptureMode = .arkitRGBD
    var videoPreset: VideoPreset = .hd60
    var videoBitrateMbps: Double = 50
    var smoothedDepth = true
    var sceneMesh = true
    var lockFocus = true
    var lockExposureAndWhiteBalance = false
    var hiresStillIntervalSec: Double = 0      // 0 = off; captureHighResolutionFrame stills
    var recordAudio = false
    var recordLocation = true
    var saveWorldMap = true
    var multicamIncludeFront = false
    var uploadURL: String = ""                 // e.g. http://192.168.1.20:8765

    private static let key = "r2s.captureSettings.v1"

    static func load() -> CaptureSettings {
        guard let data = UserDefaults.standard.data(forKey: key),
              let s = try? JSONDecoder().decode(CaptureSettings.self, from: data) else { return CaptureSettings() }
        return s
    }

    func save() {
        if let data = try? JSONEncoder().encode(self) {
            UserDefaults.standard.set(data, forKey: CaptureSettings.key)
        }
    }

    var asDictionary: [String: Any] {
        guard let data = try? JSONEncoder().encode(self),
              let obj = try? JSONSerialization.jsonObject(with: data) as? [String: Any] else { return [:] }
        return obj
    }
}
