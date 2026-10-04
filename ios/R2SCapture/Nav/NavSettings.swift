import Foundation

enum RobotLinkKind: String, CaseIterable, Identifiable, Codable {
    case none
    case websocket
    case ble

    var id: String { rawValue }
    var title: String {
        switch self {
        case .none: return "None (guide only)"
        case .websocket: return "Wi-Fi (WebSocket)"
        case .ble: return "Bluetooth LE (UART)"
        }
    }
}

/// What the navigation screen does with the plan.
enum NavDriveMode: String, CaseIterable, Identifiable, Codable {
    case guide      // arrows, voice and beeps only; a person (or a robot you drive yourself) follows
    case auto       // the phone drives the robot along the path
    case manual     // joystick and "move X cm / turn N degrees" commands

    var id: String { rawValue }
    var title: String {
        switch self {
        case .guide: return "Guide"
        case .auto: return "Auto"
        case .manual: return "Manual"
        }
    }
}

/// Persisted navigation settings (separate from CaptureSettings so the capture modes are unaffected).
struct NavSettings: Codable, Equatable {
    var link: RobotLinkKind = .websocket
    var wsURL = "ws://192.168.4.1:8777/robot"   // ESP32 soft-AP default; the Python receiver prints its own
    var bleName = "R2S-Robot"
    /// Camera height above the floor, used until ARKit finds the floor plane.
    var mountHeight = 0.25
    /// Camera distance ahead of the robot's turning centre (negative = behind).
    var mountForward = 0.0
    var robotRadius = 0.20
    /// Points between 4 cm above the floor and this height are obstacles.
    var robotHeight = 0.60
    var maxSpeed = 0.30            // m/s
    var maxTurnRateDeg = 60.0      // deg/s
    var slowDistance = 0.60        // m, forward gap from the robot's edge
    var stopDistance = 0.25
    var goalTolerance = 0.15
    var voice = true
    var beeps = true
    var haptics = true
    /// Session folder whose worldmap.arworldmap (and mesh / RoomPlan / map.pgm) is the map; "" = new map.
    var mapSession = ""
    var logToSession = true
    /// "move" / "turn" commands: true = the phone closes the loop with ARKit odometry and sends velocities;
    /// false = the command is sent to the robot as is (it needs encoders or timing to execute it).
    var closedLoopMoves = true
    var minimapSpan = 8.0          // metres shown across the minimap

    private static let key = "r2s.navSettings.v1"

    static func load() -> NavSettings {
        guard let data = UserDefaults.standard.data(forKey: key),
              let s = try? JSONDecoder().decode(NavSettings.self, from: data) else { return NavSettings() }
        return s
    }

    func save() {
        if let data = try? JSONEncoder().encode(self) {
            UserDefaults.standard.set(data, forKey: NavSettings.key)
        }
    }

    var asDictionary: [String: Any] {
        guard let data = try? JSONEncoder().encode(self),
              let obj = try? JSONSerialization.jsonObject(with: data) as? [String: Any] else { return [:] }
        return obj
    }

    var followerConfig: FollowerConfig {
        var c = FollowerConfig()
        c.maxSpeed = maxSpeed
        c.maxTurnRate = rad(maxTurnRateDeg)
        c.goalTolerance = goalTolerance
        c.slowDistance = max(slowDistance, stopDistance + 0.05)
        c.stopDistance = stopDistance
        return c
    }

    var plannerConfig: PlannerConfig {
        var c = PlannerConfig()
        c.robotRadius = robotRadius
        return c
    }
}
