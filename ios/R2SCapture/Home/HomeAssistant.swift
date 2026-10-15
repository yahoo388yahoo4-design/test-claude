import Foundation

/// Room-map / Home Assistant settings, kept apart from CaptureSettings so older saved settings still decode.
struct HomeSettings: Codable, Equatable {
    var roomName: String = ""
    var haURL: String = ""          // e.g. http://homeassistant.local:8123
    var haToken: String = ""        // long-lived access token (HA profile page)
    var autoSend: Bool = true

    private static let key = "r2s.homeSettings.v1"

    static func load() -> HomeSettings {
        guard let data = UserDefaults.standard.data(forKey: key),
              let s = try? JSONDecoder().decode(HomeSettings.self, from: data) else { return HomeSettings() }
        return s
    }

    func save() {
        if let data = try? JSONEncoder().encode(self) { UserDefaults.standard.set(data, forKey: HomeSettings.key) }
    }

    var configured: Bool { !haURL.trimmingCharacters(in: .whitespaces).isEmpty && !haToken.trimmingCharacters(in: .whitespaces).isEmpty }
}

/// Sends roommap/room_map.json to the Home Assistant "Real2Sim Home" integration:
/// POST <ha>/api/real2sim_home/room_map with the long-lived token (see HOME.md).
enum HomeAssistantClient {
    static func roomMapURL(session dir: URL) -> URL { dir.appendingPathComponent("roommap/room_map.json") }

    static func hasRoomMap(_ dir: URL) -> Bool { FileManager.default.fileExists(atPath: roomMapURL(session: dir).path) }

    /// Returns a status line for the UI.
    static func send(session dir: URL, settings: HomeSettings) async -> String {
        guard settings.configured else { return "Home Assistant: set the URL and token in Settings" }
        guard var body = try? Data(contentsOf: roomMapURL(session: dir)) else { return "No room map in \(dir.lastPathComponent)" }
        // RuView nodes placed since the scan (esp_nodes.json) go along as the `sensors` layer
        if let map = (try? JSONSerialization.jsonObject(with: body)) as? [String: Any],
           let merged = RoomMapCore.withEspNodes(map, session: dir),
           let d = try? JSONSerialization.data(withJSONObject: merged, options: [.prettyPrinted, .sortedKeys]) {
            body = d
            try? d.write(to: roomMapURL(session: dir), options: .atomic)
        }
        var base = settings.haURL.trimmingCharacters(in: .whitespaces)
        while base.hasSuffix("/") { base.removeLast() }
        guard let url = URL(string: base + "/api/real2sim_home/room_map") else { return "Home Assistant: bad URL" }
        var req = URLRequest(url: url)
        req.httpMethod = "POST"
        req.timeoutInterval = 30
        req.setValue("application/json", forHTTPHeaderField: "Content-Type")
        req.setValue("Bearer \(settings.haToken.trimmingCharacters(in: .whitespaces))", forHTTPHeaderField: "Authorization")
        do {
            let (data, resp) = try await URLSession.shared.upload(for: req, from: body)
            let code = (resp as? HTTPURLResponse)?.statusCode ?? 0
            if (200..<300).contains(code) {
                let obj = (try? JSONSerialization.jsonObject(with: body)) as? [String: Any]
                let n = (obj?["objects"] as? [Any])?.count ?? 0
                return "Sent room \"\(obj?["name"] as? String ?? "")\" (\(n) objects) to Home Assistant"
            }
            return "Home Assistant answered \(code): \(String(data: data.prefix(200), encoding: .utf8) ?? "")"
        } catch {
            return "Home Assistant unreachable: \(error.localizedDescription)"
        }
    }
}
