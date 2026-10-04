import SwiftUI

@main
struct R2SCaptureApp: App {
    @StateObject private var model = CaptureModel()

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(model)
                .preferredColorScheme(.dark)
        }
    }
}
