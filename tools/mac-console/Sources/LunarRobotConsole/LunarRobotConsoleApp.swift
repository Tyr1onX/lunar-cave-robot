import SwiftUI

@main
struct LunarRobotConsoleApp: App {
    @StateObject private var model = ConsoleModel()

    var body: some Scene {
        WindowGroup {
            ConsoleRootView(model: model)
                .preferredColorScheme(.dark)
                .frame(minWidth: 1120, minHeight: 760)
                .onAppear { model.start() }
                .onDisappear { model.stop() }
        }
        .defaultSize(width: 1400, height: 900)
    }
}
