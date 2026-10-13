// swift-tools-version: 5.9
import PackageDescription

let package = Package(
    name: "LunarRobotConsole",
    platforms: [.macOS(.v14)],
    products: [
        .executable(name: "LunarRobotConsole", targets: ["LunarRobotConsole"])
    ],
    targets: [
        .executableTarget(name: "LunarRobotConsole"),
        .testTarget(name: "LunarRobotConsoleTests", dependencies: ["LunarRobotConsole"])
    ],
    swiftLanguageVersions: [.v5]
)
