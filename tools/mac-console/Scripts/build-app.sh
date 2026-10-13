#!/bin/bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
app_root=$(cd -- "$script_dir/.." && pwd)
app_path="$app_root/.build/Lunar Robot Console.app"
binary="$app_root/.build/LunarRobotConsole"
sdk_path=${SDKROOT:-$(xcrun --sdk macosx --show-sdk-path)}
swiftc_path=$(xcrun --find swiftc)

mkdir -p "$app_root/.build" "$app_path/Contents/MacOS"
"$swiftc_path" \
  -sdk "$sdk_path" \
  -target arm64-apple-macosx14.0 \
  -swift-version 5 \
  -O \
  -o "$binary" \
  "$app_root"/Sources/LunarRobotConsole/*.swift
cp "$binary" "$app_path/Contents/MacOS/LunarRobotConsole"
cat > "$app_path/Contents/Info.plist" <<'PLIST'
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleExecutable</key>
    <string>LunarRobotConsole</string>
    <key>CFBundleIdentifier</key>
    <string>com.tyr1onx.LunarRobotConsole</string>
    <key>CFBundleName</key>
    <string>Lunar Robot Console</string>
    <key>CFBundleDisplayName</key>
    <string>Lunar Robot Console</string>
    <key>CFBundlePackageType</key>
    <string>APPL</string>
    <key>LSMinimumSystemVersion</key>
    <string>14.0</string>
    <key>NSHighResolutionCapable</key>
    <true/>
</dict>
</plist>
PLIST
codesign --force --deep --sign - "$app_path" >/dev/null
printf 'Built: %s\n' "$app_path"
