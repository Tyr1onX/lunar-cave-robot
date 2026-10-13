import http from "node:http";
import { spawn } from "node:child_process";
import { readFile } from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(HERE, "../..");
const PUBLIC = path.join(HERE, "public");
const REMOTE_SCRIPT = path.join(HERE, "remote_snapshot.py");
const STATUS_SCRIPT = path.join(ROOT, "tools/device_status.sh");
const HOST = process.env.LUNAR_NUC_ALIAS || "nuc";
const THINKBOOK_HOST = process.env.LUNAR_THINKBOOK_ALIAS || "thinkbook";
const PORT = Number(process.env.LUNAR_CONSOLE_PORT || 4173);
const MAX_OUTPUT = 1_000_000;
const CACHE_MS = 8_000;
const ALLOWED_ASSETS = new Map([
  ["/", ["index.html", "text/html; charset=utf-8"]],
  ["/app.js", ["app.js", "text/javascript; charset=utf-8"]],
  ["/styles.css", ["styles.css", "text/css; charset=utf-8"]],
]);

if (!/^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$/.test(HOST)) {
  throw new Error("LUNAR_NUC_ALIAS must be a simple SSH config alias.");
}
if (!/^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$/.test(THINKBOOK_HOST)) {
  throw new Error("LUNAR_THINKBOOK_ALIAS must be a simple SSH config alias.");
}
if (!Number.isInteger(PORT) || PORT < 1024 || PORT > 65535) {
  throw new Error("LUNAR_CONSOLE_PORT must be between 1024 and 65535.");
}

function run(command, args, { input, timeoutMs = 25_000, cwd = HERE, env = process.env } = {}) {
  return new Promise((resolve, reject) => {
    const child = spawn(command, args, {
      cwd,
      env,
      stdio: ["pipe", "pipe", "pipe"],
    });
    let stdout = "";
    let stderr = "";
    let settled = false;
    const timer = setTimeout(() => {
      child.kill("SIGTERM");
      finish(new Error("Timed out"));
    }, timeoutMs);
    const finish = (error, result) => {
      if (settled) return;
      settled = true;
      clearTimeout(timer);
      if (error) reject(error);
      else resolve(result);
    };
    child.stdout.setEncoding("utf8");
    child.stderr.setEncoding("utf8");
    child.stdout.on("data", (part) => {
      stdout += part;
      if (stdout.length + stderr.length > MAX_OUTPUT) child.kill("SIGTERM");
    });
    child.stderr.on("data", (part) => {
      stderr += part;
      if (stdout.length + stderr.length > MAX_OUTPUT) child.kill("SIGTERM");
    });
    child.on("error", (error) => finish(error));
    child.on("close", (code, signal) => {
      if (stdout.length + stderr.length > MAX_OUTPUT) {
        finish(new Error("Output limit exceeded"));
        return;
      }
      finish(null, { code: code ?? 128, signal, stdout, stderr });
    });
    if (input) child.stdin.write(input);
    child.stdin.end();
  });
}

function parseStatus(output) {
  const sections = {};
  let section = "";
  for (const line of output.split(/\r?\n/)) {
    if (/^(GitHub main|ThinkBook|NUC):$/.test(line.trim())) {
      section = line.trim().slice(0, -1);
      sections[section] = {};
      continue;
    }
    const match = line.match(/^\s{2}([^:]+):\s*(.*)$/);
    if (match && section) sections[section][match[1]] = match[2];
  }
  return sections;
}

async function collectStatus() {
  const [statusResult, remoteSource] = await Promise.all([
    run("/bin/bash", [STATUS_SCRIPT], {
      cwd: ROOT,
      timeoutMs: 28_000,
      env: {
        ...process.env,
        NUC_ALIAS: HOST,
        THINKBOOK_ALIAS: THINKBOOK_HOST,
      },
    }),
    readFile(REMOTE_SCRIPT, "utf8"),
  ]);
  const remoteResult = await run(
    "ssh",
    [
      "-o", "BatchMode=yes",
      "-o", "ConnectTimeout=8",
      "-o", "ClearAllForwardings=yes",
      "-o", "ForwardAgent=no",
      "-o", "RequestTTY=no",
      HOST,
      "python3",
      "-",
    ],
    { input: remoteSource, timeoutMs: 22_000 },
  );

  let remote = null;
  let remoteError = "";
  try {
    if (remoteResult.code !== 0) throw new Error("SSH snapshot unavailable");
    remote = JSON.parse(remoteResult.stdout);
  } catch {
    remoteError = "无法读取 NUC 实时快照；请确认 SSH 别名和连接状态。";
  }

  let status = {};
  let statusError = "";
  try {
    if (statusResult.code !== 0) throw new Error("device_status failed");
    status = parseStatus(statusResult.stdout);
  } catch {
    statusError = "统一设备状态脚本未能完成；下方系统快照仍会独立显示。";
  }

  return {
    generatedAt: new Date().toISOString(),
    source: "SSH 只读查询",
    status,
    statusError,
    remote,
    remoteError,
  };
}

let cachedStatus = null;
let cachedAt = 0;
let activeRequest = null;

async function getStatus() {
  if (activeRequest) return activeRequest;
  if (cachedStatus && Date.now() - cachedAt < CACHE_MS) return cachedStatus;
  activeRequest = collectStatus();
  try {
    cachedStatus = await activeRequest;
    cachedAt = Date.now();
    return cachedStatus;
  } finally {
    activeRequest = null;
  }
}

function sendJson(response, statusCode, payload) {
  response.writeHead(statusCode, {
    "Content-Type": "application/json; charset=utf-8",
    "Cache-Control": "no-store",
  });
  response.end(JSON.stringify(payload));
}

const server = http.createServer(async (request, response) => {
  response.setHeader("X-Content-Type-Options", "nosniff");
  response.setHeader("Referrer-Policy", "no-referrer");
  response.setHeader("Content-Security-Policy", "default-src 'self'; style-src 'self'; script-src 'self'; connect-src 'self'; img-src 'self' data:; object-src 'none'; frame-ancestors 'none'");

  const url = new URL(request.url || "/", `http://${request.headers.host || "127.0.0.1"}`);
  if (url.pathname === "/api/status") {
    if (request.method !== "GET") {
      sendJson(response, 405, { error: "Only GET is supported." });
      return;
    }
    try {
      sendJson(response, 200, await getStatus());
    } catch (error) {
      console.error("Status refresh failed:", error?.message || "unknown error");
      sendJson(response, 200, {
        generatedAt: new Date().toISOString(),
        source: "SSH 只读查询",
        status: {},
        statusError: "无法读取设备状态。",
        remote: null,
        remoteError: "无法读取 NUC 实时快照；请确认 SSH 别名和连接状态。",
      });
    }
    return;
  }

  if (request.method !== "GET") {
    response.writeHead(405, { "Content-Type": "text/plain; charset=utf-8" });
    response.end("Method Not Allowed");
    return;
  }
  const asset = ALLOWED_ASSETS.get(url.pathname);
  if (!asset) {
    response.writeHead(404, { "Content-Type": "text/plain; charset=utf-8" });
    response.end("Not Found");
    return;
  }
  try {
    const body = await readFile(path.join(PUBLIC, asset[0]));
    response.writeHead(200, {
      "Content-Type": asset[1],
      "Cache-Control": "no-store",
    });
    response.end(body);
  } catch {
    response.writeHead(500, { "Content-Type": "text/plain; charset=utf-8" });
    response.end("Console asset unavailable.");
  }
});

server.listen(PORT, "127.0.0.1", () => {
  const url = `http://127.0.0.1:${PORT}`;
  console.log(`Lunar Robot Console (read-only) listening at ${url}`);
  if (process.env.LUNAR_CONSOLE_OPEN_BROWSER === "1") {
    const opener = spawn("open", [url], { stdio: "ignore", detached: true });
    opener.unref();
  }
});
