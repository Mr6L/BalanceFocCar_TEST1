import asyncio
import json
import socket
import threading
from datetime import datetime

import requests
from flask import Flask, Response, jsonify, request

try:
    from bleak import BleakClient, BleakScanner
    BLE_AVAILABLE = True
except Exception:
    BleakClient = None
    BleakScanner = None
    BLE_AVAILABLE = False


app = Flask(__name__)

DEFAULT_ESP32_IP = "192.168.4.1"
REQUEST_TIMEOUT = 0.8

BLE_DEVICE_NAME = "HC-04BLE"
BLE_SERVICE_UUID = "0000FFE0-0000-1000-8000-00805F9B34FB"
BLE_CHAR_UUID = "0000FFE1-0000-1000-8000-00805F9B34FB"

BLE_MAX_THROTTLE = 0.70
BLE_MAX_TURN = 0.80
BLE_MANUAL_DURATION_MS = 420


def get_local_ip():
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        local_ip = sock.getsockname()[0]
        sock.close()
        return local_ip
    except Exception:
        return "127.0.0.1"


def request_payload():
    data = request.get_json(silent=True)
    if isinstance(data, dict):
        return data
    if request.form:
        return request.form.to_dict()
    return request.args.to_dict()


def esp32_base(source=None):
    source = source or request_payload()
    ip = source.get("ip") or request.args.get("ip") or request.form.get("ip") or DEFAULT_ESP32_IP
    ip = str(ip).strip().replace("http://", "").replace("https://", "").rstrip("/")
    return f"http://{ip}"


def proxy_get(path, params=None):
    source = dict(params if params is not None else request_payload())
    clean_params = dict(source)
    clean_params.pop("ip", None)

    try:
        resp = requests.get(
            f"{esp32_base(source)}{path}",
            params=clean_params,
            timeout=REQUEST_TIMEOUT,
        )
        return Response(
            resp.content,
            status=resp.status_code,
            content_type=resp.headers.get("content-type", "text/plain"),
        )
    except Exception as exc:
        return Response(
            json.dumps({"error": str(exc)}, ensure_ascii=False),
            status=502,
            content_type="application/json",
        )


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_ble_command(cmd):
    cmd = str(cmd or "").strip()
    if not cmd:
        raise RuntimeError("蓝牙命令为空")
    if cmd.startswith("[") and cmd.endswith("]"):
        return cmd
    return f"[{cmd}]"


class BleController:
    def __init__(self):
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

        self.lock = threading.Lock()
        self.client = None
        self.discovered_devices = {}

        self.state = {
            "available": BLE_AVAILABLE,
            "scanning": False,
            "connected": False,
            "device_name": "",
            "device_address": "",
            "last_command": "",
            "last_command_time": "",
            "last_notify": "",
            "last_notify_time": "",
            "last_json": None,
            "last_error": "" if BLE_AVAILABLE else "未安装 bleak",
            "devices": [],
            "log": [],
        }

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def run(self, coro, timeout=15):
        future = asyncio.run_coroutine_threadsafe(coro, self.loop)
        return future.result(timeout=timeout)

    def add_log(self, message):
        line = f"[{datetime.now().strftime('%H:%M:%S')}] {message}"
        print(line)
        with self.lock:
            self.state["log"].insert(0, line)
            self.state["log"] = self.state["log"][:120]

    def set_error(self, message):
        with self.lock:
            self.state["last_error"] = str(message)
        self.add_log("错误: " + str(message))

    def snapshot(self):
        with self.lock:
            return dict(self.state)

    def on_disconnect(self, client):
        with self.lock:
            self.state["connected"] = False
        self.add_log("蓝牙已断开")

    def on_notify(self, sender, data):
        try:
            text = data.decode("utf-8", errors="ignore")
        except Exception:
            text = str(data)

        parsed = None
        payload = text
        if payload.startswith("OK:"):
            payload = payload[3:]
        if payload.startswith("{") and payload.endswith("}"):
            try:
                parsed = json.loads(payload)
            except Exception:
                parsed = None

        with self.lock:
            self.state["last_notify"] = text
            self.state["last_notify_time"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.state["last_json"] = parsed

        self.add_log(f"通知 <- {text}")

    async def scan(self):
        if not BLE_AVAILABLE:
            raise RuntimeError("未安装 bleak")

        with self.lock:
            self.state["scanning"] = True
            self.state["last_error"] = ""

        self.add_log("正在扫描蓝牙设备")

        try:
            devices = await BleakScanner.discover(timeout=5.0)
            rows = []
            self.discovered_devices = {}

            for device in devices:
                name = device.name or ""
                rows.append({"name": name, "address": device.address})
                self.discovered_devices[device.address] = device

            rows.sort(
                key=lambda item: (
                    0 if item["name"] == BLE_DEVICE_NAME else 1,
                    item["name"],
                    item["address"],
                )
            )

            with self.lock:
                self.state["scanning"] = False
                self.state["devices"] = rows

            self.add_log(f"扫描完成，找到 {len(rows)} 个设备")
            return {"ok": True, "devices": rows}
        except Exception as exc:
            with self.lock:
                self.state["scanning"] = False
            self.set_error(exc)
            return {"ok": False, "error": str(exc)}

    async def connect(self, address=""):
        if not BLE_AVAILABLE:
            raise RuntimeError("未安装 bleak")

        with self.lock:
            self.state["last_error"] = ""

        try:
            if self.client is not None and self.client.is_connected:
                return {"ok": True, "message": "已连接"}

            target = None
            target_address = str(address or "").strip()

            if target_address:
                target = self.discovered_devices.get(target_address, target_address)
            else:
                self.add_log(f"正在搜索 {BLE_DEVICE_NAME}")
                devices = await BleakScanner.discover(timeout=5.0)
                for device in devices:
                    if device.name == BLE_DEVICE_NAME:
                        target = device
                        target_address = device.address
                        break
                if target is None:
                    raise RuntimeError(f"找不到蓝牙设备: {BLE_DEVICE_NAME}")

            self.add_log(f"正在连接 {target_address}")
            self.client = BleakClient(target, disconnected_callback=self.on_disconnect, timeout=15.0)
            await self.client.connect()

            if not self.client.is_connected:
                raise RuntimeError("蓝牙连接失败")

            try:
                await self.client.start_notify(BLE_CHAR_UUID, self.on_notify)
                self.add_log("通知已开启")
            except Exception as exc:
                self.add_log(f"通知开启失败: {exc}")

            with self.lock:
                self.state["connected"] = True
                self.state["device_name"] = BLE_DEVICE_NAME
                self.state["device_address"] = target_address

            self.add_log("蓝牙已连接")
            return {"ok": True, "address": target_address}
        except Exception as exc:
            self.set_error(exc)
            with self.lock:
                self.state["connected"] = False
            return {"ok": False, "error": str(exc)}

    async def disconnect(self):
        try:
            if self.client is not None and self.client.is_connected:
                try:
                    await self.client.stop_notify(BLE_CHAR_UUID)
                except Exception:
                    pass
                await self.client.disconnect()

            with self.lock:
                self.state["connected"] = False

            self.add_log("已请求断开蓝牙")
            return {"ok": True}
        except Exception as exc:
            self.set_error(exc)
            return {"ok": False, "error": str(exc)}

    async def send(self, command, wait_response=True):
        command = normalize_ble_command(command)

        if self.client is None or not self.client.is_connected:
            raise RuntimeError("蓝牙未连接")

        self.add_log(f"发送 -> {command}")
        data = command.encode("utf-8")

        if wait_response:
            try:
                await self.client.write_gatt_char(BLE_CHAR_UUID, data, response=True)
            except Exception:
                await self.client.write_gatt_char(BLE_CHAR_UUID, data, response=False)
        else:
            try:
                await self.client.write_gatt_char(BLE_CHAR_UUID, data, response=False)
            except Exception:
                await self.client.write_gatt_char(BLE_CHAR_UUID, data, response=True)

        with self.lock:
            self.state["last_command"] = command
            self.state["last_command_time"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        return {"ok": True, "cmd": command}


ble = BleController()


PAGE = """
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>平衡车控制台</title>
<style>
:root{
  --bg:#eef2f6;--panel:#fff;--line:#cfd8e3;--text:#111827;
  --muted:#5f6f82;--blue:#1f6feb;--green:#12805c;--red:#bd1e3c;--amber:#9a6700;
}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--text);font-family:Arial,"Microsoft YaHei",sans-serif}
header{padding:14px 18px;border-bottom:1px solid var(--line);background:#f8fafc}
h1{font-size:21px;margin:0}
main{padding:14px;display:grid;grid-template-columns:360px 1fr;gap:14px}
.section{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:14px}
.section h2{font-size:16px;margin:0 0 12px}
.stack{display:flex;flex-direction:column;gap:10px}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.pid-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}
.driveGrid{display:grid;grid-template-columns:90px 90px 90px;grid-template-rows:54px 54px 54px;gap:8px}
label{font-size:13px;color:var(--muted);display:block;margin-bottom:4px}
input,select,button{font-size:14px;border:1px solid #b8c5d4;border-radius:6px;padding:9px 10px}
input,select{width:100%;background:#fff}
button{cursor:pointer;font-weight:700;background:#f8fafc}
button.primary{background:var(--blue);border-color:var(--blue);color:#fff}
button.green{background:var(--green);border-color:var(--green);color:#fff}
button.red{background:#fff1f2;border-color:#fecdd3;color:var(--red)}
button.amber{background:#fffbeb;border-color:#fde68a;color:var(--amber)}
button.drive{width:90px;height:54px}
.status{padding:10px;border:1px solid var(--line);border-radius:8px;background:#f8fafc;line-height:1.6;font-size:14px}
.controlTop{display:grid;grid-template-columns:1fr auto;gap:12px;align-items:start}
.batteryBox{min-width:190px;padding:10px 12px;border:1px solid var(--line);border-radius:8px;background:#f8fafc;text-align:right}
.batteryMain{font-size:18px;font-weight:700}
.batteryBar{height:8px;background:#dbe3ee;border-radius:99px;margin:8px 0;overflow:hidden}
.batteryFill{height:100%;width:0%;background:var(--green)}
.batteryBox.low .batteryFill{background:var(--amber)}
.batteryBox.critical .batteryFill{background:var(--red)}
main > .section.full:first-child{position:relative}
main > .section.full:first-child > .grid2{padding-right:210px}
main > .section.full:first-child .batteryBox{position:absolute;right:14px;top:42px}
.ok{color:var(--green);font-weight:700}
.bad{color:var(--red);font-weight:700}
.muted{color:var(--muted);font-size:13px}
pre{margin:0;min-height:180px;background:#0f172a;color:#e5e7eb;border-radius:8px;padding:12px;white-space:pre-wrap;overflow:auto}
.full{grid-column:1 / -1}
@media(max-width:1000px){main{grid-template-columns:1fr}.pid-grid{grid-template-columns:1fr 1fr}}
@media(max-width:620px){.pid-grid,.grid2,.controlTop{grid-template-columns:1fr}.batteryBox{text-align:left}main > .section.full:first-child > .grid2{padding-right:0}main > .section.full:first-child .batteryBox{position:static;margin-top:10px}}
</style>
</head>

<body>
<header>
  <h1>平衡车 WiFi + 蓝牙控制台</h1>
</header>

<main>
  <section class="section full">
    <h2>当前控制</h2>
    <div class="grid2">
      <div>
        <label>控制通道</label>
        <select id="controlMode" onchange="switchControlMode(this.value)">
          <option value="ble" selected>蓝牙 BLE</option>
          <option value="wifi">WiFi 控制</option>
        </select>
      </div>
      <div class="status">
        当前: <span id="controlModeText" class="ok">蓝牙 BLE</span><br>
        <span class="muted">切换控制方式时会先发送停车命令。</span>
      </div>
    </div>
    <div id="batteryBox" class="batteryBox">
      <div class="batteryMain"><span id="batteryVoltage">--.--</span> V</div>
      <div><span id="batteryPercent">--</span>%</div>
      <div class="batteryBar"><div id="batteryFill" class="batteryFill"></div></div>
      <div id="batteryDetail" class="muted">正在读取电量...</div>
    </div>
  </section>

  <section class="section">
    <h2>蓝牙</h2>
    <div class="stack">
      <div class="status">
        <div>状态: <span id="bleConnected" class="bad">未连接</span></div>
        <div>设备: <span id="bleDevice">-</span></div>
        <div>地址: <span id="bleAddress">-</span></div>
        <div>上一条命令: <span id="lastCommand">-</span></div>
        <div>上一条通知: <span id="lastNotify">-</span></div>
        <div>错误: <span id="lastError">-</span></div>
      </div>

      <div>
        <label>蓝牙设备</label>
        <select id="bleDeviceSelect">
          <option value="">请先扫描</option>
        </select>
      </div>

      <div class="row">
        <button class="primary" onclick="scanBle()">扫描</button>
        <button class="green" onclick="connectBle()">连接</button>
        <button onclick="disconnectBle()">断开</button>
        <button class="primary" onclick="switchControlMode('ble')">使用蓝牙</button>
      </div>

      <div class="row">
        <button onclick="sendHello()">测试连接</button>
        <button onclick="carStatus()">小车状态</button>
        <button class="red" onclick="stopCar()">停车</button>
      </div>
    </div>
  </section>

  <section class="section">
    <h2>行驶控制</h2>
    <div class="stack">
      <div class="status">
        前进速度: <b id="driveSpeedText">0.30</b><br>
        转向速度: <b id="turnSpeedText">0.20</b>
      </div>

      <div class="grid2">
        <div>
          <label>转向速度比例</label>
          <input id="turnSpeedSlider" type="range" min="0.20" max="0.60" step="0.01" value="0.20" oninput="updateDriveSliders()">
        </div>
      </div>

      <div class="driveGrid">
        <div></div>
        <button class="drive primary" onmousedown="holdDrive(1,0)" onmouseup="releaseDrive()" onmouseleave="releaseDrive()" ontouchstart="touchDrive(event,1,0)" ontouchend="releaseDrive()">前进<br>W</button>
        <div></div>

        <button class="drive" onmousedown="holdDrive(0,1)" onmouseup="releaseDrive()" onmouseleave="releaseDrive()" ontouchstart="touchDrive(event,0,1)" ontouchend="releaseDrive()">左转<br>A</button>
        <button class="drive red" onclick="stopCar()">停车<br>空格</button>
        <button class="drive" onmousedown="holdDrive(0,-1)" onmouseup="releaseDrive()" onmouseleave="releaseDrive()" ontouchstart="touchDrive(event,0,-1)" ontouchend="releaseDrive()">右转<br>D</button>

        <button class="drive amber" onmousedown="holdDrive(0,1)" onmouseup="releaseDrive()" onmouseleave="releaseDrive()" ontouchstart="touchDrive(event,0,1)" ontouchend="releaseDrive()">左旋<br>Q</button>
        <button class="drive primary" onmousedown="holdDrive(-1,0)" onmouseup="releaseDrive()" onmouseleave="releaseDrive()" ontouchstart="touchDrive(event,-1,0)" ontouchend="releaseDrive()">后退<br>S</button>
        <button class="drive amber" onmousedown="holdDrive(0,-1)" onmouseup="releaseDrive()" onmouseleave="releaseDrive()" ontouchstart="touchDrive(event,0,-1)" ontouchend="releaseDrive()">右旋<br>E</button>
      </div>
    </div>
  </section>

  <section class="section">
    <h2>WiFi</h2>
    <div class="stack">
      <div>
        <label>ESP32 地址</label>
        <input id="espIp" value="192.168.4.1">
      </div>
      <div class="row">
        <button onclick="wifiStatus()">WiFi 状态</button>
        <button onclick="wifiStop()">WiFi 停车</button>
        <button class="primary" onclick="switchControlMode('wifi')">使用 WiFi</button>
      </div>
    </div>
  </section>

  <section class="section full">
    <h2>控制参数调试</h2>
    <div class="pid-grid">
      <div><label>直立环 Kp</label><input id="upright_kp" type="number" step="0.001"></div>
      <div><label>直立环 Ki</label><input id="upright_ki" type="number" step="0.001"></div>
      <div><label>直立环 Kd</label><input id="upright_kd" type="number" step="0.001"></div>
      <div><label>直立输出限幅</label><input id="upright_limit" type="number" step="0.1"></div>

      <div><label>速度外环 Kp</label><input id="velocity_kp" type="number" step="0.001"></div>
      <div><label>速度外环 Ki</label><input id="velocity_ki" type="number" step="0.001"></div>
      <div><label>速度外环 Kd</label><input id="velocity_kd" type="number" step="0.001"></div>
      <div>
        <label>速度外环开关</label>
        <select id="velocity_loop">
          <option value="1">开启</option>
          <option value="0">关闭</option>
        </select>
      </div>

      <div><label>电机速度环 Kp</label><input id="motor_vel_kp" type="number" step="0.001"></div>
      <div><label>电机速度环 Ki</label><input id="motor_vel_ki" type="number" step="0.001"></div>
      <div><label>电机速度滤波</label><input id="motor_lpf" type="number" step="0.001"></div>
      <div><label>电机电压限幅</label><input id="voltage_limit" type="number" step="0.01"></div>

      <div><label>平衡角偏置</label><input id="target_angle_offset" type="number" step="0.01"></div>
      <div><label>目标倾角限幅</label><input id="target_angle_limit" type="number" step="0.1"></div>
      <div><label>内部目标速度上限</label><input id="target_velocity_limit" type="number" step="0.1"></div>
      <div>
        <label>速度反馈方向</label>
        <select id="velocity_feedback_sign">
          <option value="1">1</option>
          <option value="-1">-1</option>
        </select>
      </div>
      <div><label>遥控最大速度</label><input id="drive_max_velocity" type="number" step="0.1"></div>

      <div><label>速度阻尼 Kp</label><input id="direct_speed_damping_kp" type="number" step="0.01"></div>
      <div><label>速度阻尼限幅</label><input id="direct_speed_damping_limit" type="number" step="0.1"></div>
      <div><label>速度阻尼死区</label><input id="direct_speed_damping_deadband" type="number" step="0.01"></div>
      <div><label>速度阻尼滤波</label><input id="direct_speed_damping_filter_alpha" type="number" step="0.01"></div>
      <div><label>遥控最大转向</label><input id="drive_max_steer" type="number" step="0.1"></div>
      <div><label>速度斜坡步长</label><input id="drive_vel_ramp_step" type="number" step="0.01"></div>
      <div><label>刹车斜坡步长</label><input id="drive_brake_ramp_step" type="number" step="0.01"></div>
      <div><label>转向斜坡步长</label><input id="drive_steer_ramp_step" type="number" step="0.01"></div>
      <div><label>BLE 起步斜坡 ms</label><input id="ble_start_ramp_ms" type="number" step="10"></div>

      <div><label>BLE 起步最小比例</label><input id="ble_start_min_scale" type="number" step="0.01"></div>
    </div>

    <div class="row" style="margin-top:12px">
      <button onclick="readPid()">读取PID</button>
      <button class="primary" onclick="applyPid()">应用PID</button>
      <button class="red" onclick="stopCar()">紧急停车</button>
      <span class="muted" id="pidEditText">自动刷新开启</span>
    </div>

    <div class="status" id="pidStatus" style="margin-top:10px">状态: -</div>
  </section>

  <section class="section full">
    <h2>蓝牙原始命令</h2>
    <div class="row">
      <input id="rawCommand" value="[Car,status]" style="max-width:420px">
      <button class="primary" onclick="sendRaw()">发送</button>
      <button class="red" onclick="powerOff()">关机</button>
    </div>
  </section>

  <section class="section">
    <h2>响应</h2>
    <pre id="responseBox">就绪。</pre>
  </section>

  <section class="section">
    <h2>日志</h2>
    <pre id="logBox"></pre>
  </section>
</main>

<script>
const SEND_INTERVAL_MS = 180;
const COMMAND_DURATION_MS = 650;
const DEFAULT_THROTTLE = 0.30;
const DEFAULT_TURN = 0.20;

let activeDrive = {throttleSign:0, turnSign:0};
let driveTimer = null;
let driveSendBusy = false;
let keys = {};
let controlMode = "ble";
let pidEditing = false;
let pidTimer = null;
let lastBleBatteryRequestMs = 0;
let lastBatteryVoltage = null;
let lastBatteryUpdatedAt = "";

const pidFields = [
  "upright_kp","upright_ki","upright_kd","upright_limit",
  "velocity_kp","velocity_ki","velocity_kd","velocity_loop",
  "motor_vel_kp","motor_vel_ki","motor_lpf","voltage_limit",
  "target_angle_offset","target_angle_limit","target_velocity_limit","velocity_feedback_sign",
  "direct_speed_damping_kp","direct_speed_damping_limit",
  "direct_speed_damping_deadband","direct_speed_damping_filter_alpha",
  "drive_max_velocity","drive_max_steer","drive_vel_ramp_step","drive_brake_ramp_step",
  "drive_steer_ramp_step","ble_start_ramp_ms","ble_start_min_scale"
];

function sliderNumber(id, fallback){
  const el = document.getElementById(id);
  if(!el) return fallback;
  const value = Number(el.value);
  return Number.isFinite(value) ? value : fallback;
}

function throttleValue(){ return DEFAULT_THROTTLE; }
function turnValue(){ return sliderNumber("turnSpeedSlider", DEFAULT_TURN); }
function espIp(){ return document.getElementById("espIp").value.trim() || "192.168.4.1"; }

function updateDriveSliders(){
  const driveText = document.getElementById("driveSpeedText");
  const turnText = document.getElementById("turnSpeedText");
  if(driveText) driveText.textContent = DEFAULT_THROTTLE.toFixed(2);
  if(turnText) turnText.textContent = turnValue().toFixed(2);
}

function setInputValue(id, value, digits=3){
  const el = document.getElementById(id);
  if(!el || value === undefined || value === null) return;
  el.value = Number(value).toFixed(digits);
}

function showResponse(data){
  document.getElementById("responseBox").textContent =
    typeof data === "string" ? data : JSON.stringify(data, null, 2);
}

async function postJson(url, body={}){
  const res = await fetch(url, {
    method:"POST",
    headers:{"Content-Type":"application/json"},
    body:JSON.stringify(body)
  });
  const data = await res.json();
  showResponse(data);
  return data;
}

async function postJsonQuiet(url, body={}){
  const res = await fetch(url, {
    method:"POST",
    headers:{"Content-Type":"application/json"},
    body:JSON.stringify(body)
  });
  return await res.json();
}

async function getJson(url){
  const res = await fetch(url);
  const text = await res.text();
  try{
    const data = JSON.parse(text);
    showResponse(data);
    return data;
  }catch{
    showResponse(text);
    return text;
  }
}

function updateControlModeUi(){
  const select = document.getElementById("controlMode");
  const text = document.getElementById("controlModeText");
  if(select) select.value = controlMode;
  if(text) text.textContent = controlMode === "wifi" ? "WiFi 控制" : "蓝牙 BLE";
}

async function switchControlMode(mode){
  mode = mode === "wifi" ? "wifi" : "ble";
  if(mode === controlMode){
    updateControlModeUi();
    return;
  }
  await stopCar();
  controlMode = mode;
  updateControlModeUi();
  if(controlMode === "wifi") await wifiStatus();
  else await refreshBleStatus();
  await refreshBattery();
}

async function wifiDrive(throttle, turn){
  const url =
    `/drive?ip=${encodeURIComponent(espIp())}` +
    `&throttle=${encodeURIComponent(throttle.toFixed(3))}` +
    `&turn=${encodeURIComponent(turn.toFixed(3))}`;
  return await getJson(url);
}

async function refreshBleStatus(){
  try{
    const data = await fetch("/ble/status").then(r => r.json());

    const connected = document.getElementById("bleConnected");
    connected.textContent = data.connected ? "已连接" : "未连接";
    connected.className = data.connected ? "ok" : "bad";

    document.getElementById("bleDevice").textContent = data.device_name || "-";
    document.getElementById("bleAddress").textContent = data.device_address || "-";
    document.getElementById("lastCommand").textContent = data.last_command || "-";
    document.getElementById("lastNotify").textContent = data.last_notify || "-";
    document.getElementById("lastError").textContent = data.last_error || "-";
    document.getElementById("logBox").textContent = (data.log || []).join("\\n");

    if(data.last_json && (data.last_json.battery !== undefined || data.last_json.voltage !== undefined)){
      applyBatteryDisplay(Number(data.last_json.battery !== undefined ? data.last_json.battery : data.last_json.voltage));
    }

    const select = document.getElementById("bleDeviceSelect");
    const selected = select.value;
    if(data.devices && data.devices.length){
      select.innerHTML = "";
      for(const dev of data.devices){
        const option = document.createElement("option");
        option.value = dev.address;
        option.textContent = `${dev.name || "(未知设备)"} | ${dev.address}`;
        select.appendChild(option);
      }
      if(selected) select.value = selected;
    }
  }catch(e){
    console.log(e);
  }
}

async function scanBle(){
  await postJson("/ble/scan");
  await refreshBleStatus();
}

async function connectBle(){
  const address = document.getElementById("bleDeviceSelect").value;
  await postJson("/ble/connect", {address});
  await refreshBleStatus();
}

async function disconnectBle(){
  await postJson("/ble/disconnect");
  await refreshBleStatus();
}

async function sendHello(){ await postJson("/ble/send", {cmd:"[hello world]"}); }

async function carStatus(){
  if(controlMode === "wifi") await getJson(`/drive/status?ip=${encodeURIComponent(espIp())}`);
  else await postJson("/ble/car/status");
}

async function stopCar(){
  stopDriveTimer();
  if(controlMode === "wifi") await getJson(`/drive?ip=${encodeURIComponent(espIp())}&stop=1`);
  else await postJson("/ble/stop");
}

async function sendDrive(throttle, turn){
  if(driveSendBusy) return;
  driveSendBusy = true;
  try{
    if(controlMode === "wifi"){
      const url =
        `/drive?ip=${encodeURIComponent(espIp())}` +
        `&throttle=${encodeURIComponent(throttle.toFixed(3))}` +
        `&turn=${encodeURIComponent(turn.toFixed(3))}`;
      await fetch(url);
      return;
    }

    await postJsonQuiet("/ble/drive", {
      throttle,
      turn,
      duration_ms: COMMAND_DURATION_MS
    });
  }catch(e){
    console.log(e);
  }finally{
    driveSendBusy = false;
  }
}

function computeDrive(){
  let throttle = activeDrive.throttleSign * throttleValue();
  let turn = activeDrive.turnSign * turnValue();

  if(activeDrive.throttleSign < 0){
    turn = -turn;
  }

  return {throttle, turn};
}

function startDriveTimer(){
  if(driveTimer) return;
  driveTimer = setInterval(async () => {
    const d = computeDrive();
    if(Math.abs(d.throttle) > 0 || Math.abs(d.turn) > 0){
      await sendDrive(d.throttle, d.turn);
    }
  }, SEND_INTERVAL_MS);
}

function stopDriveTimer(){
  if(driveTimer){
    clearInterval(driveTimer);
    driveTimer = null;
  }
  activeDrive = {throttleSign:0, turnSign:0};
}

function holdDrive(throttleSign, turnSign){
  activeDrive = {throttleSign, turnSign};
  const d = computeDrive();
  sendDrive(d.throttle, d.turn);
  startDriveTimer();
}

function releaseDrive(){
  stopDriveTimer();
  stopCar();
}

function touchDrive(event, throttleSign, turnSign){
  event.preventDefault();
  holdDrive(throttleSign, turnSign);
}

function updateKeyboardDrive(){
  let throttleSign = 0;
  let turnSign = 0;

  if(keys.w) throttleSign += 1;
  if(keys.s) throttleSign -= 1;
  if(keys.a) turnSign += 1;
  if(keys.d) turnSign -= 1;
  if(keys.q){ throttleSign = 0; turnSign = 1; }
  if(keys.e){ throttleSign = 0; turnSign = -1; }

  if(throttleSign === 0 && turnSign === 0){
    releaseDrive();
    return;
  }

  activeDrive = {throttleSign, turnSign};
  const d = computeDrive();
  sendDrive(d.throttle, d.turn);
  startDriveTimer();
}

document.addEventListener("keydown", event => {
  const key = event.key.toLowerCase();
  if(["w","a","s","d","q","e"].includes(key)){
    if(!keys[key]){
      keys[key] = true;
      updateKeyboardDrive();
    }
    event.preventDefault();
  }

  if(event.code === "Space"){
    keys = {};
    stopCar();
    event.preventDefault();
  }
});

document.addEventListener("keyup", event => {
  const key = event.key.toLowerCase();
  if(keys[key]){
    delete keys[key];
    updateKeyboardDrive();
    event.preventDefault();
  }
});

async function sendRaw(){
  const cmd = document.getElementById("rawCommand").value.trim();
  await postJson("/ble/send", {cmd});
}

async function powerOff(){ await postJson("/ble/power/off"); }
async function wifiStatus(){ await getJson(`/wifi/status?ip=${encodeURIComponent(espIp())}`); }
async function wifiStop(){ await getJson(`/drive?ip=${encodeURIComponent(espIp())}&stop=1`); }

function bindPidEditEvents(){
  for(const key of pidFields){
    const el = document.getElementById(key);
    if(!el) continue;
    el.addEventListener("focus", () => {
      pidEditing = true;
      updatePidEditText();
    });
    el.addEventListener("input", () => {
      pidEditing = true;
      updatePidEditText();
    });
  }
}

function updatePidEditText(){
  const text = document.getElementById("pidEditText");
  if(!text) return;
  text.textContent = pidEditing ? "正在编辑，已暂停自动覆盖输入框" : "自动刷新开启";
}

function formatPidValue(key, value){
  if(value === undefined || value === null) return "";
  if(key === "velocity_loop") return Number(value) ? "1" : "0";
  if(key === "velocity_feedback_sign") return Number(value) >= 0 ? "1" : "-1";
  if(key === "ble_start_ramp_ms") return String(Math.round(Number(value)));
  if(typeof value === "number") return value.toFixed(4);
  return String(value);
}

async function loadPid(force=false){
  if(pidEditing && !force) return;

  const data = await getJson(`/pid?ip=${encodeURIComponent(espIp())}`);
  if(!data || typeof data !== "object") return;

  for(const key of pidFields){
    const el = document.getElementById(key);
    if(!el || data[key] === undefined) continue;
    el.value = formatPidValue(key, data[key]);
  }

  const status = document.getElementById("pidStatus");
  if(status){
    const angle = Number(data.angle_x || 0).toFixed(2);
    const targetAngle = Number(data.target_angle || 0).toFixed(2);
    const targetVelocity = Number(data.target_velocity || 0).toFixed(2);
    const carVelocity = Number(data.car_velocity || 0).toFixed(2);
    const velocityError = Number(data.velocity_error || 0).toFixed(2);

    status.textContent =
      `状态: ${data.run_state_name || "-"} | ` +
      `角度: ${angle} | ` +
      `目标角: ${targetAngle} | ` +
      `目标速度: ${targetVelocity} | ` +
      `车速: ${carVelocity} | ` +
      `速度误差: ${velocityError}`;
  }

  if(force){
    pidEditing = false;
    updatePidEditText();
  }
}

async function readPid(){
  pidEditing = false;
  updatePidEditText();
  await loadPid(true);
}

async function applyPid(){
  let url = `/pid/set?ip=${encodeURIComponent(espIp())}`;

  for(const key of pidFields){
    const el = document.getElementById(key);
    if(!el) continue;
    const value = String(el.value || "").trim();
    if(value !== ""){
      url += `&${encodeURIComponent(key)}=${encodeURIComponent(value)}`;
    }
  }

  const data = await getJson(url);

  pidEditing = false;
  updatePidEditText();
  await loadPid(true);

  return data;
}

function updatePidEditText(){
  const text = document.getElementById("pidEditText");
  if(!text) return;
  text.textContent = pidEditing ? "正在编辑，已暂停自动覆盖输入框" : "自动刷新开启";
}

async function loadPid(force=false){
  if(pidEditing && !force) return;

  const data = await getJson(`/pid?ip=${encodeURIComponent(espIp())}`);
  if(!data || typeof data !== "object") return;

  for(const key of pidFields){
    const el = document.getElementById(key);
    if(!el || data[key] === undefined) continue;
    el.value = formatPidValue(key, data[key]);
  }

  const status = document.getElementById("pidStatus");
  if(status){
    const angle = Number(data.angle_x || 0).toFixed(2);
    const targetAngle = Number(data.target_angle || 0).toFixed(2);
    const targetVelocity = Number(data.target_velocity || 0).toFixed(2);
    const carVelocity = Number(data.car_velocity || 0).toFixed(2);
    const velocityError = Number(data.velocity_error || 0).toFixed(2);
    const speedDamping = Number(data.speed_damping || 0).toFixed(2);

    status.textContent =
      `状态: ${data.run_state_name || "-"} | ` +
      `角度: ${angle} | ` +
      `目标角: ${targetAngle} | ` +
      `目标速度: ${targetVelocity} | ` +
      `车速: ${carVelocity} | ` +
      `速度误差: ${velocityError} | ` +
      `速度阻尼: ${speedDamping}`;
  }

  if(force){
    pidEditing = false;
    updatePidEditText();
  }
}

function batteryPercentFromVoltage(voltage){
  // 3S Li-ion
  const fullVoltage = 12.6;
  const warnVoltage = 10.8;
  const zeroVoltage = 9.0;
  const warnPercent = 20;

  if(voltage >= fullVoltage) return 100;
  if(voltage <= zeroVoltage) return 0;

  if(voltage <= warnVoltage){
    return Math.round((voltage - zeroVoltage) * warnPercent / (warnVoltage - zeroVoltage));
  }

  return Math.round(
    warnPercent +
    (voltage - warnVoltage) * (100 - warnPercent) / (fullVoltage - warnVoltage)
  );
}

function batteryLevelFromVoltage(voltage){
  if(voltage <= 9.9) return "critical";
  if(voltage <= 10.8) return "low";
  return "normal";
}

function batteryLevelText(level){
  if(level === "critical") return "严重低电";
  if(level === "low") return "低电量";
  return "正常";
}

function applyBatteryDisplay(voltage){
  if(!Number.isFinite(voltage) || voltage <= 0){
    return false;
  }

  const percent = Math.max(0, Math.min(100, batteryPercentFromVoltage(voltage)));
  const level = batteryLevelFromVoltage(voltage);
  lastBatteryVoltage = voltage;
  lastBatteryUpdatedAt = new Date().toLocaleTimeString();

  document.getElementById("batteryVoltage").textContent = voltage.toFixed(2);
  document.getElementById("batteryPercent").textContent = percent.toFixed(0);
  document.getElementById("batteryFill").style.width = `${percent}%`;

  const box = document.getElementById("batteryBox");
  box.classList.remove("low", "critical");
  if(level === "critical") box.classList.add("critical");
  else if(level === "low") box.classList.add("low");

  document.getElementById("batteryDetail").textContent =
    `${batteryLevelText(level)} | 单节 ${(voltage / 2).toFixed(2)}V | ${lastBatteryUpdatedAt}`;
  return true;
}

async function requestBleBatteryStatus(){
  const now = Date.now();
  if(now - lastBleBatteryRequestMs < 1500) return;
  lastBleBatteryRequestMs = now;

  try{
    await fetch("/ble/car/status", {method:"POST"});
  }catch(e){
    console.log(e);
  }
}

async function refreshBattery(){
  try{
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), 700);
    const res = await fetch(`/battery?ip=${encodeURIComponent(espIp())}`, {signal: controller.signal});
    clearTimeout(timer);

    if(res.ok){
      const data = await res.json();
      if(applyBatteryDisplay(Number(data.voltage || 0))){
        return;
      }
    }
  }catch(e){
    console.log("WiFi battery unavailable", e);
  }

  try{
    const state = await fetch("/ble/status").then(r => r.json());
    const last = state.last_json || {};
    const voltage = Number(last.battery !== undefined ? last.battery : (last.voltage !== undefined ? last.voltage : 0));
    if(applyBatteryDisplay(voltage)){
      return;
    }

    if(controlMode === "ble" && state.connected){
      if(lastBatteryVoltage === null){
        document.getElementById("batteryDetail").textContent = "等待蓝牙电量...";
      }
      await requestBleBatteryStatus();
      return;
    }
  }catch(e){
    console.log("BLE battery unavailable", e);
  }

  if(lastBatteryVoltage === null){
    document.getElementById("batteryDetail").textContent = "暂无电量数据";
  }else{
    document.getElementById("batteryDetail").textContent =
      `保持上次电量 | ${lastBatteryUpdatedAt}`;
  }
}

function startPidAutoRefresh(){
  bindPidEditEvents();
  loadPid(true);

  if(pidTimer) clearInterval(pidTimer);

  pidTimer = setInterval(() => {
    loadPid(false);
  }, 1200);
}

window.addEventListener("load", () => {
  updateControlModeUi();
  refreshBleStatus();
  refreshBattery();
  startPidAutoRefresh();
  updateDriveSliders();

  setInterval(refreshBleStatus, 1000);
  setInterval(refreshBattery, 2000);
});
</script>
</body>
</html>
"""

@app.route("/")
def index():
    return PAGE


@app.route("/pid", methods=["GET"])
def pid():
    return proxy_get("/pid")


@app.route("/pid/set", methods=["GET", "POST"])
def pid_set():
    return proxy_get("/pid/set")


@app.route("/drive", methods=["GET"])
def drive():
    return proxy_get("/drive")


@app.route("/drive/status", methods=["GET"])
def drive_status():
    return proxy_get("/drive/status")


@app.route("/distance", methods=["GET"])
def distance():
    return proxy_get("/distance")


@app.route("/distance/stop", methods=["GET"])
def distance_stop():
    return proxy_get("/distance/stop")


@app.route("/distance/status", methods=["GET"])
def distance_status():
    return proxy_get("/distance/status")


@app.route("/distance/config", methods=["GET"])
def distance_config():
    return proxy_get("/distance/config")


@app.route("/wifi/status", methods=["GET"])
def wifi_status():
    return proxy_get("/wifi/status")


@app.route("/battery", methods=["GET"])
def battery():
    return proxy_get("/battery")


@app.route("/ble/status", methods=["GET"])
def ble_status():
    return jsonify(ble.snapshot())


@app.route("/ble/scan", methods=["POST"])
def ble_scan():
    try:
        return jsonify(ble.run(ble.scan(), timeout=10))
    except Exception as exc:
        ble.set_error(exc)
        return jsonify({"ok": False, "error": str(exc)})


@app.route("/ble/connect", methods=["POST"])
def ble_connect():
    try:
        data = request_payload()
        address = data.get("address", "")
        return jsonify(ble.run(ble.connect(address), timeout=20))
    except Exception as exc:
        ble.set_error(exc)
        return jsonify({"ok": False, "error": str(exc)})


@app.route("/ble/disconnect", methods=["POST"])
def ble_disconnect():
    try:
        return jsonify(ble.run(ble.disconnect(), timeout=10))
    except Exception as exc:
        ble.set_error(exc)
        return jsonify({"ok": False, "error": str(exc)})


@app.route("/ble/send", methods=["POST"])
def ble_send():
    try:
        data = request_payload()
        return jsonify(ble.run(ble.send(data.get("cmd", "")), timeout=10))
    except Exception as exc:
        ble.set_error(exc)
        return jsonify({"ok": False, "error": str(exc)})


@app.route("/ble/car/status", methods=["POST"])
def ble_car_status():
    try:
        return jsonify(ble.run(ble.send("[Car,status]"), timeout=10))
    except Exception as exc:
        ble.set_error(exc)
        return jsonify({"ok": False, "error": str(exc)})


@app.route("/ble/power/off", methods=["POST"])
def ble_power_off():
    try:
        return jsonify(ble.run(ble.send("[Car,down]"), timeout=10))
    except Exception as exc:
        ble.set_error(exc)
        return jsonify({"ok": False, "error": str(exc)})


@app.route("/ble/stop", methods=["POST"])
def ble_stop():
    try:
        return jsonify(ble.run(ble.send("[Drive,stop]"), timeout=10))
    except Exception as exc:
        ble.set_error(exc)
        return jsonify({"ok": False, "error": str(exc)})


@app.route("/ble/drive", methods=["POST"])
def ble_drive():
    try:
        data = request_payload()
        throttle = clamp(float(data.get("throttle", 0.0)), -BLE_MAX_THROTTLE, BLE_MAX_THROTTLE)
        turn = clamp(float(data.get("turn", 0.0)), -BLE_MAX_TURN, BLE_MAX_TURN)
        duration_ms = int(float(data.get("duration_ms", BLE_MANUAL_DURATION_MS)))
        duration_ms = int(clamp(duration_ms, 120, 800))

        command = f"[Drive,manual,{throttle:.3f},{turn:.3f},{duration_ms}]"
        return jsonify(ble.run(ble.send(command, wait_response=False), timeout=5))
    except Exception as exc:
        ble.set_error(exc)
        return jsonify({"ok": False, "error": str(exc)})


if __name__ == "__main__":
    local_ip = get_local_ip()
    print("=======================================")
    print("Balance Car WiFi Proxy + BLE Control")
    print("Open on this computer:")
    print("http://127.0.0.1:5000")
    print()
    print("Open from same LAN:")
    print(f"http://{local_ip}:5000")
    print()
    print("BLE target:")
    print(f"  name: {BLE_DEVICE_NAME}")
    print(f"  service: {BLE_SERVICE_UUID}")
    print(f"  char: {BLE_CHAR_UUID}")
    print("=======================================")
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
