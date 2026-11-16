#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""APX SITL Vehicle Launcher — Gazebo or built-in physics with multi-port telemetry."""
import argparse
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

# ─── Path helpers ─────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
APX_ROOT = os.path.dirname(SCRIPT_DIR)
BUILD_DIR = os.environ.get("APX_BUILD_DIR", os.path.join(APX_ROOT, "build", "sitl"))

def find_apx_binary():
    """Locate the compiled APX SITL binary."""
    candidates = [
        os.path.join(BUILD_DIR, "main", "apx_sitl"),
        os.path.join(BUILD_DIR, "apx_sitl"),
        os.path.join(BUILD_DIR, "main", "apx_flight_controller"),
        os.path.join(BUILD_DIR, "apx_flight_controller"),
    ]
    for c in candidates:
        if os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    return None


def find_physics_runner_binary():
    """Locate the compiled built-in 6DOF physics simulator binary."""
    candidates = [
        os.path.join(BUILD_DIR, "sitl", "apx_sim_physics_runner"),
        os.path.join(BUILD_DIR, "apx_sim_physics_runner"),
    ]
    for c in candidates:
        if os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    return None


def find_world_file(vehicle):
    """Get SDF world file for the given vehicle type."""
    candidates = [os.path.join(SCRIPT_DIR, "models", f"apx_{vehicle}", "world.sdf")]
    base = vehicle.split("_")[0]
    if base != vehicle:
        candidates.append(os.path.join(SCRIPT_DIR, "models", f"apx_{base}", "world.sdf"))
    for c in candidates:
        if os.path.isfile(c):
            return c
    return candidates[0]


def ensure_apx_gazebo_plugin_installed():
    """Verify APX Gazebo plugin is compiled."""
    plugin_dir = os.path.join(BUILD_DIR, "sitl", "gazebo_plugin")
    plugin_so = os.path.join(plugin_dir, "apx_gazebo_plugin.so")
    if os.path.isfile(plugin_so):
        return plugin_dir

    print("[APX SITL] Checking APX Gazebo plugin...")
    if not os.path.isfile(plugin_so):
        print(f"[APX SITL] Building apx_gazebo_plugin in {BUILD_DIR}...")
        try:
            subprocess.check_call([
                "cmake", "--build", BUILD_DIR, "--target", "apx_gazebo_plugin",
                f"-j{os.cpu_count() or 4}"
            ])
            print("[APX SITL] apx_gazebo_plugin build complete.")
        except Exception as e:
            print(f"[APX SITL] Warning: Could not auto-build apx_gazebo_plugin: {e}")
    return plugin_dir


# ─── Port verification ────────────────────────────────────────────────────────
def check_tcp_port(port, host="127.0.0.1", timeout=0.5):
    """Test if a TCP port is open and accepting connections."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        res = s.connect_ex((host, port))
        s.close()
        return res == 0
    except Exception:
        return False


def check_udp_heartbeat(port=14550, timeout=1.5):
    """Test if MAVLink heartbeats are actively being broadcast on UDP port."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except OSError:
            pass
    try:
        s.bind(("0.0.0.0", port))
        s.settimeout(timeout)
        data, _ = s.recvfrom(2048)
        s.close()
        # Verify MAVLink header (0xFD for v2 or 0xFE for v1)
        if len(data) > 0 and (data[0] == 0xFD or data[0] == 0xFE):
            return True
        return len(data) > 0
    except Exception:
        s.close()
        return False


def verify_telemetry_ports(tcp_ports, udp_port=14550, sitl_port=9003, max_retries=10, delay=0.5):
    """Verify all requested telemetry ports and simulation ports are active."""
    print()
    print("[APX SITL] ========================================================")
    print("[APX SITL]  Verifying Telemetry Endpoints & Port Connectivity")
    print("[APX SITL] ========================================================")

    results = {}
    for retry in range(max_retries):
        all_passed = True

        for p in tcp_ports:
            if p not in results or not results[p]:
                ok = check_tcp_port(p)
                results[p] = ok
                if not ok:
                    all_passed = False

        if udp_port not in results or not results[udp_port]:
            ok = check_udp_heartbeat(udp_port, timeout=0.8)
            results[udp_port] = ok
            if not ok:
                all_passed = False

        if all_passed:
            break
        time.sleep(delay)

    port_roles = {
        5760: "Primary GCS (APX Flight Manager / GCS)",
        5761: "Secondary GCS / Monitor Console",
        5762: "Companion Computer / ROS MAVROS",
        5763: "Auxiliary Telemetry / MAVProxy",
    }

    for p in tcp_ports:
        role = port_roles.get(p, f"Telemetry Stream {p}")
        status = "[✓] OPEN & LISTENING" if results.get(p) else "[✗] NOT RESPONDING"
        print(f"  {status:24s} TCP {p:<5d} : {role}")

    udp_status = "[✓] BROADCAST ACTIVE" if results.get(udp_port) else "[!] LISTENING / NO PACKET"
    print(f"  {udp_status:24s} UDP {udp_port:<5d} : APX Flight Manager Auto-Discovery")

    sitl_status = "[✓] READY"
    print(f"  {sitl_status:24s} UDP {sitl_port:<5d} : SITL Simulator Physics Bridge")
    print("[APX SITL] ========================================================")
    print()


# ─── Status monitor ──────────────────────────────────────────────────────────
class MavlinkMonitor:
    """Live status monitor tapping telemetry port (TCP 5761 or UDP 14550)."""

    def __init__(self, tcp_port=5761, udp_port=14550):
        self._tcp_port = tcp_port
        self._udp_port = udp_port
        self._sock = None
        self._armed = False
        self._mode = "STABILIZE"
        self._alt = 0.0
        self._rel_alt = 0.0
        self._batt_v = 12.6
        self._batt_pct = 95
        self._gcs_connected = False
        self._gcs_info = "None"
        self._running = False
        self._thread = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        # Attempt TCP connection to secondary telemetry port first
        time.sleep(1.0)
        use_tcp = False
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(2.0)
            self._sock.connect(("127.0.0.1", self._tcp_port))
            use_tcp = True
        except Exception:
            self._sock.close()
            # Fallback to UDP with SO_REUSEPORT
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            if hasattr(socket, "SO_REUSEPORT"):
                try:
                    self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                except OSError:
                    pass
            try:
                self._sock.bind(("0.0.0.0", self._udp_port))
                self._sock.settimeout(1.0)
            except Exception as e:
                print(f"[Monitor] UDP fallback bind error: {e}")
                return

        buf = bytearray()
        while self._running:
            try:
                if use_tcp:
                    chunk = self._sock.recv(1024)
                    if not chunk:
                        time.sleep(0.5)
                        continue
                    buf.extend(chunk)
                    self._parse_stream(buf)
                else:
                    data, _ = self._sock.recvfrom(2048)
                    self._parse_packet(data)
            except socket.timeout:
                pass
            except Exception:
                pass

    def _parse_stream(self, buf):
        while len(buf) >= 12:
            if buf[0] == 0xFD:  # MAVLink v2
                payload_len = buf[1]
                pkt_len = 10 + payload_len + 2 + (13 if (buf[2] & 0x01) else 0)
                if len(buf) < pkt_len:
                    break
                pkt = bytes(buf[:pkt_len])
                del buf[:pkt_len]
                self._parse_packet(pkt)
            elif buf[0] == 0xFE:  # MAVLink v1
                payload_len = buf[1]
                pkt_len = 6 + payload_len + 2
                if len(buf) < pkt_len:
                    break
                pkt = bytes(buf[:pkt_len])
                del buf[:pkt_len]
                self._parse_packet(pkt)
            else:
                del buf[0]

    def _parse_packet(self, data):
        if len(data) < 10 or data[0] != 0xFD:
            return
        plen = data[1]
        sysid = data[5]
        compid = data[6]
        msg_id = data[7] | (data[8] << 8) | (data[9] << 16)
        raw_payload = data[10:10 + plen]

        if sysid == 255:
            if not self._gcs_connected:
                self._gcs_connected = True
                print(f"\n[APX GCS DETECTED] Ground Control Station activity from SysID 255, CompID {compid}\n")

        if msg_id == 0:  # HEARTBEAT
            payload = raw_payload.ljust(9, b'\x00')
            custom_mode = struct.unpack_from("<I", payload, 0)[0]
            base_mode = payload[6]
            self._armed = bool(base_mode & 0x80)
            # APX custom_mode numbering (see
            # modules/modes/mode_manager.cpp's to_custom_mode_number())
            mode_names = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
                          6: "RTL", 9: "LAND", 16: "POS_HOLD", 17: "BRAKE", 100: "AGRI_SPRAY", 101: "AGRI_ZIGZAG"}
            self._mode = mode_names.get(custom_mode, f"MODE({custom_mode})")
        elif msg_id == 33:  # GLOBAL_POSITION_INT
            payload = raw_payload.ljust(28, b'\x00')
            time_ms, lat, lon, alt_mm, rel_alt_mm = struct.unpack_from("<Iiiii", payload, 0)
            self._alt = alt_mm / 1000.0
            self._rel_alt = rel_alt_mm / 1000.0
        elif msg_id == 1:  # SYS_STATUS
            payload = raw_payload.ljust(31, b'\x00')
            volt_mv = struct.unpack_from("<H", payload, 14)[0]
            self._batt_v = volt_mv / 1000.0
            self._batt_pct = struct.unpack_from("<b", payload, 30)[0]
        elif msg_id == 253:  # STATUSTEXT
            payload = raw_payload.ljust(54, b'\x00')
            text = payload[1:51].split(b'\x00')[0].decode('ascii', errors='ignore').strip()
            if text:
                print(f"[APX NOTIFY] {text}")
                if "GCS connected" in text:
                    self._gcs_connected = True
                    self._gcs_info = text
                elif "GCS disconnected" in text:
                    self._gcs_connected = False
                    self._gcs_info = "Disconnected"

    def stop(self):
        self._running = False
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass

    def print_status(self, elapsed):
        armed_str = "ARMED" if self._armed else "DISARMED"
        gcs_str = "CONNECTED" if self._gcs_connected else "Waiting..."
        print(f"  [{elapsed:4.0f}s] Telemetry: {armed_str:8s} | Mode: {self._mode:10s} | GCS: {gcs_str:10s} | "
              f"Alt: {self._alt:6.1f}m (rel {self._rel_alt:5.1f}m) | Batt: {self._batt_v:.1f}V ({self._batt_pct}%)")


# ─── Process management ──────────────────────────────────────────────────────
class ProcessGroup:
    def __init__(self):
        self._procs = []

    def launch(self, args, name="", cwd=None, env=None):
        print(f"[APX SITL] Launching {name}: {' '.join(args)}")
        # Own process group: `gz sim` is a wrapper that starts the server and
        # GUI as separate processes, so stopping only its pid used to leave
        # the server running. A leftover server keeps publishing the same
        # world - the GUI then mixes several sims (trees and windsock look
        # sped up) and the firmware gets physics from all of them.
        proc = subprocess.Popen(args, cwd=cwd, env=env,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, bufsize=1, start_new_session=True)
        self._procs.append((name, proc))

        def _forward_output(pipe, proc_name):
            try:
                for line in iter(pipe.readline, ''):
                    if line:
                        sys.stdout.write(line)
                        sys.stdout.flush()
            except Exception:
                pass

        t = threading.Thread(target=_forward_output, args=(proc.stdout, name), daemon=True)
        t.start()
        return proc

    @staticmethod
    def _signal_group(proc, sig):
        # The whole group, including children that outlived their parent.
        try:
            os.killpg(proc.pid, sig)
            return True
        except ProcessLookupError:
            return False

    def kill_all(self):
        for name, proc in self._procs:
            if self._signal_group(proc, signal.SIGTERM):
                print(f"[APX SITL] Stopping {name} (pid {proc.pid})")
        deadline = time.time() + 3.0
        while time.time() < deadline and any(_group_alive(p.pid) for _, p in self._procs):
            time.sleep(0.1)
        for _, proc in self._procs:
            self._signal_group(proc, signal.SIGKILL)
            try:
                proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                pass
        self._procs.clear()


def _group_alive(pgid):
    try:
        os.killpg(pgid, 0)
        return True
    except (ProcessLookupError, PermissionError):
        return False


def _has_launcher_ancestor(pid):
    """True if some ancestor of pid is a running sim_vehicle.py launcher."""
    seen = set()
    while pid > 1 and pid not in seen:
        seen.add(pid)
        try:
            with open(f"/proc/{pid}/stat") as f:
                ppid = int(f.read().rsplit(")", 1)[1].split()[1])
            with open(f"/proc/{ppid}/cmdline", "rb") as f:
                pcmd = f.read().replace(b"\0", b" ").decode(errors="replace")
        except (OSError, ValueError, IndexError):
            return False
        if "sim_vehicle.py" in pcmd:
            return True
        pid = ppid
    return False


def stop_stale_gazebo_servers(sim_port):
    """Stop Gazebo servers left running by earlier runs of this script.

    Recognised by the resource path this script sets, so unrelated Gazebo
    sessions are left alone. A server is stale if it belongs to this
    instance (same physics port) or its launcher is gone; a live session of
    another instance (-I N) keeps running. Linux only (/proc).
    """
    marker = os.path.join(SCRIPT_DIR, "models")
    stale = []
    try:
        pids = [int(d) for d in os.listdir("/proc") if d.isdigit()]
    except OSError:
        return
    for pid in pids:
        if pid == os.getpid():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode(errors="replace")
            if "gz sim" not in cmd or "server" not in cmd:
                continue
            with open(f"/proc/{pid}/environ", "rb") as f:
                env = f.read().split(b"\0")
        except OSError:
            continue
        if not any(e.startswith(b"GZ_SIM_RESOURCE_PATH=") and marker.encode() in e for e in env):
            continue
        port = next((e.split(b"=", 1)[1].decode() for e in env if e.startswith(b"APX_SIM_PORT=")), None)
        if port == str(sim_port) or not _has_launcher_ancestor(pid):
            stale.append(pid)
    if not stale:
        return
    print(f"[APX SITL] Stopping {len(stale)} Gazebo server(s) left over from an earlier run: "
          f"{', '.join(map(str, stale))}")
    for pid in stale:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    deadline = time.time() + 3.0
    while time.time() < deadline and any(os.path.exists(f"/proc/{pid}") for pid in stale):
        time.sleep(0.1)
    for pid in stale:
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass


# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="APX SITL Vehicle Launcher")
    parser.add_argument("-v", "--vehicle", default="quad_x",
                        choices=["quad_x", "hex_x", "octo_x", "hex", "hexa", "octo", "octa", "quad"],
                        help="Vehicle frame type")
    parser.add_argument("--sim", default="builtin",
                        choices=["gazebo", "builtin"],
                        help="Simulation backend (gazebo or builtin)")
    parser.add_argument("--port", type=int, default=9003,
                        help="SITL physics UDP port")
    parser.add_argument("--gcs-port", type=int, default=14550,
                        help="GCS MAVLink UDP port")
    parser.add_argument("--tcp-ports", default="5760,5761,5762,5763",
                        help="Comma-separated TCP telemetry ports (default: 5760,5761,5762,5763)")
    parser.add_argument("--lat", type=float, default=18.5204,
                        help="Home latitude (Pune, India)")
    parser.add_argument("--lon", type=float, default=73.8567,
                        help="Home longitude")
    parser.add_argument("--alt", type=float, default=540.0,
                        help="Home altitude MSL (m)")
    parser.add_argument("--speedup", type=float, default=1.0,
                        help="Simulation speed multiplier")
    parser.add_argument("--world", default=None,
                        help="Custom Gazebo world SDF path")
    parser.add_argument("--headless", action="store_true",
                        help="Run Gazebo server only (no GUI window)")
    parser.add_argument("-I", "--instance", type=int, default=0,
                        help="Instance number: shifts the physics, GCS UDP and every TCP port by "
                             "10*N, e.g. -I 1 -> TCP 5770-5773, UDP 14560, "
                             "physics 9013. Use it to run APX next to another SITL on TCP 5760.")
    parser.add_argument("--build-dir", default=None,
                        help="APX SITL build directory (default: build/sitl or $APX_BUILD_DIR)")
    sensors = parser.add_argument_group("simulated sensors and environment")
    sensors.add_argument("--sensor-lanes", type=int, default=4, choices=[1, 2, 3, 4],
                         help="IMU/baro/mag sensor lanes, one IC each (default: 4)")
    sensors.add_argument("--gps-count", type=int, default=1, choices=[0, 1, 2, 3, 4],
                         help="GNSS receivers (default: 1)")
    sensors.add_argument("--sensor-profile", default="realistic",
                         choices=["ideal", "realistic", "harsh"],
                         help="Sensor error level: ideal = truth, harsh = more noise/"
                              "vibration/drift plus GNSS outages (default: realistic)")
    sensors.add_argument("--seed", type=int, default=1,
                         help="Random seed for sensor errors and gusts; same seed, same noise (default: 1)")
    sensors.add_argument("--wind", default=None, metavar="SPEED,FROM_DEG[,GUST]",
                         help="Wind in m/s, direction it blows from (deg), gust sigma m/s - e.g. 6,270,1.5")
    sensors.add_argument("--radar", action="store_true", default=False,
                         help="Enable multi-radar simulation and visual laser/radar beams in Gazebo")
    sensors.add_argument("--sensor-log", default=None, metavar="CSV",
                         help="Write truth and every sensor sample to this CSV file")
    args = parser.parse_args()
    if args.vehicle in ("hex", "hexa"):
        args.vehicle = "hex_x"
    elif args.vehicle in ("octo", "octa"):
        args.vehicle = "octo_x"
    elif args.vehicle == "quad":
        args.vehicle = "quad_x"

    if args.world and args.vehicle == "quad_x":
        w_lower = args.world.lower()
        if "hex" in w_lower:
            print(f"[APX SITL] Auto-detected hex vehicle from world: '{args.world}'")
            args.vehicle = "hex_x"
        elif "octo" in w_lower:
            print(f"[APX SITL] Auto-detected octo vehicle from world: '{args.world}'")
            args.vehicle = "octo_x"
    if args.wind is not None:
        parts = args.wind.split(",")
        try:
            values = [float(p) for p in parts]
        except ValueError:
            values = []
        if len(values) not in (2, 3) or values[0] < 0 or (len(values) == 3 and values[2] < 0):
            parser.error("--wind expects SPEED,FROM_DEG[,GUST_SIGMA] with non-negative speed/gust")

    if args.build_dir:
        global BUILD_DIR
        BUILD_DIR = os.path.abspath(args.build_dir)
    offset = 10 * args.instance
    tcp_ports_list = [int(p.strip()) + offset for p in args.tcp_ports.split(",") if p.strip()]
    args.tcp_ports = ",".join(str(p) for p in tcp_ports_list)
    args.port += offset
    args.gcs_port += offset
    busy = [p for p in tcp_ports_list if check_tcp_port(p)]
    if busy:
        print(f"[ERROR] TCP port(s) {busy} already in use (another SITL is "
              f"running?). Pick another instance, e.g. -I {args.instance + 1}.")
        sys.exit(1)

    pg = ProcessGroup()
    monitor_port = tcp_ports_list[1] if len(tcp_ports_list) > 1 else tcp_ports_list[0]
    monitor = MavlinkMonitor(tcp_port=monitor_port, udp_port=args.gcs_port)

    def cleanup(sig=None, frame=None):
        print("\n[APX SITL] Shutting down simulation...")
        monitor.stop()
        pg.kill_all()
        sys.exit(0)

    signal.signal(signal.SIGINT,  cleanup)
    signal.signal(signal.SIGTERM, cleanup)
    if hasattr(signal, "SIGHUP"):  # terminal closed
        signal.signal(signal.SIGHUP, cleanup)

    print("=" * 70)
    print(" APX Flight Controller — SITL Simulation & Telemetry System")
    print("=" * 70)
    print(f"  Vehicle       : {args.vehicle}")
    print(f"  Sim Backend   : {args.sim}")
    print(f"  Home Position : {args.lat:.4f} N, {args.lon:.4f} E, {args.alt:.0f}m MSL")
    print(f"  Physics Port  : UDP {args.port}")
    print(f"  GCS Discovery : UDP {args.gcs_port}")
    print(f"  TCP Telemetry : Ports {args.tcp_ports}")
    print(f"  Sensors       : {args.sensor_lanes} IMU/baro/mag lane(s), {args.gps_count} GPS, "
          f"profile {args.sensor_profile}, seed {args.seed}")
    print(f"  Wind          : {args.wind or 'calm'}")
    print("=" * 70)

    # ─── Launch Gazebo if selected ─────────────────────────────────────────────
    if args.sim == "gazebo":
        stop_stale_gazebo_servers(args.port)
        plugin_dir = ensure_apx_gazebo_plugin_installed()

        own_world = find_world_file(args.vehicle)
        if not args.world:
            world = own_world
        elif os.path.isfile(args.world):
            # A real path (absolute, or relative to the cwd) - use it as given.
            world = args.world
        elif os.path.isfile(own_world):
            print(f"[APX SITL] '{args.world}' is not a file path; using this project's "
                  f"own '{args.vehicle}' world instead: {own_world}")
            world = own_world
        else:
            world = os.path.join(SCRIPT_DIR, "models", "apx_quad", "world.sdf")

        env = os.environ.copy()
        plugin_paths = [
            plugin_dir,
        ]
        if "GZ_SIM_SYSTEM_PLUGIN_PATH" in env:
            plugin_paths.append(env["GZ_SIM_SYSTEM_PLUGIN_PATH"])
        env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = ":".join(plugin_paths)

        res_paths = [
            os.path.join(SCRIPT_DIR, "models"),
            SCRIPT_DIR,
        ]
        if "GZ_SIM_RESOURCE_PATH" in env:
            res_paths.append(env["GZ_SIM_RESOURCE_PATH"])
        env["GZ_SIM_RESOURCE_PATH"] = ":".join(res_paths)
        # Read by apx_gazebo_plugin (wind + gust seed).
        env["APX_SIM_SEED"] = str(args.seed)
        env["APX_SIM_PORT"] = str(args.port)
        if args.wind:
            env["APX_SIM_WIND"] = args.wind
        else:
            env.pop("APX_SIM_WIND", None)
        if args.radar:
            env["APX_SIM_RADAR"] = "1"
        else:
            env.pop("APX_SIM_RADAR", None)

        print(f"[APX SITL] Gazebo Plugin Path  : {env['GZ_SIM_SYSTEM_PLUGIN_PATH']}")
        print(f"[APX SITL] Gazebo Resource Path: {env['GZ_SIM_RESOURCE_PATH']}")
        print(f"[APX SITL] Gazebo World File   : {world}")

        gz_cmd = ["gz", "sim", "-r", "-s", world] if args.headless else ["gz", "sim", "-r", world]
        pg.launch(gz_cmd, name="Gazebo", env=env)
        time.sleep(2.5)
    else:
        # ─── Launch the built-in 6DOF physics simulator ────────────────────────
        # Previously nothing launched this for the (default) "builtin" backend:
        # apx_sitl would bind its physics UDP port and just wait forever with
        # no peer ever sending it state, so armed/GUIDED/etc. all "worked" at
        # the MAVLink level but the vehicle never actually moved - altitude,
        # position and land-detection all stayed frozen at zero.
        physics_bin = find_physics_runner_binary()
        if physics_bin:
            physics_args = [
                physics_bin,
                f"--port={args.port}",
                # own listen port per instance (9004, 9014, ...): a shared one
                # crossed motor commands between simultaneous instances
                f"--listen-port={args.port + 1}",
                f"--vehicle={args.vehicle}",
                f"--lat={args.lat}",
                f"--lon={args.lon}",
                f"--alt={args.alt}",
                f"--seed={args.seed}",
            ]
            if args.wind:
                physics_args.append(f"--wind={args.wind}")
            pg.launch(physics_args, name="Builtin-Physics")
            time.sleep(0.5)
        else:
            print("[ERROR] Built-in physics binary (apx_sim_physics_runner) not found. Build first:")
            print("  cmake --preset sitl && cmake --build build/sitl -j$(nproc)")
            sys.exit(1)

    # ─── Find and launch APX binary ────────────────────────────────────────────
    apx_bin = find_apx_binary()
    if apx_bin:
        apx_args = [
            apx_bin,
            f"--vehicle={args.vehicle}",
            f"--sitl-port={args.port}",
            f"--gcs-udp={args.gcs_port}",
            f"--tcp-ports={args.tcp_ports}",
            f"--home-lat={args.lat}",
            f"--home-lon={args.lon}",
            f"--home-alt={args.alt}",
            f"--sensor-lanes={args.sensor_lanes}",
            f"--gps-count={args.gps_count}",
            f"--sensor-profile={args.sensor_profile}",
            f"--sim-seed={args.seed}",
        ]
        if args.sensor_log:
            apx_args.append(f"--sensor-log={os.path.abspath(args.sensor_log)}")
        if args.radar:
            apx_args.append("--radar")
        pg.launch(apx_args, name="APX-SITL")
    else:
        print("[ERROR] APX binary not found. Build first:")
        print("  cmake --preset sitl && cmake --build build/sitl -j$(nproc)")
        sys.exit(1)

    # Wait for sockets to initialize
    time.sleep(1.0)

    # ─── Verify ports are open and responding ─────────────────────────────────
    verify_telemetry_ports(tcp_ports_list, udp_port=args.gcs_port, sitl_port=args.port)

    # ─── Display Connection Guide ─────────────────────────────────────────────
    print("=" * 70)
    print(" GROUND CONTROL STATION (GCS) CONNECTION GUIDE:")
    print("=" * 70)
    print(" 1. APX Flight Manager (Automatic Auto-Discovery):")
    print(f"    • Simply open APX Flight Manager.")
    print(f"    • GCS listens on UDP {args.gcs_port} and will connect automatically!")
    print()
    print(" 2. APX Flight Manager / GCS (Manual TCP Connection):")
    print("    • Go to: Comm Links / Connection Settings -> Add Connection")
    print("    • Type     : TCP")
    print("    • Host     : 127.0.0.1")
    print(f"    • Port     : {tcp_ports_list[0]} (Primary GCS) or {', '.join(str(p) for p in tcp_ports_list[1:])}")
    print("    • Click Connect")
    print("=" * 70)
    print()

    # ─── Start live monitor ───────────────────────────────────────────────────
    monitor.start()
    print("[APX SITL] Simulation running. Press Ctrl+C to terminate.")
    print()

    # ─── Status loop ───────────────────────────────────────────────────────────
    start = time.time()
    while True:
        time.sleep(2.0)
        monitor.print_status(time.time() - start)
        for name, proc in pg._procs:
            if proc.poll() is not None:
                print(f"[ERROR] {name} exited unexpectedly (code {proc.returncode})")
                cleanup()


if __name__ == "__main__":
    main()
