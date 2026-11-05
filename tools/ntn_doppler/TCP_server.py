# SPDX-License-Identifier: LicenseRef-CSSL-1.0

"""Experiment clock server.

Serves the pass time (seconds, 0 at maximum elevation) to the transmitter (eNB
"sonica" or gNB "oai_gnb") and to the receivers over TCP port 65432. The clock
starts when the transmitter reports its first transmitted subframe or slot
("started") and loops over the pass read from the configuration file.

usage: python3 TCP_server.py [-v] [--config config.txt]
"""
import socket
import threading
import time
import numpy as np
from math import *
from datetime import datetime
import sys
from rich.console import Console
from rich.table import Table
from rich.live import Live
import struct
VERBOSE = "-v" in sys.argv
def vprint(*args, **kwargs):
    if VERBOSE:
        print(*args, **kwargs)

# Constants
HOST = '0.0.0.0'
PORT = 65432
H = 600000
Theta_min = 30
K = 3.986005e14
R_e = 6371000
R = R_e + H
c = 299792458
Omega_s = sqrt(K / (H + R_e) ** 3)
T_s = 2 * pi / Omega_s
Alpha_max = degrees(acos((R_e / R) * cos(radians(Theta_min)))) - Theta_min
R_c = R_e * sin(radians(Alpha_max))
T_max = 2 * radians(Alpha_max) / Omega_s
filename = sys.argv[sys.argv.index("--config") + 1] if "--config" in sys.argv else "config.txt"

# Shared state
shutdown_event = threading.Event()
experiment_time = 0.0
experiment_time_lock = threading.Lock()
running = False
start_offset = 0.0
tv_global = 0.0            # full visibility duration = one pass (loop period)
loop_id = 0               # increments each time the pass clock loops / re-anchors
start_time = None  # Make sure start_time is properly initialized
sonica_connected = False  # Flag to track if the "sonica" client is connected
client_connections = {}  # To track all connected clients
clients_list = []  # List of all connected client identifiers
client_connections_lock = threading.Lock()
client_addresses = {}
client_last_sent_time = {}  # client_id -> last time value sent

# ---- NTP discipline: keep the pass clock's wall-time base exact ----
# The whole distributed clock (eNB + all UEs) syncs to THIS server's clock, so
# disciplining the server's time base to NTP disciplines the entire experiment.
# now_ntp() = system time + ntp_offset, where ntp_offset is refreshed
# periodically from an NTP server (graceful no-op offset=0 if unreachable, e.g.
# offline -- set NTP_HOST to your LAN NTP server in that case).
NTP_HOST = "pool.ntp.org"
NTP_SYNC_INTERVAL = 64      # seconds between NTP queries (NTP disciplines slowly)
NTP_DELTA = 2208988800      # seconds between the NTP epoch (1900) and Unix epoch (1970)
ntp_offset = 0.0

def query_ntp_offset(host=NTP_HOST, timeout=3.0):
    """One SNTP round-trip -> clock offset (seconds) vs the NTP server, or None
    on failure. No external dependency (raw NTPv3 packet)."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(timeout)
        pkt = b'\x1b' + 47 * b'\0'   # LI=0, VN=3, Mode=3 (client)
        t1 = time.time()
        s.sendto(pkt, (host, 123))
        data, _ = s.recvfrom(48)
        t4 = time.time()
        s.close()
        secs, frac = struct.unpack("!II", data[40:48])   # transmit timestamp
        ntp_time = (secs - NTP_DELTA) + frac / 2**32
        return ntp_time - (t1 + t4) / 2.0                # server time minus local midpoint
    except Exception as e:
        vprint(f"[NTP] query failed: {e}")
        return None

def now_ntp():
    return time.time() + ntp_offset

def ntp_syncer():
    """Periodically refresh ntp_offset. When it steps, shift start_time by the
    same delta so the pass-time (start_offset + now_ntp() - start_time) stays
    continuous -- the discipline never jumps the running pass."""
    global ntp_offset, start_time
    while not shutdown_event.is_set():
        off = query_ntp_offset()
        if off is not None:
            with experiment_time_lock:
                delta = off - ntp_offset
                ntp_offset = off
                if start_time is not None:
                    start_time += delta
            vprint(f"[NTP] offset = {off*1000:+.2f} ms (step {delta*1000:+.2f} ms)")
        for _ in range(NTP_SYNC_INTERVAL * 10):
            if shutdown_event.is_set():
                return
            time.sleep(0.1)

def display_status():
    console = Console()
    with Live(refresh_per_second=2, console=console) as live:
        while True:
            table = Table(title="Clock Server Status", expand=True)

            table.add_column("Client", style="bold cyan")
            table.add_column("Address", style="magenta")
            table.add_column("Running", justify="center")
            table.add_column("Last Time Sent", justify="right", style="green")

            # Server clock row
            with experiment_time_lock:
                time_display = f"{experiment_time:.3f} s"
            running_status = "Yes" if running else "No"
            table.add_row("[bold yellow]Server Clock[/bold yellow]", "-", running_status, time_display)

            # Connected clients
            for client_id in clients_list:
                addr = client_addresses.get(client_id, ("?", "?"))
                addr_str = f"{addr[0]}:{addr[1]}"
                last_sent = client_last_sent_time.get(client_id, "-")
                table.add_row(client_id, addr_str, "Yes", last_sent)

            live.update(table)
            #time.sleep(0.5)


def get_max_elevation(filename):
    try:
        with open(filename, 'r') as file:
            for line in file:
                if "Maximum elevation =" in line:
                    print(f"Found line in config: {line.strip()}")
                    return int(line.split("=")[1].strip())
    except Exception as e:
        vprint(f"Error reading config: {e}")
    return None

def t_v(x):
    p = np.cos(np.radians(np.degrees(np.arccos(R_e / R * np.cos(np.radians(Theta_min)))) - Theta_min))
    return (2 / Omega_s) * np.arccos(
        p / np.cos(np.radians(np.degrees(np.arccos(R_e / R * np.cos(np.radians(x)))) - x))
    )

def time_updater(initial_offset):
    global experiment_time, start_offset, running, start_time, tv_global, loop_id
    start_offset = initial_offset
    tv_global = abs(initial_offset) * 2  # Full visibility duration = one pass

    # NOTE: the pass clock is NOT started here at server launch (that was the
    # bug -- it made the pass already-running before the eNB could transmit its
    # beginning). It is anchored to the eNB's FIRST transmitted subframe via the
    # "started" message (see handle_client). `start_time` stays None until then.
    while not shutdown_event.is_set():
        if running and start_time is not None:
            elapsed = now_ntp() - start_time
            if elapsed >= tv_global:
                # LOOP the pass instead of shutting down: reset to the beginning
                # so a continuous eNB run replays passes with no full restart.
                with experiment_time_lock:
                    start_time = now_ntp()
                    experiment_time = start_offset
                loop_id += 1
                vprint(f"[Clock] pass complete -> looping (loop {loop_id})")
            else:
                with experiment_time_lock:
                    experiment_time = start_offset + elapsed
        time.sleep(0.005)



def connection_listener():
    global sonica_connected, running, client_connections, clients_list
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(1.0)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        vprint(f"\nClock server running on {HOST}:{PORT}")
        while not shutdown_event.is_set():
            try:
                conn, addr = s.accept()
                vprint(f"\nConnected by {addr}")
                threading.Thread(target=handle_client, args=(conn,), daemon=True).start()
            except socket.timeout:
                continue
            except KeyboardInterrupt:
                vprint("\nServer interrupted by user.")
                shutdown_event.set()
                break
            except Exception as e:
                vprint(f"[connection_listener] Error: {e}")
                break


def handle_client(conn):
    global sonica_connected, running, client_connections, clients_list, client_addresses
    global experiment_time, start_time, loop_id

    with conn:
        try:
            addr = conn.getpeername()

            # Ask for identifier
            conn.sendall(b"Please send your client identifier (e.g., 'sonica'):")
            client_id = conn.recv(1024).decode().strip()
            vprint(f"Received client identifier: {client_id} from {addr}")

            # Track client
            client_connections[client_id] = conn
            client_addresses[client_id] = addr  # store address in separate dict
            if client_id not in clients_list:
                clients_list.append(client_id)

            if client_id.lower() == "sonica":
                sonica_connected = True
                # Do NOT start the clock on connect -- wait for the eNB's
                # "started" message at its first transmitted subframe so the
                # pass is anchored to the true beginning of visibility.
                conn.sendall(b"Sonica connected. Send 'started' at first TX to anchor the pass clock.")
            else:
                conn.sendall(f"Client '{client_id}' connected. You may query the time.".encode())

            while True:
                data = conn.recv(1024)
                if not data:
                    break

                request = data.decode().strip().lower()
                if request == "time?":
                    with experiment_time_lock:
                        # Before the pass is anchored (the eNB hasn't sent
                        # "started" yet -- e.g. it crashed at boot) serve
                        # start_offset, the BEGINNING of the pass. A dumb UE then
                        # runs one bounded pass and ENDS normally (saving 0 NPSS,
                        # which the launcher uses to detect the dead eNB and restart
                        # it) instead of latching a NaN and hanging forever with
                        # a stop condition that can never fire.
                        if not running or start_time is None:
                            current_time = start_offset
                        else:
                            current_time = experiment_time

                    vprint(f"Sending time: {current_time:.3f} s")

                    float_bytes = struct.pack('!d', current_time)  # double-precision, network order
                    conn.sendall(float_bytes)

                    client_last_sent_time[client_id] = f"{current_time:.3f}"  # Track last time sent
                elif request == "started":
                    # The eNB reached its first transmitted subframe. Anchor the
                    # pass clock to THIS instant ONLY if it is not already running
                    # (fresh server, or after an explicit 'reset'). The eNB crashes
                    # often and is auto-restarted MID-PASS; a RECONNECTING eNB must
                    # NOT reset the clock, or every crash would throw away all the
                    # late-pass measurements. Instead the pass clock free-runs on
                    # wall time and the restarted eNB rejoins IN PHASE by querying
                    # the current pass-time (it re-syncs every 1 s). A genuinely
                    # NEW pass is started by recreating the server or via 'reset'
                    # (both leave start_time = None).
                    with experiment_time_lock:
                        first_anchor = (start_time is None)
                        if first_anchor:
                            start_time = now_ntp()
                            experiment_time = start_offset
                            loop_id += 1
                        running = True
                        cur = experiment_time
                    if first_anchor:
                        vprint(f"[Clock] anchored by eNB 'started' (loop {loop_id}) at "
                               f"experiment_time={start_offset:.3f}s")
                    else:
                        vprint(f"[Clock] eNB reconnected -> clock kept running at "
                               f"experiment_time={cur:.3f}s (loop {loop_id}); no reset")
                    conn.sendall(b"anchored")
                else:
                    conn.sendall(b"Unknown request. Send 'time?' or 'started'.")

        except (ConnectionResetError, ConnectionAbortedError):
            vprint(f"Connection lost from {addr}")
        except Exception as e:
            vprint(f"Client handler error: {e}")
        finally:
            # Cleanup
            for key, value in list(client_connections.items()):
                if value == conn:
                    del client_connections[key]
                    client_addresses.pop(key, None)
                    if key in clients_list:
                        clients_list.remove(key)
                    break




def command_listener():
    global running, start_offset, experiment_time, start_time
    vprint("\n[Command Interface Ready] Type 'start', 'stop', or 'reset'")
    while not shutdown_event.is_set():
        try:
            cmd = input().strip().lower()
            if cmd == "stop":
                running = False
                vprint("Clock paused.")
            elif cmd == "start":
                running = True
                start_time = time.time() - experiment_time  # Sync start_time with experiment_time
                vprint("Clock resumed.")
            elif cmd == "reset":
                with experiment_time_lock:
                    experiment_time = start_offset
                start_time = None
                vprint(f"Clock reset to initial time: {start_offset:.3f} s")
            elif cmd == "exit":
                vprint("Exiting via user command.")
                shutdown_event.set()
                break
            else:
                vprint("Unknown command. Use: start | stop | reset | exit")
        except KeyboardInterrupt:
            vprint("\nCommand listener interrupted.")
            shutdown_event.set()
            break


def get_calibration_mode(filename):
    try:
        with open(filename, 'r') as f:
            for line in f:
                if line.strip().startswith("Calibration Mode ="):
                    return int(line.split("=", 1)[1].strip()) != 0
    except Exception:
        pass
    return False


def run_server():
    max_elev = get_max_elevation(filename)
    if max_elev is None:
        vprint("Failed to load Maximum elevation from config.")
        return

    # Calibration phase is transparent to the clock (the eNB just transmits 0
    # Doppler); we log it so the operator can see which mode this run is in.
    if get_calibration_mode(filename):
        print("[Clock] CALIBRATION MODE (eNB transmits 0 Doppler; clock still anchors for UE timestamps)")

    vis_time = t_v(max_elev)
    initial_offset = -vis_time / 2

    threads = [
        threading.Thread(target=time_updater, args=(initial_offset,), daemon=True),
        threading.Thread(target=connection_listener, daemon=True),
        threading.Thread(target=command_listener, daemon=True),
        threading.Thread(target=display_status, daemon=True),
        threading.Thread(target=ntp_syncer, daemon=True),
    ]

    for t in threads:
        t.start()

    try:
        while not shutdown_event.is_set():
            time.sleep(0.01)
        vprint("\n[Main] Shutdown event received. Exiting.")
        print("Done but not successful")
        sys.exit(0)  # ✅ Fully exit the program
    except KeyboardInterrupt:
        vprint("\nMain thread interrupted. Exiting.")
        shutdown_event.set()


if __name__ == '__main__':
    run_server()
