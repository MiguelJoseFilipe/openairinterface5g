# SPDX-License-Identifier: LicenseRef-CSSL-1.0

"""Experiment clock server for the NTN Doppler emulation.

Serves the experiment time (seconds within the satellite pass, 0 at maximum
elevation) to the gNB and to any other client over TCP. The clock advances
once the client "oai_gnb" has connected, over the pass window read from
config.txt, and the server exits at the end of the window.

usage: python3 TCP_server.py [-v] [--config config.txt]
"""
import os
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
# Same config.txt as the gNB
filename = sys.argv[sys.argv.index("--config") + 1] if "--config" in sys.argv else "config.txt"

# Shared state
shutdown_event = threading.Event()
experiment_time = 0.0
experiment_time_lock = threading.Lock()
running = False
start_offset = 0.0
start_time = None
oai_gnb_connected = False  # Flag to track if the "oai_gnb" client is connected
client_connections = {}  # To track all connected clients
clients_list = []  # List of all connected client identifiers
client_connections_lock = threading.Lock()
client_addresses = {}
client_last_sent_time = {}  # client_id -> last time value sent

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


def get_max_elevation(filename):
    try:
        with open(filename, 'r') as file:
            for line in file:
                if "Maximum elevation =" in line:
                    return int(line.split("=")[1].strip())
    except Exception as e:
        vprint(f"Error reading config: {e}")
    return None

def t_v(x):
    p = np.cos(np.radians(np.degrees(np.arccos(R_e / R * np.cos(np.radians(Theta_min)))) - Theta_min))
    return (2 / Omega_s) * np.arccos(
        p / np.cos(np.radians(np.degrees(np.arccos(R_e / R * np.cos(np.radians(x)))) - x))
    )

def read_key(filename, key):
    """Value after "<key> =" in the config (comment lines ignored), or None."""
    try:
        with open(filename, 'r') as file:
            for line in file:
                if not line.lstrip().startswith('#') and key + " =" in line:
                    return line.split("=", 1)[1].split("#")[0].strip()
    except Exception as e:
        vprint(f"Error reading config: {e}")
    return None


def pass_window(filename):
    """(start, duration) of the experiment clock for the configured model.

    Earth-moving: the analytic pass, symmetric about maximum elevation.
    Quasi-earth-fixed: exactly the rows the gNB plays from the table (full
    pass, or the elevation-masked window), which need not be symmetric.
    """
    if read_key(filename, "Doppler model") == "1":
        table = read_key(filename, "QEF table") or "qef_table.csv"
        if not os.path.isabs(table):
            table = os.path.join(os.path.dirname(os.path.abspath(filename)), table)
        masked_only = read_key(filename, "QEF window") != "0"
        t = []
        with open(table) as f:
            for line in f:
                if line.startswith('#') or not line.strip():
                    continue
                cols = line.split(',')
                if not masked_only or int(cols[10]) == 1:
                    t.append(float(cols[0]))
        print(f"[Clock] quasi-earth-fixed: {'masked' if masked_only else 'full'} window "
              f"{t[0]:.3f} .. {t[-1]:.3f} s from {table}")
        return t[0], t[-1] - t[0]
    max_elev = get_max_elevation(filename)
    if max_elev is None:
        return None
    vis_time = t_v(max_elev)
    print(f"[Clock] earth-moving: +-{vis_time / 2:.3f} s")
    return -vis_time / 2, vis_time


def time_updater(initial_offset, tv):
    global experiment_time, start_offset, running, start_time, oai_gnb_connected
    start_offset = initial_offset

    start_time = time.time()
    while not shutdown_event.is_set():
        if oai_gnb_connected and running:
            elapsed = time.time() - start_time
            if elapsed >= tv:
                with experiment_time_lock:
                    experiment_time = start_offset + tv
                running = False
                print(f"\n[Clock] Visibility time {tv:.3f}s completed. Shutting down.")
                shutdown_event.set()
                break

            with experiment_time_lock:
                experiment_time = start_offset + elapsed
        time.sleep(0.01)



def connection_listener():
    global oai_gnb_connected, running, client_connections, clients_list
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
    global oai_gnb_connected, running, client_connections, clients_list, client_addresses

    with conn:
        try:
            addr = conn.getpeername()

            # Ask for identifier
            conn.sendall(b"Please send your client identifier (e.g., 'oai_gnb'):")
            client_id = conn.recv(1024).decode().strip()
            vprint(f"Received client identifier: {client_id} from {addr}")

            # Track client
            client_connections[client_id] = conn
            client_addresses[client_id] = addr  # store address in separate dict
            if client_id not in clients_list:
                clients_list.append(client_id)

            if client_id.lower() == "oai_gnb":
                oai_gnb_connected = True
                running = True
                conn.sendall(b"oai_gnb client connected. Time countdown starting.")
            else:
                conn.sendall(f"Client '{client_id}' connected. You may query the time.".encode())

            while True:
                data = conn.recv(1024)
                if not data:
                    break

                request = data.decode().strip().lower()
                if request == "time?":
                    with experiment_time_lock:
                        current_time = experiment_time
                    vprint(f"Sending time: {current_time:.3f} s")
                    # big-endian double
                    conn.sendall(struct.pack('!d', current_time))
                    client_last_sent_time[client_id] = f"{current_time:.3f}"  # Track last time sent
                else:
                    conn.sendall(b"Unknown request. Send 'time?' to get current experiment time.")

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


def run_server():
    window = pass_window(filename)
    if window is None:
        vprint("Failed to load the pass window from config.")
        return
    initial_offset, vis_time = window

    threads = [
        threading.Thread(target=time_updater, args=(initial_offset, vis_time), daemon=True),
        threading.Thread(target=connection_listener, daemon=True),
        threading.Thread(target=command_listener, daemon=True),
        threading.Thread(target=display_status, daemon=True),
    ]

    for t in threads:
        t.start()

    try:
        while not shutdown_event.is_set():
            time.sleep(0.01)
        vprint("\n[Main] Shutdown event received. Exiting.")
        sys.exit(0)
    except KeyboardInterrupt:
        vprint("\nMain thread interrupted. Exiting.")
        shutdown_event.set()


if __name__ == '__main__':
    run_server()
