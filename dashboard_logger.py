import os
import sys
import time
import select
import tty
import termios
import csv
import subprocess

VERSION = "v3.4-DIAG"

try:
    from domusa_mqtt import REGISTER_NAMES as DESCRIPTIONS, R32_BITS
except ImportError:
    DESCRIPTIONS = {}
    R32_BITS = {}


PHASE_LABELS_FR = {
    0:  "Éteint",
    1:  "Nettoyage initial",
    3:  "Allumage (1)",
    4:  "Allumage (2)",
    5:  "Allumage (3)",
    9:  "Détection flamme",
    10: "Modulation Travail",
    11: "Chauffe normale",
    12: "Modulation",
    14: "Nettoyage en marche",
    16: "Standby",
    17: "Nettoyage final",
    18: "Post-ventilation",
    19: "Blocage sécurité",
}

def modbus_crc(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

MAX_EVENTS = 15
events_log = []

def add_event(msg):
    events_log.append(msg)
    if len(events_log) > MAX_EVENTS:
        events_log.pop(0)

def draw_screen(parsed, last_parsed, update_times, current_time, marker_count, life_char="", raw_bytes=0, last_frame_info=""):
    sys.stdout.write("\033[?25l\033[H")
    # En-tête avec Version et Stats Octets
    sys.stdout.write(f"\033[1;44;37m {VERSION} \033[0m \033[1;36mDOMUSA MODBUS DASHBOARD\033[0m | Octets: \033[1;33m{raw_bytes}\033[0m | {life_char}\n")
    sys.stdout.write(f"Dernière trame valide: \033[1;32m{last_frame_info}\033[0m\n\n")
    
    rows = 18
    cols = 4
    for r in range(rows):
        line = []
        for c in range(cols):
            reg = c * rows + r
            if reg < len(parsed):
                val = parsed[reg]
                time_since_update = current_time - update_times[reg]
                
                if time_since_update < 0.8 and update_times[reg] > 0:
                    color = "\033[1;32m"
                elif val != 0:
                    color = "\033[1;37m"
                else:
                    color = "\033[2;37m"

                desc = DESCRIPTIONS.get(reg, "")
                label = f"R{reg:02d}({desc})" if desc else f"R{reg:02d}"
                line.append(f"{label:<16}: {color}{val:7d}\033[0m")
        sys.stdout.write("  |  ".join(line) + "\n")

    # Footer R32 Bits
    if R32_BITS and len(parsed) > 32:
        r32_val = parsed[32]
        bits_str = []
        for b_idx, (b_key, b_name, b_icon, b_class) in R32_BITS.items():
            state = "ON" if (r32_val & (1 << b_idx)) else "OFF"
            color = "\033[1;32m" if state == "ON" else "\033[2;37m"
            bits_str.append(f"{b_name}: {color}{state}\033[0m")
        sys.stdout.write("\n\033[1;35m--- STATUT RELAIS (R32) ---\033[0m\n")
        mid = len(bits_str) // 2
        sys.stdout.write("  |  ".join(bits_str[:mid]) + "\n")
        sys.stdout.write("  |  ".join(bits_str[mid:]) + "\n")
        
    sys.stdout.write(f"\n\033[1;33m--- JOURNAL (Touche 'T' = Marqueur | 'Q' = Quitter) | Markers: #{marker_count} ---\033[0m\n")
    
    for _ in range(MAX_EVENTS):
        sys.stdout.write("\033[K\n")
    sys.stdout.write(f"\033[{MAX_EVENTS}A")
    
    for event in events_log:
        sys.stdout.write(f"{event}\033[K\n")
        
    for _ in range(MAX_EVENTS - len(events_log)):
        sys.stdout.write("\033[K\n")
        
    sys.stdout.flush()

def main():
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.write("=== DOMUSA LOGGER SETUP ===\n")
    sys.stdout.write("Voulez-vous activer l'enregistrement CSV ? (y/n) : ")
    sys.stdout.flush()
    import termios as t_module
    fd_in = sys.stdin.fileno()
    old_attr = t_module.tcgetattr(fd_in)
    try:
        tty.setcbreak(fd_in)
        choice = sys.stdin.read(1).lower()
    finally:
        t_module.tcsetattr(fd_in, t_module.TCSADRAIN, old_attr)
    
    enable_logging = (choice == 'y')
    log_filename = f"/home/scotch/domusa_log_{time.strftime('%Y%m%d_%H%M%S')}.csv" if enable_logging else None
    
    sys.stdout.write(f"\n[{VERSION}] Vérification du port série...\n")
    sys.stdout.flush()
    
    try:
        proc = subprocess.run(["fuser", "/dev/serial0"], capture_output=True, text=True)
        if proc.stdout.strip():
            sys.stdout.write(f"\n\033[1;41;37m !!! ATTENTION : Port occupé par les PIDs: {proc.stdout.strip()} !!! \033[0m\n")
            sys.stdout.write("Arrêtez le service domusa-mqtt avant de continuer.\n")
    except: pass

    try:
        subprocess.run(["stty", "-F", "/dev/serial0", "9600", "raw", "-echo", "min", "0", "time", "10"], 
                       capture_output=True, timeout=2)
    except Exception as e:
        sys.stdout.write(f"Note: stty ({e})\n")
    
    time.sleep(1)
    
    try:
        fd = os.open("/dev/serial0", os.O_RDWR | os.O_NONBLOCK)
    except Exception as e:
        sys.stdout.write(f"\033[2J\033[H\nErreur d'ouverture du port série: {e}\n")
        sys.stdout.flush()
        return

    old_term_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    buffer = bytearray()
    parsed = [0] * 70
    update_times = [0.0] * 70
    
    def create_read_packet(slave_id, start_addr, quantity):
        data = [slave_id, 0x03, (start_addr >> 8) & 0xFF, start_addr & 0xFF, (quantity >> 8) & 0xFF, quantity & 0xFF]
        crc = modbus_crc(bytes(data))
        data.append(crc & 0xFF)
        data.append((crc >> 8) & 0xFF)
        return bytes(data)
    
    ACTUATORS = [7, 9, 12, 14, 17, 26, 32, 35, 40, 53, 60]
    
    marker_counter = 0
    pending_marker = ""
    last_frame_time = time.time()
    last_frame_info = "Aucune"
    
    log_msg_file = os.path.basename(log_filename) if log_filename else "Désactivé"
    add_event(f"\033[2;37m[Système] Écoute démarrée. Fichier de log: {log_msg_file}\033[0m")

    if log_filename:
        csv_file = open(log_filename, 'w', newline='')
        log_writer = csv.writer(csv_file)
        header = ['Timestamp', 'Heure'] + [f'R{i:02d}' for i in range(70)] + ['Marqueur']
        log_writer.writerow(header)
    else:
        csv_file = None
        log_writer = None

    first_draw = True
    heartbeat_time = time.time()
    raw_stats = {"bytes": 0, "frames": 0, "errors": 0}
    
    try:
        while True:
            current_time = time.time()
            t_str = time.strftime('%H:%M:%S')
            updated_this_cycle = False
            
            life_char = "⚡" if int(current_time) % 2 == 0 else "  "
            if current_time - heartbeat_time > 2:
                heartbeat_time = current_time
                updated_this_cycle = True
            
            if first_draw:
                sys.stdout.write("\033[2J\033[H")
                draw_screen(parsed, parsed, update_times, current_time, marker_counter, life_char, raw_stats["bytes"], last_frame_info)
                first_draw = False
            
            if current_time - last_frame_time > 15:
                add_event(f"\033[1;41;37m[{t_str}] ⚠️ ATTENTE DONNÉES... ⚠️\033[0m")
                last_frame_time = current_time
                updated_this_cycle = True
                
            if select.select([sys.stdin], [], [], 0)[0]:
                c = sys.stdin.read(1)
                if c.lower() == 't':
                    marker_counter += 1
                    pending_marker = f"MARKER_{marker_counter}"
                    add_event(f"\033[1;44;37m[{t_str}] 🚩 MARQUEUR #{marker_counter} 🚩\033[0m")
                elif c.lower() == 'q':
                    break
                        
            # ── Polling actif (0x03) ──────────────────────────────────────────
            if 'last_query' not in locals(): last_query = 0.0
            query_interval = 2.0

            if current_time - last_query >= query_interval:
                success_flags = [False, False]
                packets = [
                    (0, 35, create_read_packet(3, 0, 35)),
                    (35, 35, create_read_packet(3, 35, 35))
                ]

                for b_idx, (start_addr, qtty, packet) in enumerate(packets):
                    try:
                        os.write(fd, packet)
                    except Exception as e:
                        add_event(f"\033[1;31m[{t_str}] Erreur écriture Bloc {b_idx+1}: {e}\033[0m")

                    time.sleep(0.4) # Temps de réponse

                    resp_buffer = bytearray()
                    try:
                        while True:
                            chunk = os.read(fd, 1024)
                            if not chunk: break
                            resp_buffer.extend(chunk)
                    except (BlockingIOError, OSError): pass

                    raw_stats["bytes"] += len(resp_buffer)

                    expected_len = 3 + (qtty * 2) + 2
                    expected_data_len = qtty * 2

                    if len(resp_buffer) >= expected_len:
                        for i in range(len(resp_buffer) - (expected_len - 1)):
                            if resp_buffer[i] == 3 and resp_buffer[i+1] == 0x03 and resp_buffer[i+2] == expected_data_len:
                                frame = resp_buffer[i : i + expected_len]
                                if modbus_crc(frame[:-2]) == ((frame[-1] << 8) | frame[-2]):
                                    last_frame_time = current_time
                                    raw_stats["frames"] += 1
                                    
                                    for j in range(qtty):
                                        idx = 3 + (j * 2)
                                        raw_val = (frame[idx] << 8) | frame[idx + 1]
                                        signed_val = raw_val if raw_val < 32768 else raw_val - 65536
                                        actual_idx = start_addr + j
                                        if signed_val != parsed[actual_idx]:
                                            parsed[actual_idx] = signed_val
                                            update_times[actual_idx] = current_time
                                    
                                    success_flags[b_idx] = True
                                    last_frame_info = f"ID=3 | READ {start_addr}-{start_addr+qtty-1} OK"
                                    break

                if any(success_flags):
                    last_query = current_time
                    if log_writer:
                        row = [f"{current_time:.3f}", t_str] + parsed + [pending_marker]
                        log_writer.writerow(row)
                        csv_file.flush()
                        pending_marker = ""
                    updated_this_cycle = True
            
            if updated_this_cycle:
                draw_screen(parsed, parsed, update_times, time.time(), marker_counter, life_char, raw_stats["bytes"], last_frame_info)
                
            time.sleep(0.01)

    except KeyboardInterrupt: pass
    finally:
        if 'fd' in locals(): os.close(fd)
        if csv_file: csv_file.close()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term_settings)
        sys.stdout.write("\nArrêt du Dashboard Logger.\n")

if __name__ == "__main__":
    try: main()
    except Exception as fatal_e:
        print(f"\n!!! ERREUR FATALE !!!\n{fatal_e}")
        import traceback
        traceback.print_exc()
