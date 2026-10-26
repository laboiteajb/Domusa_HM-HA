#!/usr/bin/env python3
"""
domusa_mqtt.py — Domusa Modbus → MQTT Bridge with Home Assistant Discovery
=========================================================================
Lit le flux Modbus série de la chaudière Domusa (esclave ID=3, FC=16)
et diffuse toutes les valeurs en MQTT avec auto-découverte Home Assistant.

Usage sur le RPi :
  sudo apt install python3-paho-mqtt
  python3 domusa_mqtt.py
"""

import os
import sys
import time
import json
import signal
import logging
import logging.handlers
import threading
import statistics

try:
    import RPi.GPIO as GPIO
except ImportError:
    # Pour le debug hors RPi
    class GPIO_Mock:
        BCM = 1
        IN = 1
        OUT = 0
        def setmode(self, a): pass
        def setup(self, a, b): pass
        def output(self, a, b): pass
        def input(self, a): return 0
        def cleanup(self): pass
    GPIO = GPIO_Mock()

try:
    import paho.mqtt.client as mqtt
    from paho.mqtt.client import CallbackAPIVersion
except ImportError:
    print("Erreur : paho-mqtt n'est pas installé.")
    print("Installer avec : sudo apt install python3-paho-mqtt")
    sys.exit(1)

# Désactivation des warnings GPIO
GPIO.setwarnings(False)

# ── CONFIGURATION ─────────────────────────────────────────────────────────────
SERIAL_PORT    = "/dev/serial0"
SERIAL_BAUD    = 9600

MQTT_BROKER    = "192.168.0.251"
MQTT_PORT      = 1883
MQTT_USER      = "mqtt"
MQTT_PASSWORD  = os.environ.get("MQTT_PASSWORD", "")   # défini dans /etc/default/domusa-mqtt (hors dépôt)
MQTT_CLIENT_ID = "domusa_boiler"
MQTT_BASE      = "domusa"

HA_PREFIX      = "homeassistant"          # préfixe de découverte HA
HA_STATUS      = f"{HA_PREFIX}/status"    # topic du message de naissance/testament HA
AVAIL_TOPIC    = f"{MQTT_BASE}/state"     # online / offline

# ── PARAMÈTRES MODIFIABLES SANS TOUCHER AU CODE ───────────────────────────────
# Fichier : /etc/default/domusa-mqtt  (une ligne NOM=valeur), puis : sudo systemctl restart domusa-mqtt
#   DOMUSA_LOG=quiet|changes|verbose   quiet = avertissements + 1 ligne de statut / 10 min
#                                      changes = + changements de phase/erreur (défaut)
#                                      verbose = tout, comme avant (beaucoup d'écritures sur la carte SD)
#   DOMUSA_STATUS_LOG_S=600            période de la ligne de statut (modes quiet/changes)
#   DOMUSA_BUS_STALE_S=60              sans trame Modbus depuis X s => entités chaudière "indisponibles"
#   DOMUSA_SENSOR_STALE_S=60           sans mesure valide depuis X s => capteur GPIO "indisponible"
#   DOMUSA_DIST_SAMPLES=5              mesures par cycle, la médiane est publiée
#   DOMUSA_DIST_MAX_CM=400             mesure au-delà = rejetée
#   DOMUSA_DIST_JUMP_CM=30             saut plus grand = confirmé par une 2e mesure cohérente
#   DOMUSA_CAPTURE=0|1                 1 = enregistre chaque changement de registre (reverse engineering)
def _env(name, default, cast=str):
    try:
        return cast(os.environ.get(name, default))
    except ValueError:
        return cast(default)

LOG_MODE         = _env("DOMUSA_LOG", "changes")
STATUS_LOG_S     = _env("DOMUSA_STATUS_LOG_S", "600", int)
BUS_STALE_S      = _env("DOMUSA_BUS_STALE_S", "60", int)
SENSOR_STALE_S   = _env("DOMUSA_SENSOR_STALE_S", "60", int)
DIST_SAMPLES     = _env("DOMUSA_DIST_SAMPLES", "5", int)
DIST_MAX_CM      = _env("DOMUSA_DIST_MAX_CM", "400", float)
DIST_JUMP_CM     = _env("DOMUSA_DIST_JUMP_CM", "30", float)
CAPTURE          = _env("DOMUSA_CAPTURE", "0") == "1"
CAPTURE_DIR      = "/var/lib/domusa"
CAPTURE_X_MAX    = _env("DOMUSA_CAPTURE_X_MAX", "5000000", int)   # octets/jour max pour les trames ignorées (lignes X)
CAPTURE_KEEP_DAYS = 21

# ── CONFIGURATION GPIO (RPi 3) ────────────────────────────────────────────────
# Broches physiques converties en BCM pour RPi.GPIO
GPIO_MODE = GPIO.BCM

# Activation des capteurs (Mettre à False pour désactiver la lecture/MQTT)
ENABLE_PELLET_DIST_1 = True
ENABLE_PELLET_DIST_2 = True
ENABLE_TEMP_ESC       = True
ENABLE_PROX_PELLET    = True
ENABLE_ALARM_IN       = True

# Pins (BCM)
PIN_DIST1_TRIG = 17    # Broche 11 (GPIO 17)
PIN_DIST1_ECHO = 27    # Broche 13 (GPIO 27)

PIN_DIST2_TRIG = 23    # Broche 16 (GPIO 23)
PIN_DIST2_ECHO = 22    # Broche 15 (GPIO 22)

PIN_TEMP_ESC   = 4     # Broche 07 (GPIO 04) - 1-Wire

PIN_PROX_PELLET = 18   # Broche 12 (GPIO 18) - TCRT5000 D0

PIN_ALARM_IN    = 25   # Broche 22 (GPIO 25) - Contact sec
# ─────────────────────────────────────────────────────────────────────────────

# ── Configuration des Logs ──────────────────────────────────────────────────
LOG_DIR = "/var/log/domusa"
LOG_FILE = os.path.join(LOG_DIR, "domusa-mqtt.log")

# On s'assure qu'on peut logger, sinon on fallback sur console seule
log_handlers = []
try:
    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR, exist_ok=True)
    
    file_handler = logging.handlers.TimedRotatingFileHandler(
        LOG_FILE, when="midnight", interval=1, backupCount=7
    )
    file_handler.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s", "%Y-%m-%d %H:%M:%S"))
    log_handlers.append(file_handler)
except Exception as e:
    print(f"Impossible de créer le fichier de log: {e}", file=sys.stderr)

# Handler console pour systemd (accessible via journalctl)
console_handler = logging.StreamHandler()
console_handler.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s", "%Y-%m-%d %H:%M:%S"))
log_handlers.append(console_handler)

logging.basicConfig(level=logging.INFO, handlers=log_handlers)
log = logging.getLogger("domusa_mqtt")

# ── Info appareil HA (groupement de toutes les entités) ──────────────────────
DEVICE = {
    "identifiers": ["domusa_pellet_boiler"],
    "name": "Chaudière Domusa Pellets",
    "manufacturer": "Domusa",
    "model": "Bioclass HM",
    "sw_version": "1.0",
}

AVAIL_BUS_TOPIC = f"{MQTT_BASE}/bus"    # online tant que des trames Modbus fraîches arrivent
_AV = {"payload_available": "online", "payload_not_available": "offline"}
AVAILABILITY_BUS = [{"topic": AVAIL_TOPIC, **_AV}, {"topic": AVAIL_BUS_TOPIC, **_AV}]
AVAILABILITY_BRIDGE = [{"topic": AVAIL_TOPIC, **_AV}]

# Capteurs GPIO disposant de leur propre disponibilité (mesure fraîche) : uid -> topic
GPIO_AVAIL = {
    "pellets_dist_1": f"{MQTT_BASE}/avail/pellets_dist_1",
    "pellets_dist_2": f"{MQTT_BASE}/avail/pellets_dist_2",
    "temp_esc_gpio":  f"{MQTT_BASE}/avail/temp_esc_gpio",
}
GPIO_DIGITAL = {"prox_pellet", "alarme_externe"}   # toujours lisibles : disponibilité du pont seule

_avail_state = {}
def set_avail(client, topic, online):
    """Publie online/offline seulement sur changement. Retourne True si changé."""
    val = "online" if online else "offline"
    if _avail_state.get(topic) == val:
        return False
    _avail_state[topic] = val
    client.publish(topic, val, qos=1, retain=True)
    return True

def avail_for(uid):
    if uid in GPIO_AVAIL:
        return [{"topic": AVAIL_TOPIC, **_AV}, {"topic": GPIO_AVAIL[uid], **_AV}]
    if uid in GPIO_DIGITAL:
        return AVAILABILITY_BRIDGE
    return AVAILABILITY_BUS

# Journal : verbose = tout ; sinon seulement changements / statut / avertissements
def vlog(msg, *args):
    if LOG_MODE == "verbose":
        log.info(msg, *args)

# ── Registres Modbus ──────────────────────────────────────────────────────────
REGISTER_NAMES = {
    0:  "temp_chaudiere",
    1:  "registre_r01",        # Température retour (option HotStream)
    2:  "consigne_chaudiere",
    4:  "timer_cycle_a",
    8:  "pression_eau",
    9:  "ventilateur_pct",
    11: "consigne_ecs",
    12: "capteur_flamme",
    14: "vis_pellets",
    16: "phase",
    19: "bouton_on_off",
    20: "registre_r20",
    21: "cendrier_b",
    22: "code_erreur",
    27: "fin_course_raclette_a",
    28: "fin_course_raclette_b",
    32: "relais_bitmask",
    35: "nettoyage",
    37: "demande_chaleur",
    40: "rpm_extracteur",
    41: "timer_cycle_b",
    42: "heures_totales",
    43: "nb_allumages",
    44: "kg_pellets",
    46: "consigne_ballon",
    49: "registre_r49",
    53: "relais_alarme",
    60: "temp_ecs",
}
TEMP_REGISTERS     = {0, 1, 2, 3, 11, 46, 60}   # R01 ajouté (temp retour)
PRESSURE_REGISTERS = {8}

# ── Décomposition Bitmask R32 (relais 230V) ───────────────────────────────────
# Source : documentation Domusa + reverse-engineering
R32_BITS = {
    0:  ("relais_b0",          "Relais b0",               "mdi:chart-scatter-plot", None),
    1:  ("relais_b1",          "Relais b1",               "mdi:chart-scatter-plot", None),
    2:  ("relais_b2",          "Relais b2",               "mdi:chart-scatter-plot", None),
    3:  ("relais_b3",          "Relais b3",               "mdi:chart-scatter-plot", None),
    4:  ("relais_b4",          "Relais b4",               "mdi:chart-scatter-plot", None),
    5:  ("relais_b5",          "Relais b5",               "mdi:chart-scatter-plot", None),
    6:  ("relais_b6",          "Relais b6",               "mdi:chart-scatter-plot", None),
    7:  ("relais_b7",          "Relais b7",               "mdi:chart-scatter-plot", None),
    10: ("relais_b10",         "Relais b10",              "mdi:chart-scatter-plot", None),
    11: ("relais_b11",         "Relais b11",              "mdi:chart-scatter-plot", None),
}

# ── Définitions capteurs HA Discovery ─────────────────────────────────────────
# (component, unique_id_suffix, nom_affiché, state_topic, extra_config)
HA_SENSORS = [
    # Températures
    ("sensor", "temp_chaudiere",    "T° Chaudière",
     f"{MQTT_BASE}/sensor/temp_chaudiere",
     {"unit_of_measurement": "°C", "device_class": "temperature", "state_class": "measurement"}),
    ("sensor", "registre_r01",       "Registre R01",
     f"{MQTT_BASE}/sensor/registre_r01",
     {"unit_of_measurement": "°C", "device_class": "temperature", "state_class": "measurement"}),
    ("sensor", "consigne_chaudiere","Consigne Chaudière",
     f"{MQTT_BASE}/sensor/consigne_chaudiere",
     {"unit_of_measurement": "°C", "device_class": "temperature", "state_class": "measurement"}),
    ("sensor", "consigne_ecs",      "Consigne ECS",
     f"{MQTT_BASE}/sensor/consigne_ecs",
     {"unit_of_measurement": "°C", "device_class": "temperature", "state_class": "measurement"}),
    ("sensor", "consigne_ballon",   "Consigne Ballon Tampon",
     f"{MQTT_BASE}/sensor/consigne_ballon",
     {"unit_of_measurement": "°C", "device_class": "temperature", "state_class": "measurement"}),
    ("sensor", "temp_ecs",          "T° ECS",
     f"{MQTT_BASE}/sensor/temp_ecs",
     {"unit_of_measurement": "°C", "device_class": "temperature", "state_class": "measurement"}),
    # Pression
    ("sensor", "pression_eau",      "Pression Circuit",
     f"{MQTT_BASE}/sensor/pression_eau",
     {"unit_of_measurement": "bar", "device_class": "pressure", "state_class": "measurement"}),
    # Analogiques
    ("sensor", "ventilateur_pct",   "Ventilateur %",
     f"{MQTT_BASE}/sensor/ventilateur_pct",
     {"unit_of_measurement": "%", "state_class": "measurement", "icon": "mdi:fan"}),
    ("sensor", "capteur_flamme",    "Capteur Flamme (Lumex)",
     f"{MQTT_BASE}/sensor/capteur_flamme",
     {"unit_of_measurement": "%", "state_class": "measurement", "icon": "mdi:fire"}),
    ("sensor", "rpm_extracteur",    "RPM Extracteur",
     f"{MQTT_BASE}/sensor/rpm_extracteur",
     {"unit_of_measurement": "RPM", "state_class": "measurement", "icon": "mdi:fan-speed-3"}),
    # Compteurs
    ("sensor", "heures_totales",    "Heures Totales Brûleur",
     f"{MQTT_BASE}/sensor/heures_totales",
     {"unit_of_measurement": "h", "state_class": "total_increasing",
      "device_class": "duration", "icon": "mdi:clock-outline"}),
    ("sensor", "nb_allumages",      "Nombre d'Allumages",
     f"{MQTT_BASE}/sensor/nb_allumages",
     {"state_class": "total_increasing", "icon": "mdi:counter"}),
    ("sensor", "kg_pellets",        "Consommation Pellets",
     f"{MQTT_BASE}/sensor/kg_pellets",
     {"unit_of_measurement": "kg", "state_class": "total_increasing",
      "icon": "mdi:weight-kilogram"}),
    # Cendriers
    ("sensor", "registre_r20",        "Registre R20",
     f"{MQTT_BASE}/sensor/registre_r20",
     {"state_class": "measurement", "icon": "mdi:numeric"}),
    ("sensor", "cendrier_b",        "Niveau Cendrier B",
     f"{MQTT_BASE}/sensor/cendrier_b",
     {"state_class": "measurement", "icon": "mdi:tray-full"}),
    # État
    ("sensor", "phase",             "Phase (code)",
     f"{MQTT_BASE}/sensor/phase",
     {"icon": "mdi:state-machine"}),
    ("sensor", "phase_label",       "Phase (clé)",
     f"{MQTT_BASE}/sensor/phase_label",
     {"icon": "mdi:state-machine"}),
    ("sensor", "phase_label_fr",    "Phase (libellé)",
     f"{MQTT_BASE}/sensor/phase_label_fr",
     {"icon": "mdi:state-machine"}),
    ("sensor", "code_erreur",       "Code Erreur",
     f"{MQTT_BASE}/sensor/code_erreur",
     {"icon": "mdi:alert-circle"}),
    ("sensor", "code_erreur_fr",    "Code Erreur (libellé)",
     f"{MQTT_BASE}/sensor/code_erreur_fr",
     {"icon": "mdi:alert-circle"}),
    ("sensor", "demande_chaleur",   "Demande Chaleur",
     f"{MQTT_BASE}/sensor/demande_chaleur",
     {"icon": "mdi:thermometer-chevron-up"}),
    ("sensor", "registre_r49",  "Registre R49",
     f"{MQTT_BASE}/sensor/registre_r49",
     {"icon": "mdi:counter", "state_class": "total"}),
    # Nouveaux capteurs GPIO
    ("sensor", "pellets_dist_1",    "Niveau Pellets 1",
     f"{MQTT_BASE}/sensor/pellets_dist_1",
     {"unit_of_measurement": "cm", "state_class": "measurement", "icon": "mdi:tray-arrow-down"}),
    ("sensor", "pellets_dist_2",    "Niveau Pellets 2",
     f"{MQTT_BASE}/sensor/pellets_dist_2",
     {"unit_of_measurement": "cm", "state_class": "measurement", "icon": "mdi:tray-arrow-down"}),
    ("sensor", "temp_esc_gpio",     "T° ESC (externe)",
     f"{MQTT_BASE}/sensor/temp_esc_gpio",
     {"unit_of_measurement": "°C", "device_class": "temperature", "state_class": "measurement"}),
]

# (unique_id_suffix, nom_affiché, state_topic, extra_config)
HA_BINARY_SENSORS = [
    ("vis_pellets",     "Vis Pellets (R14)",    f"{MQTT_BASE}/binary/vis_pellets",
     {"icon": "mdi:screw-flat-top"}),
    ("nettoyage",       "Nettoyage Mécanique", f"{MQTT_BASE}/binary/nettoyage",
     {"icon": "mdi:broom"}),
    ("bouton_on_off",   "Bouton ON/OFF",       f"{MQTT_BASE}/binary/bouton_on_off",
     {"icon": "mdi:power"}),
    ("relais_alarme",   "Relais Alarme",       f"{MQTT_BASE}/binary/relais_alarme",
     {"device_class": "safety"}),
    # Bits R32 — composants physiques 230V
    ("relais_b0",        "Relais b0",             f"{MQTT_BASE}/binary/relais_b0",
     {"icon": "mdi:chart-scatter-plot"}),
    ("relais_b1",        "Relais b1",             f"{MQTT_BASE}/binary/relais_b1",
     {"icon": "mdi:chart-scatter-plot"}),
    ("relais_b2",        "Relais b2",             f"{MQTT_BASE}/binary/relais_b2",
     {"icon": "mdi:chart-scatter-plot"}),
    ("relais_b3",        "Relais b3",             f"{MQTT_BASE}/binary/relais_b3",
     {"icon": "mdi:chart-scatter-plot"}),
    ("relais_b4",        "Relais b4",             f"{MQTT_BASE}/binary/relais_b4",
     {"icon": "mdi:chart-scatter-plot"}),
    ("relais_b5",        "Relais b5",             f"{MQTT_BASE}/binary/relais_b5",
     {"icon": "mdi:chart-scatter-plot"}),
    ("relais_b6",        "Relais b6",             f"{MQTT_BASE}/binary/relais_b6",
     {"icon": "mdi:chart-scatter-plot"}),
    ("relais_b7",        "Relais b7",             f"{MQTT_BASE}/binary/relais_b7",
     {"icon": "mdi:chart-scatter-plot"}),
    ("relais_b10",       "Relais b10",            f"{MQTT_BASE}/binary/relais_b10",
     {"icon": "mdi:chart-scatter-plot"}),
    ("relais_b11",       "Relais b11",            f"{MQTT_BASE}/binary/relais_b11",
     {"icon": "mdi:chart-scatter-plot"}),
    # Nouveaux capteurs GPIO
    ("prox_pellet",     "Proximité Pellets",    f"{MQTT_BASE}/binary/prox_pellet",
     {"icon": "mdi:sensor"}),
    ("alarme_externe",  "Alarme Externe",       f"{MQTT_BASE}/binary/alarme_externe",
     {"device_class": "problem", "icon": "mdi:alert-outline"}),
]

# ── Ajout automatique de tous les registres R00-R69 manquants dans HA Discovery ──
covered_indices = set()
for comp, uid, name, topic, extra in HA_SENSORS:
    for idx, key in REGISTER_NAMES.items():
        if key == uid:
            covered_indices.add(idx)

for i in range(70):
    if i not in covered_indices:
        HA_SENSORS.append(("sensor", f"raw_R{i:02d}", f"Registre R{i:02d}", f"{MQTT_BASE}/raw/R{i:02d}", {"icon": "mdi:numeric"}))


# ── CRC Modbus ────────────────────────────────────────────────────────────────
def modbus_crc(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


# ── Parse trame Modbus ────────────────────────────────────────────────────────
def parse_frame(frame: bytes):
    if len(frame) < 9:
        return None
    if frame[0] != 3 or frame[1] != 0x10:
        return None
    num_regs   = (frame[4] << 8) | frame[5]
    byte_count = frame[6]
    frame_len  = 7 + byte_count + 2
    if len(frame) < frame_len or num_regs != 70 or byte_count != 140:
        return None
    frame = frame[:frame_len]
    if modbus_crc(frame[:-2]) != ((frame[-1] << 8) | frame[-2]):
        return None
    data_bytes = frame[7:-2]
    values = []
    for j in range(70):
        idx = j * 2
        raw = (data_bytes[idx] << 8) | data_bytes[idx + 1]
        values.append(raw if raw < 32768 else raw - 65536)
    return values


# ── Build JSON payload ────────────────────────────────────────────────────────
PHASE_MAP = {
    0:  "eteint",
    1:  "nettoyage_initial",
    3:  "allumage_1",
    4:  "allumage_2",
    5:  "allumage_3",
    9:  "detection_flamme",
    10: "modulation_travail",
    11: "chauffe_normale",
    12: "modulation",
    14: "nettoyage_en_marche",
    16: "standby",
    17: "nettoyage_final",
    18: "post_ventilation",
    19: "blocage_securite",
}

PHASE_LABELS_FR = {
    0:  "Éteint",
    1:  "Nettoyage initial",
    3:  "Allumage (phase 1)",
    4:  "Allumage (phase 2)",
    5:  "Allumage (phase 3)",
    9:  "Détection flamme",
    10: "Travail / Modulation",
    11: "Chauffe normale",
    12: "Modulation",
    14: "Nettoyage en marche",
    16: "Standby",
    17: "Nettoyage final",
    18: "Post-ventilation",
    19: "Blocage sécurité",
}

ERROR_CODES_FR = {
    0:  "Pas d'erreur (Système OK)",
    1:  "Sonde de chaudière Sc circuit ouvert",
    2:  "Sonde de chaudière Sc court-circuitée",
    3:  "Sonde d'E.C.S. Sa circuit ouvert",
    4:  "Sonde d'E.C.S. Sa court-circuitée",
    5:  "Surchauffe de l'arrivée de combustible, Te",
    6:  "Erreur à l'allumage",
    7:  "Erreur au début du Cleaning (Nettoyage des cendres)",
    8:  "Erreur de Cleaning (Nettoyage des cendres)",
    9:  "Erreur fin de course FCp (Nettoyage des cendres)",
    10: "Surchauffe de la chaudière",
    11: "Thermostat de sécurité, Ts",
    12: "Extraction du brûleur, FCq",
    13: "Dépression d'air insuffisante",
    14: "Chute de la dépression d'air",
    15: "Dépression d'air insuffisante au pre-nettoyage",
    18: "Erreur du capteur de pression d'eau",
    19: "Basse pression d'eau",
    20: "Anomalie de la soupape de sécurité",
    21: "Erreur du capteur de pression d'air",
    22: "Dépression d'air excessive dans le foyer",
    23: "Surpression d'air excessive dans le foyer",
    25: "Calibration défectueuse",
    26: "Erreur de communication avec le Système d'aspiration CVS",
    27: "Blocage du Système d'aspiration CVS",
    29: "Capteur de niveau de combustible",
    30: "Sonde d'entrée Sr1 circuit ouvert",
    31: "Sonde d'entrée Sr1 court-circuitée",
    32: "Sonde d'entrée Sr2 circuit ouvert",
    33: "Sonde d'entrée Sr2 court-circuitée",
    34: "Sonde extérieure Sext circuit ouvert",
    35: "Sonde extérieure Sext court-circuitée",
    36: "Changement du DIP-switch mauvaise",
    37: "Erreur de communication avec le Kit hydraulique BIO",
    38: "Dépression d'air insuffisante dans le foyer",
    39: "Vitesse de ventilateur insuffisant",
    40: "Chute de la vitesse du ventilateur",
    41: "Chute durable de la vitesse du ventilateur",
    42: "Erreur de communication avec le Kit hydraulique BIO (carte électrique)",
    43: "Cendrier plein",
    44: "Maintenance de la chaudière",
    45: "Sonde du ballon BT Sbt circuit ouvert",
    46: "Sonde du ballon BT Sbt court-circuitée",
    47: "Erreur de communication avec le détecteur de niveau de combustible",
}

def build_payload(regs: list) -> dict:
    payload = {}
    for idx, name in REGISTER_NAMES.items():
        raw = regs[idx]
        if idx in TEMP_REGISTERS:
            payload[name] = round(raw / 10.0, 1)
        elif idx in PRESSURE_REGISTERS:
            payload[name] = round(raw / 100.0, 2)
        else:
            payload[name] = raw
    phase_code = regs[16]
    payload["phase_label"]    = PHASE_MAP.get(phase_code, f"inconnu_{phase_code}")
    payload["phase_label_fr"] = PHASE_LABELS_FR.get(phase_code, f"Inconnu ({phase_code})")
    
    # Traduction Code Erreur
    # R22 reste à 0 sur certains défauts (ex. E12 affiché, R18=12) : on prend R18 à défaut
    err_code = regs[22] or regs[18]
    payload["code_erreur"] = err_code
    payload["code_erreur_fr"] = ERROR_CODES_FR.get(err_code, f"Inconnu ({err_code})")
    
    # Décomposition complète R32 (tous les bits documentés)
    r32 = regs[32]
    for bit, (key, _name, _icon, _dc) in R32_BITS.items():
        payload[key] = bool(r32 & (1 << bit))
    payload["timestamp"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    return payload


# ── Lecture Capteurs GPIO ──────────────────────────────────────────────────────
def get_distance(trig, echo, label="?"):
    """Mesure la distance en cm avec un HC-SR04."""
    # S'assurer que le trig est bas
    GPIO.output(trig, False)
    time.sleep(0.01)
    
    # Pulse trigger de 10us
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    start_time = time.perf_counter()
    stop_time  = time.perf_counter()
    
    # Attente du début de l'Echo (front montant)
    timeout_start = time.perf_counter()
    while GPIO.input(echo) == 0:
        start_time = time.perf_counter()
        if start_time - timeout_start > 0.05:
            # log.debug(f"US {label} | Timeout: pas de début d'Echo reçu")
            return -1

    # Attente de la fin de l'Echo (front descendant)
    timeout_stop = time.perf_counter()
    while GPIO.input(echo) == 1:
        stop_time = time.perf_counter()
        if stop_time - timeout_stop > 0.05:
            # log.debug(f"US {label} | Timeout: l'Echo reste à l'état haut")
            return -2

    elapsed = stop_time - start_time
    dist = (elapsed * 34300) / 2
    return round(dist, 1)

def get_ds18b20_temp():
    """Lit la température du DS18B20 via sysfs."""
    base_dir = '/sys/bus/w1/devices/'
    try:
        device_folders = [f for f in os.listdir(base_dir) if f.startswith('28-')]
        if not device_folders: return -99
        device_file = base_dir + device_folders[0] + '/w1_slave'
        with open(device_file, 'r') as f:
            lines = f.readlines()
        if lines[0].strip()[-3:] != 'YES': return -99
        equals_pos = lines[1].find('t=')
        if equals_pos != -1:
            temp_string = lines[1][equals_pos+2:]
            return round(float(temp_string) / 1000.0, 1)
    except Exception:
        pass
    return -99

def measure_median(trig, echo):
    """Médiane de DIST_SAMPLES mesures valides ; None s'il y en a trop peu (capteur muet ou bruyant)."""
    vals = []
    for _ in range(max(1, DIST_SAMPLES)):
        d = get_distance(trig, echo)
        if 2.0 <= d <= DIST_MAX_CM:
            vals.append(d)
        time.sleep(0.06)
    if len(vals) < max(1, DIST_SAMPLES // 2 + 1):
        return None
    return round(statistics.median(vals), 1)

def accept_distance(state, new):
    """Rejette un saut brutal tant qu'une 2e mesure cohérente ne le confirme pas."""
    last = state.get("last")
    if last is None or abs(new - last) <= DIST_JUMP_CM:
        state["last"], state["pending"] = new, None
        return new
    pend = state.get("pending")
    if pend is not None and abs(new - pend) <= DIST_JUMP_CM / 2:
        state["last"], state["pending"] = new, None
        return new
    state["pending"] = new
    return None

def check_stale(client, name, topic, st, now):
    """Marque le capteur indisponible / disponible selon la fraîcheur de sa dernière mesure valide."""
    fresh = (now - st["ok_at"]) <= SENSOR_STALE_S
    if set_avail(client, topic, fresh):
        if fresh:
            log.info("GPIO | %s : mesures valides de nouveau", name)
        else:
            log.warning("GPIO | %s : aucune mesure valide depuis %d s -> indisponible", name, SENSOR_STALE_S)

# Global pour stocker les dernières valeurs GPIO
gpio_data = {
    "dist1": -1, "dist2": -1, "temp_esc": -99, "prox": False, "alarm": False
}

def periodic_gpio_reader(client, running_flag):
    """Thread de lecture périodique des GPIO (évite de bloquer le Modbus)."""
    log.info("Thread GPIO démarré.")
    first_log = True
    dist_state = {}
    last_prox = last_alarm = None
    while running_flag():
        try:
            now = time.time()
            # 1. Distances (HC-SR04) : médiane + rejet des sauts non confirmés
            for flag, name, trig, echo, key in (
                (ENABLE_PELLET_DIST_1, "Dist 1", PIN_DIST1_TRIG, PIN_DIST1_ECHO, "pellets_dist_1"),
                (ENABLE_PELLET_DIST_2, "Dist 2", PIN_DIST2_TRIG, PIN_DIST2_ECHO, "pellets_dist_2"),
            ):
                if not flag:
                    continue
                st = dist_state.setdefault(key, {"ok_at": time.time()})
                raw = measure_median(trig, echo)
                if raw is not None:
                    v = accept_distance(st, raw)
                    if v is not None:
                        st["ok_at"] = now
                        client.publish(f"{MQTT_BASE}/sensor/{key}", str(v), qos=0, retain=True)
                        vlog("GPIO | %s : %s cm", name, v)
                        capture.gpio(key, v)
                check_stale(client, name, GPIO_AVAIL[key], st, time.time())

            # 2. Température DS18B20
            if ENABLE_TEMP_ESC:
                st = dist_state.setdefault("temp_esc_gpio", {"ok_at": time.time()})
                t_esc = get_ds18b20_temp()
                if t_esc != -99:
                    st["ok_at"] = now
                    client.publish(f"{MQTT_BASE}/sensor/temp_esc_gpio", str(t_esc), qos=0, retain=True)
                    vlog("GPIO | Temp ESC : %s°C", t_esc)
                    capture.gpio("temp_esc_gpio", t_esc)
                check_stale(client, "Temp ESC", GPIO_AVAIL["temp_esc_gpio"], st, time.time())

            # 3. Digital (TCRT5000 et Alarme)
            # Logique inversée pour TCRT5000 (0 = Libre, 1 = Détecté ?)
            if ENABLE_PROX_PELLET:
                prox = GPIO.input(PIN_PROX_PELLET) 
                client.publish(f"{MQTT_BASE}/binary/prox_pellet", "OFF" if prox else "ON", retain=True)
                if prox != last_prox:
                    capture.gpio("prox_pellet", int(bool(prox)))
                    last_prox = prox
                if first_log:
                    log.info(f"GPIO INITIAL | Prox: {'DÉTECTÉ' if prox else 'Libre'}")

            # Alarme : supposé normalement ouvert (tiré à la masse si actif)
            if ENABLE_ALARM_IN:
                alarm = not GPIO.input(PIN_ALARM_IN)
                client.publish(f"{MQTT_BASE}/binary/alarme_externe", "ON" if alarm else "OFF", retain=True)
                if alarm != last_alarm:
                    capture.gpio("alarme_externe", int(bool(alarm)))
                    last_alarm = alarm
                if first_log:
                    log.info(f"GPIO INITIAL | Alarme: {'ALERTE' if alarm else 'OK'}")

            first_log = False
            
        except Exception as e:
            log.error("Erreur lecture GPIO : %s", e)
            
        time.sleep(2.0) # Lecture toutes les 2 secondes

# ── HA Discovery ──────────────────────────────────────────────────────────────
def publish_discovery(client: mqtt.Client) -> None:
    """Publie tous les messages de découverte HA."""
    # Mapping des flags d'activation pour le filtrage
    ENABLED_FLAGS = {
        "pellets_dist_1": ENABLE_PELLET_DIST_1,
        "pellets_dist_2": ENABLE_PELLET_DIST_2,
        "temp_esc_gpio":  ENABLE_TEMP_ESC,
        "prox_pellet":    ENABLE_PROX_PELLET,
        "alarme_externe": ENABLE_ALARM_IN,
    }

    published_sensors = 0
    published_binary = 0

    # Sensors
    for comp, uid, name, state_topic, extra in HA_SENSORS:
        # Filtrer si désactivé
        if uid in ENABLED_FLAGS and not ENABLED_FLAGS[uid]:
            continue

        config = {
            "name": name,
            "unique_id": f"domusa_{uid}",
            "state_topic": state_topic,
            "availability": avail_for(uid),
            "availability_mode": "all",
            "device": DEVICE,
            **extra,
        }
        topic = f"{HA_PREFIX}/{comp}/domusa_{uid}/config"
        client.publish(topic, json.dumps(config), qos=1, retain=True)
        published_sensors += 1

    # Binary sensors
    for uid, name, state_topic, extra in HA_BINARY_SENSORS:
        # Filtrer si désactivé
        if uid in ENABLED_FLAGS and not ENABLED_FLAGS[uid]:
            continue

        config = {
            "name": name,
            "unique_id": f"domusa_{uid}",
            "state_id": f"domusa_{uid}", # Ajout pour cohérence
            "state_topic": state_topic,
            "payload_on": "ON",
            "payload_off": "OFF",
            "availability": avail_for(uid),
            "availability_mode": "all",
            "device": DEVICE,
            **extra,
        }
        topic = f"{HA_PREFIX}/binary_sensor/domusa_{uid}/config"
        client.publish(topic, json.dumps(config), qos=1, retain=True)
        published_binary += 1

    log.info("HA Discovery : %d sensors + %d binary_sensors publiés",
             published_sensors, published_binary)


# ── Publish data ──────────────────────────────────────────────────────────────
def publish_all(client: mqtt.Client, regs: list) -> None:
    payload = build_payload(regs)

    # 1. JSON complet
    client.publish(f"{MQTT_BASE}/status", json.dumps(payload), qos=1, retain=True)

    # 2. Topics sensor/ (valeurs converties) — lus par HA
    for idx, name in REGISTER_NAMES.items():
        raw = regs[idx]
        if idx in TEMP_REGISTERS:
            val = f"{raw / 10.0:.1f}"
        elif idx in PRESSURE_REGISTERS:
            val = f"{raw / 100.0:.2f}"
        else:
            val = str(raw)
        client.publish(f"{MQTT_BASE}/sensor/{name}", val, qos=0, retain=True)

    # Topics sensor/ supplémentaires dérivés
    client.publish(f"{MQTT_BASE}/sensor/phase_label",    payload["phase_label"],    qos=0, retain=True)
    client.publish(f"{MQTT_BASE}/sensor/phase_label_fr", payload["phase_label_fr"], qos=0, retain=True)
    client.publish(f"{MQTT_BASE}/sensor/code_erreur_fr", payload["code_erreur_fr"], qos=0, retain=True)
    client.publish(f"{MQTT_BASE}/sensor/code_erreur",    str(payload["code_erreur"]), qos=0, retain=True)
    client.publish(f"{MQTT_BASE}/sensor/demande_chaleur", str(regs[37]),            qos=0, retain=True)
    client.publish(f"{MQTT_BASE}/sensor/registre_r49", str(regs[49]),           qos=0, retain=True)

    # 3. Topics binary/ (ON/OFF) — lus par HA binary_sensor
    binary = {
        "vis_pellets":      regs[14] != 0,
        "nettoyage":        regs[35] != 0,
        "bouton_on_off":    regs[19] != 0,
        "relais_alarme":    regs[53] != 0,
    }
    # Ajout des bits R32 (tous les composants 230V documentés)
    r32 = regs[32]
    for bit, (key, _name, _icon, _dc) in R32_BITS.items():
        binary[key] = bool(r32 & (1 << bit))
    for name, val in binary.items():
        client.publish(f"{MQTT_BASE}/binary/{name}", "ON" if val else "OFF", qos=0, retain=True)

    # 4. Registres bruts
    for i, val in enumerate(regs):
        client.publish(f"{MQTT_BASE}/raw/R{i:02d}", str(val), qos=0, retain=True)

    log_status(payload)


_status = {"phase": None, "err": None, "at": 0.0}

def log_status(payload):
    """Journal selon DOMUSA_LOG : verbose = chaque trame ; sinon changements + statut périodique."""
    line = ("MQTT | Phase=%-20s T°=%.1f°C Cons=%.1f°C Press=%.2fbar Venti=%d%% Flamme=%d%% RPM=%d Err=%d" % (
        payload["phase_label"], payload["temp_chaudiere"], payload["consigne_chaudiere"],
        payload["pression_eau"], payload["ventilateur_pct"], payload["capteur_flamme"],
        payload["rpm_extracteur"], payload["code_erreur"]))
    now = time.time()
    if LOG_MODE == "verbose":
        log.info(line)
        return
    changed = False
    if payload["phase_label"] != _status["phase"]:
        if payload["phase_label"].startswith("inconnu_"):
            log.warning("Phase inconnue détectée : %s", payload["phase_label"])
        changed = changed or LOG_MODE == "changes"
        _status["phase"] = payload["phase_label"]
    if payload["code_erreur"] != _status["err"]:
        changed = changed or LOG_MODE == "changes"
        _status["err"] = payload["code_erreur"]
    if changed or now - _status["at"] >= STATUS_LOG_S:
        log.info(line)
        _status["at"] = now


# ── Capture des trames (reverse engineering) ──────────────────────────────────
class Capture:
    """Écrit chaque CHANGEMENT de registre (et les capteurs GPIO) dans un CSV du jour, avec instantané complet toutes les 10 min."""
    def __init__(self, enabled):
        self.enabled = enabled
        self.prev = None
        self.snap_at = 0.0
        self.fh = None
        self.day = None
        self.flush_at = 0.0
        self.x_day = None
        self.x_bytes = 0
        self.lock = threading.Lock()
        if enabled:
            try:
                os.makedirs(CAPTURE_DIR, exist_ok=True)
                log.info("Capture activée -> %s (DOMUSA_CAPTURE=1)", CAPTURE_DIR)
            except Exception as e:
                log.warning("Capture impossible (%s) : désactivée", e)
                self.enabled = False

    def _file(self, now):
        day = time.strftime("%Y%m%d", time.localtime(now))
        if day != self.day:
            if self.fh:
                self.fh.close()
            self.fh = open(os.path.join(CAPTURE_DIR, f"capture-{day}.csv"), "a", buffering=1 << 16)
            self.day = day
            cutoff = now - CAPTURE_KEEP_DAYS * 86400
            for f in os.listdir(CAPTURE_DIR):
                fp = os.path.join(CAPTURE_DIR, f)
                if f.startswith("capture-") and os.path.getmtime(fp) < cutoff:
                    os.remove(fp)
        return self.fh

    def _ts(self, now):
        return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(now)) + ".%03d" % int((now % 1) * 1000)

    def _maybe_flush(self, now):
        if now - self.flush_at > 30 and self.fh:
            self.fh.flush()
            self.flush_at = now

    def frame(self, regs, now):
        if not self.enabled:
            return
        with self.lock:
            try:
                fh = self._file(now)
                ts = self._ts(now)
                if self.prev is None or now - self.snap_at >= 600:
                    fh.write("%s,S,%s\n" % (ts, " ".join(str(v) for v in regs)))
                    self.snap_at = now
                else:
                    for i, (a, b) in enumerate(zip(self.prev, regs)):
                        if a != b:
                            fh.write("%s,R,%d,%d,%d\n" % (ts, i, a, b))
                self.prev = list(regs)
                self._maybe_flush(now)
            except Exception as e:
                log.warning("Capture : %s", e)
                self.enabled = False

    def other(self, data, now):
        """Octets du bus qui ne sont pas une trame 'écriture de 70 registres' (autres esclaves, lectures, réponses...)."""
        if not self.enabled or not data:
            return
        with self.lock:
            try:
                fh = self._file(now)
                if self.x_day != self.day:
                    self.x_day, self.x_bytes = self.day, 0
                if self.x_bytes > CAPTURE_X_MAX:
                    return
                self.x_bytes += len(data)
                fh.write("%s,X,%s\n" % (self._ts(now), data.hex()))
                self._maybe_flush(now)
            except Exception as e:
                log.warning("Capture : %s", e)
                self.enabled = False

    def gpio(self, name, value):
        if not self.enabled:
            return
        now = time.time()
        with self.lock:
            try:
                fh = self._file(now)
                fh.write("%s,G,%s,%s\n" % (self._ts(now), name, value))
                self._maybe_flush(now)
            except Exception as e:
                log.warning("Capture : %s", e)
                self.enabled = False

capture = Capture(CAPTURE)


# ── Callbacks MQTT ────────────────────────────────────────────────────────────
bus = {"online": False, "last_frame": 0.0}

def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        log.info("Connecté au broker MQTT %s:%d", MQTT_BROKER, MQTT_PORT)
        # Disponibilité (repart de zéro : tout est republié)
        _avail_state.clear()
        client.publish(AVAIL_TOPIC, "online", qos=1, retain=True)
        set_avail(client, AVAIL_BUS_TOPIC, bus["online"])
        # Abonnement au topic de naissance HA (réenvoi discovery si HA redémarre)
        client.subscribe(HA_STATUS, qos=1)
        log.info("Abonné à %s", HA_STATUS)
        # Envoi initial des messages de découverte
        publish_discovery(client)
    else:
        log.error("Échec connexion MQTT, code=%s", reason_code)


def on_disconnect(client, userdata, flags, reason_code, properties):
    if reason_code != 0:
        log.warning("Déconnecté du broker MQTT (code=%s), reconnexion auto…", reason_code)


def on_message(client, userdata, msg):
    """Réception du message de naissance HA → re-publier la découverte."""
    payload = msg.payload.decode(errors="ignore").strip()
    log.debug("Message reçu sur %s : %s", msg.topic, payload)
    if msg.topic == HA_STATUS and payload == "online":
        log.info("Home Assistant (re)démarré → re-publication de la découverte MQTT")
        client.publish(AVAIL_TOPIC, "online", qos=1, retain=True)
        publish_discovery(client)


def drain_buffer(buffer, other):
    """Extrait les trames valides (écriture 70 registres, esclave 3) ; les autres octets vont dans `other`.
    Retourne (buffer restant, derniers registres ou None)."""
    regs_out = None
    while len(buffer) >= 9:
        if buffer[0] == 3 and buffer[1] == 0x10:
            byte_count = buffer[6]
            frame_len  = 7 + byte_count + 2
            if 9 <= frame_len < 200:
                if len(buffer) < frame_len:
                    break
                regs = parse_frame(bytes(buffer[:frame_len]))
                if regs is not None:
                    regs_out = regs
                    buffer   = buffer[frame_len:]
                    continue
        other.append(buffer[0])
        buffer = buffer[1:]
    return buffer, regs_out


# ── Vérification des prérequis ────────────────────────────────────────────────
def check_prerequisites() -> bool:
    """Vérifie l'environnement au démarrage et logue chaque point. Retourne True si tout est OK."""
    ok = True

    log.info("─── Vérification des prérequis ───────────────────────────────")

    # 1. Résolution du port série → doit pointer sur ttyAMA0 (PL011) et non ttyS0 (mini-UART)
    try:
        target = os.readlink(SERIAL_PORT) if os.path.islink(SERIAL_PORT) else SERIAL_PORT
        if "ttyAMA" in target:
            log.info("  [OK] UART  : %s → %s  (PL011 — UART matériel complet)", SERIAL_PORT, target)
        elif "ttyS" in target:
            log.error("  [KO] UART  : %s → %s  (mini-UART) — Bluetooth non désactivé !")
            log.error("       FIX   : Ajouter 'dtoverlay=disable-bt' dans /boot/firmware/config.txt")
            log.error("               puis : sudo systemctl disable hciuart && sudo reboot")
            ok = False
        else:
            log.warning("  [??] UART  : %s → %s  (inconnu)", SERIAL_PORT, target)
    except Exception as e:
        log.error("  [KO] UART  : impossible de résoudre %s : %s", SERIAL_PORT, e)
        ok = False

    # 2. Permissions sur le port série
    try:
        fd = os.open(SERIAL_PORT, os.O_RDONLY | os.O_NONBLOCK)
        os.close(fd)
        log.info("  [OK] PERMS : %s accessible en lecture", SERIAL_PORT)
    except PermissionError:
        log.error("  [KO] PERMS : %s — permission refusée", SERIAL_PORT)
        log.error("       FIX   : sudo chown root:dialout %s && sudo chmod 660 %s",
                  SERIAL_PORT, SERIAL_PORT)
        log.error("       PERM  : Vérifier aussi : sudo usermod -aG dialout $USER")
        ok = False
    except Exception as e:
        log.error("  [KO] PERMS : %s — %s", SERIAL_PORT, e)
        ok = False

    # 3. paho-mqtt version
    try:
        import paho.mqtt
        ver = getattr(paho.mqtt, "__version__", "?")
        log.info("  [OK] PAHO  : paho-mqtt %s installé", ver)
    except ImportError:
        log.error("  [KO] PAHO  : paho-mqtt non installé")
        log.error("       FIX   : sudo apt install python3-paho-mqtt")
        ok = False

    # 4. Joignabilité du broker MQTT (TCP)
    import socket
    try:
        s = socket.create_connection((MQTT_BROKER, MQTT_PORT), timeout=3)
        s.close()
        log.info("  [OK] BROKER: %s:%d joignable", MQTT_BROKER, MQTT_PORT)
    except Exception as e:
        log.error("  [KO] BROKER: %s:%d injoignable — %s", MQTT_BROKER, MQTT_PORT, e)
        ok = False

    # 5. Service Bluetooth (information)
    try:
        import subprocess
        bt = subprocess.run(
            ["systemctl", "is-active", "hciuart"],
            capture_output=True, text=True, timeout=3
        )
        bt_state = bt.stdout.strip()
        if bt_state == "active":
            log.warning("  [!!] BT    : service hciuart ACTIF — risque de conflit UART")
            log.warning("       FIX   : sudo systemctl disable hciuart && sudo reboot")
        else:
            log.info("  [OK] BT    : service hciuart %s (pas de conflit)", bt_state)
    except Exception:
        log.info("  [--] BT    : vérification hciuart impossible (non critique)")

    log.info("──────────────────────────────────────────────────────────────")
    if ok:
        log.info("Tous les prérequis sont satisfaits — démarrage du bridge.")
    else:
        log.error("Des prérequis sont manquants — voir les erreurs ci-dessus.")
    return ok


# ── MAIN ──────────────────────────────────────────────────────────────────────
def main():
    log.info("=== DOMUSA MQTT BRIDGE (HA Discovery) ===")
    log.info("Port série : %s @ %d bauds", SERIAL_PORT, SERIAL_BAUD)
    log.info("Broker MQTT : %s:%d", MQTT_BROKER, MQTT_PORT)
    log.info("HA Discovery prefix : %s", HA_PREFIX)

    if not check_prerequisites():
        sys.exit(1)

    # ── MQTT ──────────────────────────────────────────────────────────────────
    client = mqtt.Client(
        callback_api_version=CallbackAPIVersion.VERSION2,
        client_id=MQTT_CLIENT_ID,
        clean_session=True,
    )
    client.will_set(AVAIL_TOPIC, "offline", qos=1, retain=True)
    client.on_connect    = on_connect
    client.on_disconnect = on_disconnect
    client.on_message    = on_message

    if MQTT_USER:
        client.username_pw_set(MQTT_USER, MQTT_PASSWORD)

    try:
        client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    except Exception as e:
        log.error("Impossible de se connecter au broker MQTT : %s", e)
        sys.exit(1)

    client.loop_start()

    # ── GPIO SETUP ────────────────────────────────────────────────────────────
    GPIO.setmode(GPIO_MODE)
    GPIO.setup(PIN_DIST1_TRIG, GPIO.OUT)
    GPIO.setup(PIN_DIST1_ECHO, GPIO.IN)
    GPIO.setup(PIN_DIST2_TRIG, GPIO.OUT)
    GPIO.setup(PIN_DIST2_ECHO, GPIO.IN)
    GPIO.setup(PIN_PROX_PELLET, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(PIN_ALARM_IN,    GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.output(PIN_DIST1_TRIG, False)
    GPIO.output(PIN_DIST2_TRIG, False)

    # ── Thread GPIO ───────────────────────────────────────────────────────────
    running = True
    gpio_thread = threading.Thread(
        target=periodic_gpio_reader, 
        args=(client, lambda: running), 
        daemon=True
    )
    gpio_thread.start()

    # ── PORT SÉRIE ─────────────────────────────────────────────────────────────
    os.system(f"stty -F {SERIAL_PORT} {SERIAL_BAUD} raw -echo min 0 time 10 2>/dev/null")
    time.sleep(0.2)

    try:
        fd = os.open(SERIAL_PORT, os.O_RDONLY | os.O_NONBLOCK)
    except Exception as e:
        log.error("Impossible d'ouvrir %s : %s", SERIAL_PORT, e)
        client.publish(AVAIL_TOPIC, "offline", qos=1, retain=True)
        client.loop_stop()
        sys.exit(1)

    log.info("Port série ouvert. En attente de trames Modbus…")

    # ── Arrêt propre ───────────────────────────────────────────────────────────
    running = True

    def shutdown(sig, frame):
        nonlocal running
        log.info("Signal reçu, arrêt propre…")
        running = False

    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # ── Boucle principale ──────────────────────────────────────────────────────
    buffer       = bytearray()
    other        = bytearray()
    other_last   = 0.0
    last_regs    = None
    last_publish = 0.0

    while running:
        # Lecture série
        try:
            chunk = os.read(fd, 1024)
            if chunk:
                buffer.extend(chunk)
        except BlockingIOError:
            pass
        except OSError as e:
            log.error("Erreur lecture série : %s", e)
            time.sleep(1)

        # Parsing (les octets non reconnus sont gardés pour la capture, lignes X)
        n_other = len(other)
        buffer, new_regs = drain_buffer(buffer, other)
        updated = new_regs is not None
        if updated:
            last_regs = new_regs
        if len(other) != n_other:
            other_last = time.time()
        if other and time.time() - other_last > 0.05:      # silence sur le bus : fin d'un paquet
            capture.other(bytes(other), other_last)
            other.clear()

        # Publication MQTT : uniquement sur trame fraîche (plus de republication de valeurs périmées)
        now = time.time()
        if updated and last_regs is not None:
            bus["last_frame"] = now
            if not bus["online"]:
                bus["online"] = True
                log.info("Trames Modbus reçues : entités chaudière disponibles")
            set_avail(client, AVAIL_BUS_TOPIC, True)
            publish_all(client, last_regs)
            capture.frame(last_regs, now)
            last_publish = now
        elif bus["online"] and now - bus["last_frame"] > BUS_STALE_S:
            bus["online"] = False
            set_avail(client, AVAIL_BUS_TOPIC, False)
            log.warning("Aucune trame Modbus depuis %d s -> entités chaudière indisponibles", BUS_STALE_S)

        time.sleep(0.05)

    # ── Nettoyage ──────────────────────────────────────────────────────────────
    log.info("Fermeture…")
    running = False
    gpio_thread.join(timeout=2.0)
    GPIO.cleanup()
    os.close(fd)
    client.publish(AVAIL_TOPIC, "offline", qos=1, retain=True)
    time.sleep(0.5)
    client.loop_stop()
    client.disconnect()
    log.info("Arrêt propre. Au revoir !")


if __name__ == "__main__":
    main()
