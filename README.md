# Domusa Modbus to MQTT Bridge

Ce projet permet de connecter une chaudière à granulés **Domusa Bioclass HM** à un système domotique (Home Assistant, Node-RED, etc.) via un **Raspberry Pi**. 

Il combine le décodage du flux Modbus propriétaire de la chaudière avec l'ajout de capteurs GPIO additionnels pour un monitoring complet.

## Table des matières
- [Fonctionnalités](#fonctionnalités)
- [Architecture](#architecture)
- [Installation](#installation)
- [Matériel & Câblage](#matériel--câblage)
- [Protocole Modbus](#protocole-modbus)
- [Registres & Données](#registres--données)
- [Licence](#licence)

---

## Fonctionnalités

- **Lecture Modbus RTU Passive** : Décode le flux broadcasté par la chaudière (Maître Modbus FC16).
- **Pont MQTT** : Convertit les registres en topics MQTT structurés.
- **Home Assistant Discovery** : Déclaration automatique de plus de 90 entités (capteurs, binaires, diagnostics).
- **Capteurs GPIO additionnels** :
    - Niveaux de granulés (Ultrasons HC-SR04).
    - Température extérieure/ballon précise (1-Wire DS18B20).
    - Proximité (TCRT5000).
    - Alarme externe (Contact sec).
- **Multithreading** : Gestion temps-réel du flux série sans blocage par les capteurs lents.

---

## Architecture

```text
┌────────────────────────┐        série (9600 bauds)         ┌─────────────────────┐
│  Chaudière Domusa      │ ──────────────────────────────►   │  Raspberry Pi       │
│  (Maître Modbus FC=16) │   /dev/serial0                    │  (Python Script)    │
│  Slave ID=3, 70 regs   │                                   │                     │
└────────────────────────┘          Capteurs GPIO            │  ┌───────────────┐  │
                                   ┌───────────────────────► │  Modbus Thread   │  │
┌────────────────────────┐         │                         │  GPIO Thread     │  │
│  Sondes Additionnelles │ ────────┘                         └─────────┬────────┘  │
│  (Ultrasons, 1-Wire...)│                                             │           │
└────────────────────────┘                                             ▼           │
                                                            ┌──────────────────────┐
                                                            │  MQTT Broker         │
                                                            └──────────────────────┘
```

---

## Installation

### 1. Dépendances
```bash
sudo apt update
sudo apt install python3-paho-mqtt python3-rpi.gpio
```

### 2. Configuration du port série
Désactiver la console série via `raspi-config` (Interface Options -> Serial Port -> Login shell: No, Hardware: Yes) pour libérer `/dev/serial0`.

### 3. Permissions
```bash
sudo usermod -aG dialout $USER
```

### 4. Sondes 1-Wire (DS18B20)
Pour lire la sonde 1-Wire, activez le support dans `/boot/config.txt` (ou `/boot/firmware/config.txt` sur Pi 5) :
```bash
echo "dtoverlay=w1-gpio" | sudo tee -a /boot/config.txt
```
Redémarrez ensuite le Raspberry Pi.

---

## Matériel & Câblage

### Connexion Série (Chaudière → RPi)
La chaudière transmet ses données sur les pins de diagnostic de sa carte mère (JCOM) à **9600 bauds (8-N-1)**.
- **RX (Chaudière)** → **TX (RPi GPIO 14)**
- **TX (Chaudière)** → **RX (RPi GPIO 15)**
- **GND** → **GND**

### Mapping GPIO (BCM)

| Capteur | GPIO (BCM) | Fonction | Type |
| :--- | :--- | :--- | :--- |
| **Distance 1** | 17 (Trig) / 27 (Echo) | Niveau pellets | Ultrasons |
| **Distance 2** | 23 (Trig) / 22 (Echo) | Niveau pellets (bis) | Ultrasons |
| **Temp. ESC** | 04 | 1-Wire Data | DS18B20 |
| **Proximité** | 18 | Signal Numérique | TCRT5000 |
| **Alarme** | 25 | Signal Numérique | Contact Sec |

---

## Protocole Modbus

Le régulateur Domusa agit comme un **Maître Modbus** et publie (*broadcast*) cycliquement ses données à l'adresse **Slave ID 3**, via le **Function Code 16** (Write Multiple Registers).
Le script écoute passivement cette trame de **70 registres consécutifs** (`R00` à `R69`).

---

## Registres & Données

### Valeurs Analogiques

| Registre | Signification | Conversion |
| :--- | :--- | :--- |
| `R00` | Température Chaudière | /10 (°C) |
| `R02` | Consigne Chaudière | /10 (°C) |
| `R08` | Pression d'Eau | /100 (Bar) |
| `R09` | Puissance Ventilateur | % |
| `R11` | Consigne ECS | /10 (°C) |
| `R12` | Capteur de Flamme | % |
| `R40` | Vitesse Extracteur | RPM |

### Phases de fonctionnement (`R16`)
Le script traduit automatiquement les codes numériques en libellés clairs :
`standby`, `allumage`, `detection_flamme`, `chauffe_normale`, `nettoyage`, `post_ventilation`, etc.

### Relais Électriques (`R32` - Bitmask)
Le registre `R32` permet de connaître l'état des sorties 230V. 

*Note : Ces bits peuvent correspondre à des organes différents selon les modèles (Pompe, Vis, Allumeur, etc.).*
Par précaution d'exactitude, ils sont nommés génériquement : **`Relais b0`** à **`Relais b11`**.

---

---

## Registres & Données (Cartographie Modbus)

### Sondes et Valeurs Analogiques
*Note : Les températures sont divisées par 10 (°C). La pression est divisée par 100 (bar).*

| Registre | Signification | Conversion |
| :--- | :--- | :--- |
| **R00** | Température Chaudière (réelle) | ÷10 |
| **R01** | Température Retour (option HotStream) | ÷10 |
| **R02** | Consigne Température Chaudière | ÷10 |
| **R08** | Pression d'Eau du Circuit | ÷100 |
| **R09** | Puissance Ventilateur Combustion | % |
| **R11** | Consigne Température ECS | ÷10 |
| **R12** | Capteur de Flamme / Lumex | % (0-100) |
| **R40** | Vitesse Extracteur de Fumée | RPM |
| **R46** | Consigne Ballon Tampon | ÷10 |
| **R60** | Température Réelle Ballon ECS | ÷10 |

### Actionneurs et Capteurs TOR (Tout ou Rien)

| Registre | Signification | Valeurs |
| :--- | :--- | :--- |
| **R14** | Moteur Vis sans Fin (Pellets) | 0=Arrêt, 1=Marche |
| **R19** | Bouton de demande chaleur (Écran) | 0=OFF, 1=ON |
| **R27/R28**| Fin de Course Raclette A/B | Impulsion 1 |
| **R35** | État Nettoyage Mécanique | 0=Arrêt, 1=Nettoyage |
| **R37** | Demande de Chaleur Circuit(s) | 0=OFF, >0=Demande active |
| **R53** | Relais Alarme / Défaut Physique | 0=OK, 1=Alarme |

### Compteurs et Jauges (Suivi Maintenance)

| Registre | Signification | Unité |
| :--- | :--- | :--- |
| **R20/R21**| Niveau Bac à Cendres A / B | Incrémental |
| **R42** | Heures Totales Brûleur | Heures |
| **R43** | Nombre Total d'Allumages | Count |
| **R44** | Consommation Totale Pellets (Est.) | kg |
| **R49** | Compteur d'Erreurs Cumulées | Count |

---

## États de la machine (`R16`)

| Code | Phase | Signification |
| :--- | :--- | :--- |
| **0** | `eteint` | Chaudière éteinte |
| **1** | `nettoyage_initial` | Préchauffage et nettoyage |
| **3/4/5** | `allumage_1/2/3` | Séquence d'allumage pellets |
| **9** | `detection_flamme` | Flamme confirmée par Lumex |
| **10/11** | `travail / chauffe` | Fonctionnement normal |
| **12** | `modulation` | Modulation de puissance |
| **14** | `nettoyage` | Nettoyage périodique |
| **16** | `standby` | Veille (Consigne atteinte) |
| **17/18** | `refroidissement` | Post-ventilation / Nettoyage final |
| **19** | `blocage` | Arrêt sécurité |

---

## Liste Complète des Codes d'Erreur (`R22`)

| Code | Type | Libellé |
| :--- | :--- | :--- |
| **E-00** | - | Pas d'erreur (Système OK) |
| **E-01/02**| Sonde | Sonde chaudière Sc ouverte / court-circuitée |
| **E-03/04**| Sonde | Sonde ECS Sa ouverte / court-circuitée |
| **E-05** | Alerte | Surchauffe de l'arrivée de combustible (Te) |
| **E-06** | Allumage | Erreur à l'allumage |
| **E-07/08**| Nettoyage| Erreur début/pendant nettoyage cendres |
| **E-09** | Méca | Erreur fin de course FCp nettoyage cendres |
| **E-10/11**| Sécurité | Surchauffe chaudière / Thermostat sécurité (Ts) |
| **E-12** | Méca | Extraction du brûleur FCq |
| **E-13/14**| Air | Dépression air insuffisante ou chute de dépression |
| **E-15** | Air | Dépression insuffisante au pré-nettoyage |
| **E-18/19**| Eau | Capteur pression eau HS ou pression trop basse |
| **E-20** | Valve | Anomalie soupape de sécurité |
| **E-21** | Air | Erreur capteur pression air |
| **E-22/23**| Air | Dépression/Surpression excessive foyer |
| **E-25** | Diagnos | Calibration défectueuse |
| **E-26/27**| CVS | Erreur comm. / Blocage système aspiration CVS |
| **E-29** | Pellet | Capteur de niveau combustible |
| **E-30-35**| Sonde | Erreurs sondes Sr1, Sr2 ou Sext (Extérieure) |
| **E-36** | Config | Erreur position DIP-switch |
| **E-37/42**| Comm | Erreur communication Kit hydraulique BIO |
| **E-38** | Air | Dépression insuffisante dans le foyer |
| **E-39-41**| Venti | Erreur ou chute de vitesse ventilateur |
| **E-43** | Entretien | Cendrier plein |
| **E-44** | Entretien | Maintenance de la chaudière requise |
| **E-45/46**| Sonde | Sonde ballon tampon Sbt ouverte/court-circuitée |
| **E-47** | Pellet | Erreur détecteur niveau combustible |
| **E-49** | Pellet | **Trémie vide** (Manque de granulés) |
| **E-50** | Conflit | Conflit entre CVS et capteur niveau |

---

## Paramètres de Configuration (Menu Technicien - P.xxx)

Accessibles via le script à partir du registre 100 (P.01 = 101, etc.) en lecture/écriture.

| Paramètre | Description |
| :--- | :--- |
| **P.05** | Temps de pré-charge pellets allumage |
| **P.06** | Calibration vis sans fin (grammes/cycle) |
| **P.15** | Temps post-circulation pompe chauffage |
| **P.19** | Seuil pression eau minimale alarme |
| **P.22** | Temps de cycle aspiration CVS |

---

## Licence

Ce projet est publié sous **Licence MIT**. 
Vous êtes libre de copier, modifier et distribuer ce logiciel, à condition de conserver les mentions de copyright.

---
*Projet réalisé par reverse-engineering exhaustif (trame FC16 Master).*

