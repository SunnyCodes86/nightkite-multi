# Bedienungsanleitung für die NightKite Multi LED-Drachenbeleuchtung (v3.0)

Willkommen bei deiner **NightKite Multi LED-Drachenbeleuchtung**!  
Dieses System bringt deinen Lenkdrachen im Dunkeln zum Leuchten, indem es dynamische LED-Effekte erzeugt, die auf die Bewegungen und Beschleunigungen deines Drachens reagieren.  
Die Farbe der LEDs wird überwiegend durch den aktuellen Winkel (Yaw) des Drachens bestimmt, während Geschwindigkeit, Helligkeit oder Fade je nach Muster von der Bewegungsintensität abhängen.  

Diese Anleitung hilft dir dabei, deine LED-Beleuchtung optimal zu nutzen.

---

## Inhaltsverzeichnis

1. [Überblick und Funktionen](#1-überblick-und-funktionen)  
   1. [Animations-Muster im Detail](#11-animations-muster-im-detail)  
2. [Komponenten](#2-komponenten)  
3. [Inbetriebnahme und Bedienung](#3-inbetriebnahme-und-bedienung)  
   1. [Controller einschalten und Initialisierung](#31-controller-einschalten-und-initialisierung)  
   2. [Funktionen des rechten Buttons](#32-funktionen-des-rechten-buttons)  
   3. [Ausschalten](#33-ausschalten)  
   4. [USB-CLI (Konfiguration)](#34-usb-cli-konfiguration)  
4. [Stromversorgung und Aufladen](#4-stromversorgung-und-aufladen)  
5. [Wichtige Hinweise](#5-wichtige-hinweise)

---

## 1. Überblick und Funktionen

Die NightKite Multi Beleuchtung bietet dir folgende Kernfunktionen:

- **Bewegungsreaktive Beleuchtung:**  
  Die LEDs passen Farbe und Animationsdynamik an die Flugbewegungen deines Drachens an  
  *(Winkel → Farbe, Bewegung → Tempo/Fade/Helligkeit – je nach Muster)*.
- **Helligkeitsstufen:**  
  Sechs Helligkeitsstufen (95 → 255 in Schritten von 32), beeinflussen auch die Akkulaufzeit.
- **Animations-Muster:**  
  27 vordefinierte Animations-Muster, davon fünf experimentelle V2-Audio-Sync-Patterns (ebenfalls Einfluss auf die Laufzeit).
- **Autoplay:**  
  Optionales automatisches Durchschalten durch aktivierte Pattern mit einstellbarem globalem Intervall.
- **Ladestandsanzeige:**  
  Zeigt den aktuellen Akkustand direkt über das LED-Band an.
- **Speicherfunktion:**  
  Muster, Helligkeit, Strip-Länge, Motion-Smoothing, Sensor-Ranges, Boot-Kalibrierung und MPU-Offsets werden persistent gespeichert.
- **USB-CLI:**  
  USB-Serielles Interface für Konfiguration, Diagnose, Timing und Kalibrierung.

---

### 1.1. Animations-Muster im Detail

Die aktuelle NightKite-Multi-Firmware verfügt über 27 vordefinierte Animationsmuster, die per Doppelklick gewechselt werden.
Nach dem Einschalten (und der Kalibrierung) startet der Controller mit dem zuletzt verwendeten Muster und der zuletzt gewählten Helligkeit.  
Beim ersten Start: **Pattern-ID 1**, **Helligkeit 95**.

1. **Regenbogen-Muster:** Sanfter, kontinuierlicher Rainbow-Durchlauf über das gesamte Band. Nicht bewegungsreaktiv.  
2. **Voller String, Winkel-Farbe:** Gesamter LED-String leuchtet in einer Farbe entsprechend dem aktuellen Winkel (Yaw).  
3. **Voller String, Winkel-Farbe & Bewegungs-Helligkeit:** Wie (2), aber Helligkeit folgt der Bewegungsintensität.  
4. **LED-Lauflicht (fix) mit Fadeout, Winkel-Farbe:** Ein einzelner Punkt wandert mit fester Geschwindigkeit; Farbe = Winkel, sanfter Fadeout.  
5. **LED-Lauflicht (reaktiv) mit Fadeout-Speed, Winkel-Farbe:** Wie (4), aber Tempo und Fade-Geschwindigkeit reagieren auf Bewegung.  
6. **LED-Lauflicht zweifarbig (reaktiv), Winkel-Farbe:** Wie (5), jedoch mit zweifarbigem Schweif; Tempo & Fade bewegungsabhängig.  
7. **Heartbeat (Winkel-Farbe):** Pulsierende „Herzschlag“-Animation über das Band; Farbe = Winkel.  
8. **Ping-Pong (bouncing) mit reaktivem Fade:** Punkt läuft vor / zurück; Fadeout hängt von Bewegung ab.  
9. **Comet-Swarm (4 Kometen):** Vier Kometen jagen über die aktuell konfigurierte Gesamtlänge (beide Hälften gespiegelt). Farbe = Winkel.  
10. **Breath / Storm (adaptiv):**  
    - Ruhig: weiches „Atmen“ (Helligkeits-Sinus).  
    - Unruhig: „Sturm“ mit Funken, deren Anzahl mit der Bewegung zunimmt.  
11. **Jerk-Wave (Ruck-Welle):** Bewegung löst Welle von der Mitte zu den Enden aus; Farbe = Winkel.  
12. **Yaw-Spinner (mit Richtungs-Gedächtnis):** Lichtkopf mit Blur-Schweif folgt der Drehrate (Yaw-Rate) mit Totzone. Farbe = Winkel.  
13. **Yaw-Spinner Kreis:** Variante von (12), bei der der Lichtkopf optisch ringförmig über beide Strips umläuft. Die Richtung folgt weiter der Yaw-Rate; Farbe = Winkel.  
14. **LED-Lauflicht zweifarbig (reaktiv, invertiert):** Wie (6), aber Laufrichtung invertiert.
15. **Palette-Beat Motion:** Pulsierendes Palettenmuster nach dem Vorbild von FastLED PaletteBeat. Der Yaw-Winkel bestimmt die Grundfarbfamilie, die Bewegungsintensität verändert Pulsrate und Farbabstand, und Yaw-Bewegung schiebt die optische Flussrichtung an.
16. **Pacifica Kite:** Mehrlagige, ozeanartige Wellen mit weichen Whitecaps. Bewegung erhöht die Wellenenergie; Yaw verschiebt die kühle Farbwelt.
17. **Twinkle Motion:** Sanfte Sternen-/Funken-Twinkles auf dunkler Grundfläche. Mehr Bewegung erhöht Dichte und Helligkeit.
18. **Fire Jet:** Feuerartige Energie, die sich aus der Mitte aufbaut und über das Band verteilt. Bewegung bestimmt die Intensität der Funken.
19. **Noise Ring:** Fließendes, Noise-basiertes Ringmuster mit bewegungs- und yawgesteuerter Farbdynamik.
20. **Pride Yaw:** Weiche, satte Farbströme mit Yaw-abhängigem Farbversatz und bewegungsabhängigem Tempo.
21. **Confetti Jerk:** Ruhiger Grundschein mit hellen Konfetti-Ausbrüchen bei harten Bewegungsimpulsen.
22. **Center Ripple:** Klar erkennbare Wellen laufen nach Bewegungsimpulsen von der Mitte nach außen.
23. **Audio Pulse Angle Color:** Synchroner Ganzband-Puls; Energy und Bass steuern Helligkeit und Flash, Yaw/Pitch die lokale Farbe.
24. **Audio Spectrum Ribbon:** Ein phasensynchrones Band kombiniert Bass-Grundglühen, Mittenwelle und Höhenakzente.
25. **Audio Beat Ripples:** Beat- und Bass-gesteuerte Ringe laufen synchron von der Strip-Mitte nach außen.
26. **Audio Band Comets:** Zwei breite, gegenläufige Bass-/Mitten-Kometen laufen synchron; Höhen setzen dezente Akzente.
27. **Audio Beat Mosaic:** Drei bis fünf große, gespiegelte Farbzonen reagieren weich auf Beat, Energie und Frequenzbänder.

Die Patterns 23 bis 27 benötigen für echte Audioreaktion V2-Audio-Beacons. Bei V1 oder fehlenden V2-Daten laufen sie mit einem weichen, sichtbaren Fallback weiter.

Bei Sync-Verlust läuft `continue_local` mit dem letzten Pattern weiter, `fallback_autoplay` wechselt zu lokalem Autoplay und `warning_only` zeigt eine rote Loss-Warnung in der Statusanzeige. `get section=sync` trennt das Sync-Ziel `sync_pattern` vom tatsächlich gerenderten `local_pattern`.

**Audio-Sync-Hardwaretest:** Controller als Follower in dieselbe Sync-Gruppe wie den Cardputer setzen, bei alten Pattern-Masken `enable_pattern 23,24,25,26,27` ausführen und am Cardputer `V2 Mic Full` starten. Mit `NK4 seq=20 cmd=audio_sync_status` und `NK4 seq=10 cmd=get section=sync` müssen unter anderem `audio_valid=1`, `last_beacon_version=2`, steigendes `scan_decode_v2`, `sync_locked=1` und `scan_crc_fail=0` sichtbar sein.

---

## 2. Komponenten

Die NightKite Multi Beleuchtung besteht aus:

- **Mikrocontroller:** Pimoroni Pico LiPo mit integriertem Akku-Management (USB-C)  
- **Akku:** 500 mAh LiPo-Akku, direkt am Mikrocontroller  
- **Sensor:** MPU6050 (Gyroskop / Beschleunigungsmesser) für Lage (Yaw / Pitch / Roll) und Bewegung  
- **LED-Strips:** Zwei symmetrische Stränge mit gleicher Länge (konfigurierbar per CLI, 10 bis 35 Pixel pro Strang), WS281x / „Fairy-String“, GRB-Reihenfolge

---

## 3. Inbetriebnahme und Bedienung

### 3.1. Controller einschalten und Initialisierung

Der Controller besitzt zwei Tasten: **links** und **rechts**.

1. **Einschalten:** Linken Button drücken.  
2. **Initialisierung:** Nach dem Einschalten wird die gespeicherte Konfiguration geladen. Standardmäßig erfolgt anschließend eine kurze Quick-Kalibrierung des MPU6050.  
   Halte den Controller ruhig, um saubere Sensor-Offsets zu ermitteln.  
3. **Bereitschaft:** Nach Abschluss der Kalibrierung schalten sich die LEDs ein.  
   Das System startet mit zuletzt verwendetem Muster und Helligkeit  
   *(bei Erstbetrieb: Pattern-ID 1, Helligkeit 95)*.

---

### 3.2. Funktionen des rechten Buttons

Der rechte Button ist ein **Multifunktions-Button**:

- **Muster / Animation wechseln (Doppelklick):**  
  → Zyklischer Wechsel zum nächsten aktivierten Muster.
- **Autoplay umschalten (Doppelklick in der Akkuanzeige):**  
  → Schaltet Autoplay global ein oder aus.
- **Helligkeitsstufe ändern (kurz drücken):**  
  → Nur während die Akkuanzeige aktiv ist.  
    Nächste der 6 Helligkeitsstufen: 95 → 127 → 159 → 191 → 223 → 255 → 95.
- **Akkuladestand anzeigen (gedrückt halten):**  
  → Zeigt den Akkustand auf dem Haupt-Strip an.  
    Auf dem zweiten Strip blinkt eine blaue LED-Marke; zusätzlich zeigen gelbe LEDs die aktuelle Helligkeitsstufe (6 Stufen) an, und zwei weitere Status-LEDs dahinter zeigen den Autoplay-Status (`grün/grün` = an, `rot/rot` = aus).  
    Die Rückkehr zum zuletzt aktiven Muster erfolgt 5 Sekunden nach der letzten Interaktion in der Akkuanzeige.

**Skala der Akkuanzeige (prozentbasiert):**

| Anzeige | Ladezustand | Farbe |
|----------|-------------|--------|
| 5 LEDs  | ≥ 80% | Blau |
| 4 LEDs  | ≥ 60% | Grün |
| 3 LEDs  | ≥ 40% | Grün |
| 2 LEDs  | ≥ 20% | Gelb |
| 1 LED   | ≥ 8% | Gelb |
| 1 LED blinkend | < 8% | Rot |

---

### 3.3. Ausschalten

- **Ausschalten:** Linken Button erneut drücken.

---

### 3.4. USB-CLI (Konfiguration)

Sobald eine aktive serielle USB-Verbindung besteht, ist die CLI verfügbar.

- Prompt: `nk>`
- Hilfe: `help`
- Aktuelle Werte: `show`
- Einzelwert lesen: `get pattern`, `get brightness`, `get strip_length`, `get smoothing`, `get accel_range`, `get gyro_range`, `get boot_calibration`, `get enabled_patterns`, `get inverted_patterns`, `get autoplay`, `get autoplay_interval`
- Einzelwert setzen:
  - `set pattern <1..27>`
  - `set brightness <95|127|159|191|223|255>`
  - `set strip_length <10..35>`
  - `set smoothing <1..512>`
  - `set accel_range <2|4|8|16>`
  - `set gyro_range <250|500|1000|2000>`
  - `set boot_calibration <off|quick>`
  - `set autoplay <on|off>`
  - `set autoplay_interval <1..300>`
- Pattern-Auswahl:
  - `patterns`
  - `enable_pattern <1..27[,id...]>`
  - `disable_pattern <1..27[,id...]>`
  - `invert_pattern <1..27[,id...]>`
  - `normal_pattern <1..27[,id...]>`
- Diagnose:
  - `battery`
  - `sensor`
  - `timing`
  - `timing reset`
  - `offsets`
- Kalibrierung:
  - `calibrate quick`
  - `calibrate precise`
- Speichern/Laden:
  - `save` (sofort in EEPROM schreiben)
  - `load` (aus EEPROM laden)
  - `defaults` (Standardwerte laden, noch nicht speichern)
  - `reboot` / `restart`

Hinweise:
- Daten-Kommandos antworten konsistent mit `OK ...` oder `ERR ...`. Beispiel: `OK pattern=1`.
- `show` liefert alle relevanten Konfigurationswerte als kompakte `key=value`-Zeile.
- `show` enthält auch den aktuellen Autoplay-Status und das Autoplay-Intervall.
- `patterns` zeigt alle Pattern mit Status `on` oder `off`.
- `strip_length` gilt immer für beide Strips gleichzeitig (symmetrisch).
- `set pattern` schaltet das aktive Muster sofort um.
- `set pattern` darf auch deaktivierte Pattern direkt anwählen.
- Ein manueller Patternwechsel lässt Autoplay aktiv, setzt aber den Autoplay-Timer zurück.
- `set brightness` wirkt sofort.
- `set strip_length` wirkt sofort auf beide Strips.
- `enable_pattern` und `disable_pattern` steuern, welche Pattern per Doppelklick durchgeschaltet werden.
- Beide Befehle akzeptieren auch mehrere Pattern gleichzeitig als kommagetrennte Liste, z. B. `disable_pattern 3,5,7`.
- `invert_pattern` und `normal_pattern` steuern die Laufrichtung pro Pattern und speichern diese Einstellung persistent.
- Auch diese beiden Befehle akzeptieren mehrere Pattern gleichzeitig als kommagetrennte Liste, z. B. `invert_pattern 4,12,13`.
- Nicht jedes Pattern besitzt eine sichtbare Laufrichtung. Bei unterstützten Patterns kehrt die Option die Bewegungsrichtung um.
- Es muss immer mindestens ein Pattern aktiv bleiben.
- Autoplay verwendet nur aktivierte Pattern.
- Autoplay kann persistent gespeichert werden und startet nach dem Booten automatisch, wenn es gespeichert eingeschaltet war.
- `smoothing`, `accel_range`, `gyro_range` und `boot_calibration` greifen erst nach einem Neustart. Die CLI kennzeichnet das in der Antwort mit `(applies after reboot)`.
- `timing` zeigt `FastLED`-FPS, `loop`-/`work`-Zeiten, Frame-Budget und Sample-Anzahl in Mikrosekunden.
- `battery` zeigt zusätzlich `battery_percent` und `battery_state`.
- `timing reset` setzt die Timing-Statistik (`avg`, `max`, `samples`) zurück, damit einzelne Pattern oder Änderungen direkt vergleichbar sind.
- `offsets` zeigt die aktuell verwendeten MPU-Offsets.
- `calibrate quick` ist die schnelle Alltags-Kalibrierung und speichert die gefundenen Offsets.
- `calibrate precise` nutzt den deutlich langsameren `IMU_Zero`-basierten Präzisionspfad und speichert die gefundenen Offsets.
- `calibrate quick` und `calibrate precise` starten mit `OK calibrate_started=1 ...` und enden mit `OK calibrate_finished=1 ...` plus einer finalen Offset-Zeile.
- `defaults` lädt die Standardwerte nur in den Arbeitsspeicher. Für dauerhafte Speicherung ist anschließend `save` nötig.
- Beim nächsten Neustart werden die gespeicherten Werte automatisch geladen.

## 4. Stromversorgung und Aufladen

Das System wird durch einen integrierten **500 mAh LiPo-Akku** versorgt.  
Der Pimoroni Pico LiPo besitzt ein intelligentes Lade-/Entlademanagement.

- **Aufladen:** Über USB-C an PC oder Netzteil anschließen.  
- **Lade-Anzeige:** Während des Ladens zeigt das Band den Füllstand als Balken;  
  eine rote LED blinkt während des aktiven Ladevorgangs.  
- **USB-CLI aktiv:** Bei aktiver serieller Verbindung wird die Ladeanzeige unterdrückt, damit die CLI störungsfrei genutzt werden kann.  
- **Automatischer Rückwechsel:** Nach dem Trennen vom USB kehrt das System zum letzten Muster zurück.  
- **Laufzeit:** Je nach Muster und Helligkeit ca. 1 – 2,5 Stunden.

**Akkuwarnungen und Schutzverhalten:**

- Der Ladezustand wird über eine NightKite-spezifische LiPo-SoC-Kurve berechnet und geglättet.
- Unter 3.40 V für ca. 15 s meldet die Diagnose `LOW_WARNING`.
- Unter 3.30 V für ca. 10 s meldet die Diagnose `CRITICAL`; die Helligkeit wird temporär auf `MIN_BRIGHTNESS` begrenzt, ohne den gespeicherten Helligkeitswert zu ändern.
- Unter 3.20 V für ca. 15 s geht der Controller in `SOFT_CUTOFF`: LEDs schwarz, Konfiguration defensiv gespeichert, keine Pattern-/Wireless-Aktivität mehr.
- Unter ca. 3.08 V wird sofort `EMERGENCY_CUTOFF` aktiv.
- USB-Betrieb und Laden lösen keinen Low-Battery-Cutoff aus.

**Speicherfunktion (automatisch):**

- Muster und Helligkeit werden etwa alle 5 Minuten geprüft und bei Änderung gespeichert.  
- Strip-Länge wird ebenfalls geprüft und bei Änderung gespeichert.  
- Motion-Smoothing, Sensor-Ranges, Boot-Kalibrierung und MPU-Offsets werden ebenfalls überwacht und bei Änderung gespeichert.
- Beim nächsten Start werden diese Werte automatisch wiederhergestellt.

---

## 5. Wichtige Hinweise

- **Ruhige Initialisierung:** Nach dem Einschalten ruhig halten, bis LEDs aktiv sind.  
- **Präzise Kalibrierung:** Für `calibrate precise` den Controller auf eine absolut ruhige, waagerechte Fläche legen und wenn möglich vorher 5-10 Minuten thermisch stabilisieren.  
- **Wetterbedingungen:** Elektronik vor Feuchtigkeit schützen (kein Regen / Nebel).  
- **Sicherheit:** Bei Nachtflügen ausreichenden Abstand halten und freie Fläche wählen.  
- **Akku-Pflege:** Nur geeignete USB-Netzteile verwenden.  
  Akku nicht unbeaufsichtigt laden, nicht kurzschließen, nicht beschädigen oder tiefentladen.

---

**Kurzüberblick für den schnellen Einstieg:**  
`Einschalten → ruhig halten (Kalibrierung) → Doppelklick = Muster → Langdruck = Akkuanzeige → Kurztipp (in Akkuanzeige) = Helligkeit → Doppelklick (in Akkuanzeige) = Autoplay`
