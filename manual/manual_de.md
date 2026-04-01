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
  22 vordefinierte Animations-Muster zur Anpassung des Erscheinungsbilds (ebenfalls Einfluss auf die Laufzeit).
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

Die NightKite Multi (v3.0) verfügt über 22 vordefinierte Animationsmuster, die per Doppelklick gewechselt werden.  
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
  → Zyklischer Wechsel zum nächsten der 22 Muster.
- **Autoplay umschalten (Doppelklick in der Akkuanzeige):**  
  → Schaltet Autoplay global ein oder aus.
- **Helligkeitsstufe ändern (kurz drücken):**  
  → Nur während die Akkuanzeige aktiv ist.  
    Nächste der 6 Helligkeitsstufen: 95 → 127 → 159 → 191 → 223 → 255 → 95.
- **Akkuladestand anzeigen (gedrückt halten):**  
  → Zeigt den Akkustand auf dem Haupt-Strip an.  
    Auf dem zweiten Strip blinkt eine blaue LED-Marke; zusätzlich zeigen gelbe LEDs die aktuelle Helligkeitsstufe (6 Stufen) an, und zwei weitere Status-LEDs dahinter zeigen den Autoplay-Status (`grün/grün` = an, `rot/rot` = aus).  
    Die Rückkehr zum zuletzt aktiven Muster erfolgt 5 Sekunden nach der letzten Interaktion in der Akkuanzeige.

**Skala der Akkuanzeige (Spannungsbasiert):**

| Anzeige | Spannung | Farbe |
|----------|-----------|--------|
| 5 LEDs  | ≥ 4.05 V | Blau |
| 4 LEDs  | 4.05 – 3.92 V | Grün |
| 3 LEDs  | 3.92 – 3.80 V | Grün |
| 2 LEDs  | 3.80 – 3.68 V | Gelb |
| 1 LED   | 3.68 – 3.55 V | Gelb |
| 1 LED   | 3.55 – 3.40 V | Rot |
| Keine LED | < 3.40 V | (sehr leer) |

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
  - `set pattern <1..22>`
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
  - `enable_pattern <1..22[,id...]>`
  - `disable_pattern <1..22[,id...]>`
  - `invert_pattern <1..22[,id...]>`
  - `normal_pattern <1..22[,id...]>`
- Diagnose:
  - `battery`
  - `sensor`
  - `timing`
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
- `timing` zeigt `FastLED`-FPS sowie `loop`-/`work`-Zeiten in Mikrosekunden.
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
- **Voll geladen:** Die Ladeanzeige springt erst nahe der vollen Zellspannung auf 5 blaue LEDs (Firmware-Schwelle: ≥ 4.20 V, also ungefähr 4.2 V); dann geht die rote Lade-LED aus.  
- **Automatischer Rückwechsel:** Nach dem Trennen vom USB kehrt das System zum letzten Muster zurück.  
- **Laufzeit:** Je nach Muster und Helligkeit ca. 1 – 2,5 Stunden.

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
