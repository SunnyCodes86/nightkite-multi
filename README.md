# Lenkdrachen LED Beleuchtung mit Bewegungsreaktion (Kite LED Lighting with Motion Response)

## Deutsch *(Version 2.1)*

Dieses Projekt realisiert eine dynamische LED-Beleuchtung für Lenkdrachen, die auf deren Bewegungen und Beschleunigungen reagiert. Die Farbe der LEDs wird durch den aktuellen Winkel des Drachens bestimmt, während die Geschwindigkeit der Animationsabläufe von der Bewegungsgeschwindigkeit des Drachens abhängt.

[Bedienungsanleitung](manual/manual_de.md)

### Features

* Helligkeit in 6 Stufen (95 → 255 in Schritten von 32) einstellbar – beeinflusst die Akkulaufzeit
* 13 reaktionsfreudige Patterns/Animationen, teilweise bewegungsabhängig
* Ladestandsanzeige über das LED-Band inkl. Ladefortschritt während USB-Verbindung
* Persistente Konfiguration in EEPROM: Pattern, Helligkeit, Strip-Länge, Motion-Smoothing, Sensor-Range, Boot-Kalibrierung und MPU-Offsets
* USB-CLI für Status, Diagnose, Timing und Kalibrierung

### Verwendete Technologien und Komponenten

* **Mikrocontroller:** [Pimoroni Pico Lipo](https://shop.pimoroni.com) **(mit integriertem Akku-Management)**
* **Akku:** 500mAh LiPo Akku
* **Gyroskop/Beschleunigungsmesser:** [MPU6050 Modul](https://www.reichelt.de/de/de/shop/produkt/entwicklerboards_-_beschleunigung_gyroskop_3-achsen_mpu-6050-253987)
* **LED-Strip:** Zwei WS2812B Fairyight LED-Stränge mit je 25 Pixeln (50 Pixel gesamt)
* **Programmiersprache:** C++
* **Entwicklungsumgebung (optional):** [PlatformIO](https://platformio.org/) (für eigene Anpassungen)
* **Gewicht:** ca. 60 Gramm (Controller mit Gehäuse ca. 30 Gramm, LED-Strip ca. 30 Gramm)

### Hardware

![Bild des Controllers](images/controller.jpg)

![Bild der Platine](images/platine.jpg)

#### 3D-Druck Gehäuse

Für dieses Projekt steht ein Gehäuse zur Verfügung, das mit einem 3D-Drucker gefertigt werden kann. Die Dateien für den 3D-Druck (im 3MF- und STEP-Format) sowie eine Vorschau des Gehäuses befinden sich im Verzeichnis `case/` dieses Repositorys.

![Vorschau des Gehäuses](case/nightkite-multi.png)

#### Verdrahtung

Die folgende Tabelle zeigt die Verbindungen zwischen dem Pimoroni Pico Lipo, dem LED-Strip und dem MPU6050:

| Komponente          | Pin am Mikrocontroller |
| ------------------ | ---------------------- |
| LED Strip +        | VS (VSYS)              |
| LED Strip -        | GND                    |
| LED Strip 1 DIN    | GP12                   |
| LED Strip 2 DIN    | GP13                   |
| MPU6050 VCC        | 3V3\_OUT               |
| MPU6050 GND        | GND                    |
| MPU6050 INT        | GP3                    |
| MPU6050 SDA        | GP4                    |
| MPU6050 SCL        | GP5                    |

### Stromversorgung und Laufzeit

Das System wird über einen integrierten 500mAh LiPo Akku betrieben. Der Pimoroni Pico Lipo verfügt über ein integriertes Akku-Management, das ein sicheres Laden und Entladen des Akkus gewährleistet.

**Aufladen:** Das Aufladen erfolgt bequem über den USB-C Anschluss am Mikrocontroller. Während des Ladevorgangs zeigt das LED-Band den Ladefortschritt als Balken, eine rote LED blinkt zur Bestätigung. Voll geladen bedeutet fünf blaue LEDs und die rote Status-LED erlischt.

**Laufzeit:** Die Akkulaufzeit beträgt, abhängig von der gewählten LED-Animation und der Helligkeit, zwischen 1 und 2,5 Stunden. Nach dem Abziehen vom USB springt das System automatisch zurück zum zuletzt aktiven Muster.

### Bedienung

Der Controller verfügt über zwei Tasten:

* **Linker Button:** Schaltet den Controller ein und aus.
* **Rechter Button:** Hat drei Funktionen:
    * **Doppelklick (2x kurz hintereinander):** Schaltet zyklisch durch die 13 Animationsmuster.
    * **Kurz drücken (Shortpress):** Schaltet durch die 6 Helligkeitsstufen (95 → 127 → 159 → 191 → 223 → 255 → 95).
    * **Gedrückt halten (Longpress):** Zeigt für 5 Sekunden den Akkuladestand an. Bis zu fünf LEDs dienen als Balkenanzeige; eine blaue Markierung blinkt während der Messung.

**Initialisierung & Speicherfunktion:** Nach dem Einschalten lädt der Controller die gespeicherte Konfiguration und führt standardmäßig eine kurze MPU6050-Quick-Kalibrierung durch. Während dieser Phase (einige Sekunden) sollte der Controller ruhig gehalten werden. Die Boot-Kalibrierung kann über die CLI deaktiviert werden. Sobald die Initialisierung abgeschlossen ist, schalten sich die LEDs ein. Die zuletzt gespeicherten Werte werden automatisch geladen.

### Einfache Installation über .UF2-Datei

Für eine schnelle Inbetriebnahme kann eine fertig kompilierte `.uf2`-Datei verwendet werden. Gehe dazu wie folgt vor:

1.  **Boot-Modus aktivieren:** Halte den **rechten Knopf** auf dem Pimoroni Pico Lipo gedrückt, während du ihn über USB mit deinem Computer verbindest. Dadurch versetzt du den Mikrocontroller in den Boot-Modus.

2.  **Laufwerk erkennen:** Dein Computer sollte nun ein neues Laufwerk mit dem Namen `RPI-RP2` oder ähnlich anzeigen.

3.  **`.uf2`-Datei kopieren:** Lade die Datei `firmware.uf2` aus dem Release-Bereich dieses Repositorys herunter und ziehe sie per Drag & Drop auf das erkannte `RPI-RP2`-Laufwerk.

4.  **Automatischer Neustart:** Sobald die Datei kopiert wurde, wird der Pico automatisch neu gestartet und die LED-Beleuchtung sollte aktiv sein.

### Einrichtung und Start (für Entwickler und eigene Anpassungen)

Wenn du die Software selbst kompilieren oder Anpassungen vornehmen möchtest, sind folgende Schritte notwendig:

1.  **Installation von PlatformIO:** Stelle sicher, dass PlatformIO auf deinem System installiert ist. Eine detaillierte Anleitung findest du auf der [PlatformIO Webseite](https://platformio.org/install).

2.  **Klonen des Repositorys:** Klone dieses GitHub-Repository auf deinen lokalen Rechner.
    ```bash
    git clone [https://github.com/SunnyCodes86/nightkite-multi.git](https://github.com/SunnyCodes86/nightkite-multi.git)
    cd nightkite-multi
    ```

3.  **Konfiguration in `platformio.ini` (optional):** Überprüfe die `platformio.ini`-Datei im Projektverzeichnis. Hier sind die Umgebungsbedingungen für den Pimoroni Pico Lipo und die benötigten Bibliotheken definiert. Passe diese Datei bei Bedarf an deine spezifischen Bedürfnisse an.

4.  **Bibliotheken installieren:** PlatformIO sollte die benötigten Bibliotheken automatisch herunterladen und installieren, wenn du das Projekt kompilierst. Stelle sicher, dass in deiner `platformio.ini` die notwendigen Bibliotheken unter `lib_deps` aufgeführt sind.

5.  **Kompilieren und Hochladen:** Verbinde deinen Pimoroni Pico Lipo über USB mit deinem Computer. Nutze dann PlatformIO, um das Projekt zu kompilieren und auf den Mikrocontroller hochzuladen. Dies kann in der PlatformIO IDE oder über die Kommandozeile erfolgen:
    ```bash
    pio run -t upload
    ```

6.  **Los geht's!** Sobald die Software hochgeladen ist (entweder über `.uf2` oder PlatformIO) und die Hardware korrekt verbunden ist, sollte die LED-Beleuchtung deines Lenkdrachens auf dessen Bewegungen reagieren.

### USB-CLI

Sobald eine aktive serielle USB-Verbindung besteht, meldet sich die CLI mit:

```text
[NightKite CLI] USB connected. Type 'help'.
nk>
```

Verfügbare Kommandos:

```text
help
show
get <pattern|brightness|strip_length|smoothing|accel_range|gyro_range|boot_calibration>
set pattern <1..13>
set brightness <95|127|159|191|223|255>
set strip_length <10..35>
set smoothing <1..512>
set accel_range <2|4|8|16>
set gyro_range <250|500|1000|2000>
set boot_calibration <off|quick>
patterns
enable_pattern <1..13[,id...]>
disable_pattern <1..13[,id...]>
battery
sensor
timing
offsets
calibrate quick
calibrate precise
save
load
defaults
reboot
restart
```

Hinweise:

* Daten-/Konfigurationskommandos antworten konsistent mit `OK ...` oder `ERR ...`, zum Beispiel `OK pattern=1`.
* `show` liefert alle relevanten Konfigurationswerte als kompakte `key=value`-Zeile.
* `patterns` zeigt alle Pattern mit Status `on` oder `off`.
* `set pattern` schaltet das aktive Muster sofort um.
* `set pattern` darf auch deaktivierte Pattern direkt anwählen.
* `set brightness` wirkt sofort.
* `set strip_length` wirkt sofort auf beide Strips.
* `enable_pattern` und `disable_pattern` steuern, welche Pattern per Doppelklick durchgeschaltet werden.
* Beide Befehle akzeptieren auch mehrere Pattern gleichzeitig als kommagetrennte Liste, z. B. `disable_pattern 3,5,7`.
* Es muss immer mindestens ein Pattern aktiv bleiben.
* `smoothing`, `accel_range`, `gyro_range` und `boot_calibration` werden persistent gespeichert, greifen aber erst nach einem Neustart.
* `battery` zeigt den ADC-Rohwert, die berechnete Spannung sowie USB-/Serial-Status.
* `sensor` zeigt MPU-/DMP-Status und die konfigurierten bzw. aktiven Sensor-Ranges.
* `timing` zeigt `FastLED`-FPS sowie Loop-/Work-Zeiten in Mikrosekunden.
* `offsets` zeigt die aktuell verwendeten MPU-Offsets.
* `calibrate quick` führt die bisherige schnelle Kalibrierung aus und speichert die Offsets.
* `calibrate precise` nutzt den langsameren `IMU_Zero`-basierten Kalibrierpfad und speichert die Offsets.
* `calibrate quick` und `calibrate precise` starten mit `OK calibrate_started=1 ...` und enden mit `OK calibrate_finished=1 ...` plus finaler Offset-Zeile.
* `defaults` lädt nur Standardwerte in den Arbeitsspeicher. Für persistente Speicherung ist danach `save` nötig.

### Hinweise zur Kalibrierung

Es gibt zwei Kalibrierpfade:

* **Quick Calibration:** Standard beim Boot, wenige Sekunden, ausreichend für den Alltag.
* **Precise Calibration:** Nur manuell per CLI, dauert deutlich länger und sollte nur bei ruhigem, waagerecht abgelegtem Gerät durchgeführt werden.

Empfehlung:

1. Gerät vor einer präzisen Kalibrierung 5-10 Minuten thermisch stabilisieren.
2. Controller absolut ruhig und waagerecht platzieren.
3. `calibrate precise` ausführen.
4. Danach optional `set boot_calibration off` und `save`, wenn künftig nur mit gespeicherten Offsets gestartet werden soll.

### Lizenz

Dieses Projekt ist unter der [MIT Lizenz](LICENSE.txt) lizenziert.

---

## English *(Version 2.1)*

This project implements dynamic LED lighting for kites that reacts to their movements and accelerations. The color of the LEDs is determined by the current angle of the kite, while the speed of the animation sequences depends on the kite's speed of motion.

[User Manual](manual/manual_en.md)

### Features

* Six brightness levels (95 → 255 in steps of 32) impact the battery runtime
* 13 motion-reactive patterns/animations with mirrored dual-strip output
* Battery level indicator on the LED strip including charging progress while on USB power
* Persistent EEPROM configuration for pattern, brightness, strip length, motion smoothing, sensor ranges, boot calibration, and MPU offsets
* USB CLI for configuration, diagnostics, timing, and calibration

### Used Technologies and Components

* **Microcontroller:** [Pimoroni Pico Lipo](https://shop.pimoroni.com) **(with integrated battery management)**
* **Battery:** 500mAh LiPo battery
* **Gyroscope/Accelerometer:** [MPU6050 Module](https://www.reichelt.de/de/en/shop/product/developer-boards-acceleration-gyroscope-3-axes-mpu-6050-253987)
* **LED Strips:** Two WS2812B Fairyight strings with 25 pixels each (50 pixels total)
* **Programming Language:** C++
* **Development Environment (optional):** [PlatformIO](https://platformio.org/) (for own adaptations)
* **Weight:** approx. 60 grams (controller with case approx. 30 grams, LED strip approx. 30 grams)

### Hardware

![Controller Image](images/controller.jpg)

![Board Image](images/platine.jpg)

#### 3D-Printed Case

An optional case for this project is available and can be manufactured with a 3D printer. The files for 3D printing (in 3MF and STEP format) as well as a preview of the case can be found in the `case/` directory of this repository.

![Case Preview](case/nightkite-multi.png)

#### Wiring

The following table shows the connections between the Pimoroni Pico Lipo, the LED strip, and the MPU6050:

| Component          | Pin on Microcontroller |
| ------------------ | ---------------------- |
| LED Strip +        | VS (VSYS)              |
| LED Strip -        | GND                    |
| LED Strip 1 DIN    | GP12                   |
| LED Strip 2 DIN    | GP13                   |
| MPU6050 VCC        | 3V3\_OUT               |
| MPU6050 GND        | GND                    |
| MPU6050 INT        | GP3                    |
| MPU6050 SDA        | GP4                    |
| MPU6050 SCL        | GP5                    |

### Power Supply and Runtime

The system is powered by an integrated 500mAh LiPo battery. The Pimoroni Pico Lipo features integrated battery management, ensuring safe charging and discharging of the battery.

**Charging:** Charging is done via the USB-C port on the microcontroller. While charging, the LED strip displays a progress bar and a red LED blinks to indicate active charging. When five blue LEDs remain and the red LED switches off, the battery is fully charged.

**Runtime:** Battery runtime is between 1 and 2.5 hours, depending on the selected LED animation and brightness. After unplugging USB power the controller automatically resumes the last active pattern.

### Operation

The controller has two buttons:

* **Left Button:** Turns the controller on and off.
* **Right Button:** Provides three functions:
    * **Double click (two quick presses):** Cycles through the 13 animation patterns.
    * **Short press:** Cycles through the six brightness levels (95 → 127 → 159 → 191 → 223 → 255 → 95).
    * **Long press:** Shows the battery level for 5 seconds. Up to five LEDs form a bar indicator while a blue marker LED blinks during the readout.

**Initialization & Persistence:** After power-on, the controller loads the stored configuration and by default performs a short MPU6050 quick calibration. Keep the controller still for a few seconds during this phase. Boot calibration can be disabled via the CLI. Once initialization is complete, the LEDs turn on and the stored configuration is restored automatically.

### Easy Installation via .UF2 File

For quick setup, a pre-compiled `.uf2` file can be used. Proceed as follows:

1.  **Activate Boot Mode:** Press and hold the **right button** on the Pimoroni Pico Lipo while connecting it to your computer via USB. This puts the microcontroller into boot mode.

2.  **Recognize Drive:** Your computer should now display a new drive named `RPI-RP2` or similar.

3.  **Copy `.uf2` File:** Download the `firmware.uf2` file from the release section of this repository and drag and drop it onto the recognized `RPI-RP2` drive.

4.  **Automatic Restart:** Once the file is copied, the Pico will restart automatically, and the LED lighting should be active.

### Setup and Start (for Developers and Own Adaptations)

If you want to compile the software yourself or make adjustments, the following steps are necessary:

1.  **Install PlatformIO:** Make sure PlatformIO is installed on your system. Detailed instructions can be found on the [PlatformIO website](https://platformio.org/install).

2.  **Clone the Repository:** Clone this GitHub repository to your local machine.
    ```bash
    git clone [https://github.com/SunnyCodes86/nightkite-multi.git](https://github.com/SunnyCodes86/nightkite-multi.git)
    cd nightkite-multi
    ```

3.  **Configuration in `platformio.ini` (optional):** Check the `platformio.ini` file in the project directory. Here, the environment settings for the Pimoroni Pico Lipo and the required libraries are defined. Adjust this file to your specific needs if necessary.

4.  **Install Libraries:** PlatformIO should automatically download and install the required libraries when you compile the project. Ensure that the necessary libraries are listed under `lib_deps` in your `platformio.ini`.

5.  **Compile and Upload:** Connect your Pimoroni Pico Lipo to your computer via USB. Then use PlatformIO to compile the project and upload it to the microcontroller. This can be done in the PlatformIO IDE or via the command line:
    ```bash
    pio run -t upload
    ```

6.  **Let's Go!** Once the software is uploaded (either via `.uf2` or PlatformIO) and the hardware is correctly connected, the LED lighting of your kite should react to its movements.

### USB CLI

As soon as an active USB serial connection exists, the CLI announces itself with:

```text
[NightKite CLI] USB connected. Type 'help'.
nk>
```

Available commands:

```text
help
show
get <pattern|brightness|strip_length|smoothing|accel_range|gyro_range|boot_calibration>
set pattern <1..13>
set brightness <95|127|159|191|223|255>
set strip_length <10..35>
set smoothing <1..512>
set accel_range <2|4|8|16>
set gyro_range <250|500|1000|2000>
set boot_calibration <off|quick>
patterns
enable_pattern <1..13[,id...]>
disable_pattern <1..13[,id...]>
battery
sensor
timing
offsets
calibrate quick
calibrate precise
save
load
defaults
reboot
restart
```

Notes:

* Data/config commands reply consistently with `OK ...` or `ERR ...`, for example `OK pattern=1`.
* `show` returns all relevant configuration values as a compact `key=value` line.
* `patterns` lists all patterns with `on` or `off` state.
* `set pattern` switches the active pattern immediately.
* `set pattern` can also select patterns that are currently disabled for button cycling.
* `set brightness` takes effect immediately.
* `set strip_length` applies immediately to both strips.
* `enable_pattern` and `disable_pattern` control which patterns are included when cycling with the button.
* Both commands also accept multiple pattern IDs as a comma-separated list, for example `disable_pattern 3,5,7`.
* At least one pattern must always remain enabled.
* `smoothing`, `accel_range`, `gyro_range`, and `boot_calibration` are stored persistently but only take effect after reboot.
* `battery` reports raw ADC value, calculated voltage, and USB/serial status.
* `sensor` reports MPU/DMP status and both configured and active sensor ranges.
* `timing` reports `FastLED` FPS plus loop/work timings in microseconds.
* `offsets` reports the currently active MPU offsets.
* `calibrate quick` runs the fast calibration path and saves the resulting offsets.
* `calibrate precise` runs the slower `IMU_Zero`-style calibration path and saves the resulting offsets.
* `calibrate quick` and `calibrate precise` start with `OK calibrate_started=1 ...` and finish with `OK calibrate_finished=1 ...` plus a final offsets line.
* `defaults` only loads factory defaults into working memory. Run `save` afterwards if you want to keep them permanently.

### Calibration Notes

Two calibration modes are available:

* **Quick Calibration:** Default during boot, takes only a few seconds, suitable for normal use.
* **Precise Calibration:** Manual CLI-only maintenance command, takes much longer and should only be run with the device lying still on a flat surface.

Recommended procedure:

1. Let the device thermally stabilize for 5-10 minutes before precise calibration.
2. Place the controller on a flat, level, motionless surface.
3. Run `calibrate precise`.
4. Optionally switch to stored offsets only via `set boot_calibration off` and `save`.

### License

This project is licensed under the [MIT License](LICENSE.txt).

---
