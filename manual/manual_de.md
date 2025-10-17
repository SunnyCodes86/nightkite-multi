# Bedienungsanleitung für die NightKite Multi LED-Drachenbeleuchtung (v2.0)

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
  12 vordefinierte Animations-Muster zur Anpassung des Erscheinungsbilds (ebenfalls Einfluss auf die Laufzeit).
- **Ladestandsanzeige:**  
  Zeigt den aktuellen Akkustand direkt über das LED-Band an.
- **Speicherfunktion:**  
  Muster & Helligkeit werden automatisch im Speicher abgelegt und beim nächsten Start wiederhergestellt.

---

### 1.1. Animations-Muster im Detail

Die NightKite Multi (v2.0) verfügt über 12 vordefinierte Animationsmuster, die per Doppelklick gewechselt werden.  
Nach dem Einschalten (und der Kalibrierung) startet der Controller mit dem zuletzt verwendeten Muster und der zuletzt gewählten Helligkeit.  
Beim ersten Start: **Muster 1 „Regenbogen“**, **Helligkeit 95**.

1. **Regenbogen-Muster:** Sanfter, kontinuierlicher Rainbow-Durchlauf über das gesamte Band. Nicht bewegungsreaktiv.  
2. **Voller String, Winkel-Farbe:** Gesamter LED-String leuchtet in einer Farbe entsprechend dem aktuellen Winkel (Yaw).  
3. **Voller String, Winkel-Farbe & Bewegungs-Helligkeit:** Wie (2), aber Helligkeit folgt der Bewegungsintensität.  
4. **LED-Lauflicht (fix) mit Fadeout, Winkel-Farbe:** Ein einzelner Punkt wandert mit fester Geschwindigkeit; Farbe = Winkel, sanfter Fadeout.  
5. **LED-Lauflicht (reaktiv) mit Fadeout-Speed, Winkel-Farbe:** Wie (4), aber Tempo und Fade-Geschwindigkeit reagieren auf Bewegung.  
6. **LED-Lauflicht zweifarbig (reaktiv), Winkel-Farbe:** Wie (5), jedoch mit zweifarbigem Schweif; Tempo & Fade bewegungsabhängig.  
7. **Heartbeat (Winkel-Farbe):** Pulsierende „Herzschlag“-Animation über das Band; Farbe = Winkel.  
8. **Ping-Pong (bouncing) mit reaktivem Fade:** Punkt läuft vor / zurück; Fadeout hängt von Bewegung ab.  
9. **Comet-Swarm (4 Kometen):** Vier Kometen jagen über alle 50 Pixel (beide Hälften gespiegelt). Farbe = Winkel.  
10. **Breath / Storm (adaptiv):**  
    - Ruhig: weiches „Atmen“ (Helligkeits-Sinus).  
    - Unruhig: „Sturm“ mit Funken, deren Anzahl mit der Bewegung zunimmt.  
11. **Jerk-Wave (Ruck-Welle):** Bewegung löst Welle von der Mitte zu den Enden aus; Farbe = Winkel.  
12. **Yaw-Spinner (mit Richtungs-Gedächtnis):** Lichtkopf mit Blur-Schweif folgt der Drehrate (Yaw-Rate) mit Totzone. Farbe = Winkel.  
13. **LED-Lauflicht zweifarbig (reaktiv, invertiert):** Wie (6), aber Laufrichtung invertiert.

---

## 2. Komponenten

Die NightKite Multi Beleuchtung besteht aus:

- **Mikrocontroller:** Pimoroni Pico LiPo mit integriertem Akku-Management (USB-C)  
- **Akku:** 500 mAh LiPo-Akku, direkt am Mikrocontroller  
- **Sensor:** MPU6050 (Gyroskop / Beschleunigungsmesser) für Lage (Yaw / Pitch / Roll) und Bewegung  
- **LED-Strips:** Zwei Stränge à 25 Pixel (= 50 Pixel gesamt), WS281x / „Fairy-String“, GRB-Reihenfolge

---

## 3. Inbetriebnahme und Bedienung

### 3.1. Controller einschalten und Initialisierung

Der Controller besitzt zwei Tasten: **links** und **rechts**.

1. **Einschalten:** Linken Button drücken.  
2. **Initialisierung:** Nach dem Einschalten erfolgt eine kurze Kalibrierung.  
   Halte den Controller ruhig, um saubere Sensor-Offsets zu ermitteln.  
3. **Bereitschaft:** Nach Abschluss der Kalibrierung schalten sich die LEDs ein.  
   Das System startet mit zuletzt verwendetem Muster und Helligkeit  
   *(bei Erstbetrieb: Muster 1, Helligkeit 95)*.

---

### 3.2. Funktionen des rechten Buttons

Der rechte Button ist ein **Multifunktions-Button**:

- **Muster / Animation wechseln (Doppelklick):**  
  → Zyklischer Wechsel zum nächsten der 12 Muster.
- **Helligkeitsstufe ändern (kurz drücken):**  
  → Nächste der 6 Helligkeitsstufen: 95 → 127 → 159 → 191 → 223 → 255 → 95.
- **Akkuladestand anzeigen (gedrückt halten):**  
  → Für 5 Sekunden zeigt das LED-Band den Akkustand an.  
    Währenddessen blinkt eine blaue LED-Marke.

**Skala der Akkuanzeige (Spannungsbasiert):**

| Anzeige | Spannung | Farbe |
|----------|-----------|--------|
| 5 LEDs  | ≥ 4.2 V | Blau |
| 4 LEDs  | 4.2 – 4.0 V | Grün |
| 3 LEDs  | 4.0 – 3.8 V | Grün |
| 2 LEDs  | 3.8 – 3.6 V | Gelb |
| 1 LED   | 3.6 – 3.4 V | Gelb |
| 1 LED   | 3.4 – 3.2 V | Rot |
| Keine LED | 3.2 – 3.0 V | (Abschaltung) |

---

### 3.3. Ausschalten

- **Ausschalten:** Linken Button erneut drücken.

---

## 4. Stromversorgung und Aufladen

Das System wird durch einen integrierten **500 mAh LiPo-Akku** versorgt.  
Der Pimoroni Pico LiPo besitzt ein intelligentes Lade-/Entlademanagement.

- **Aufladen:** Über USB-C an PC oder Netzteil anschließen.  
- **Lade-Anzeige:** Während des Ladens zeigt das Band den Füllstand als Balken;  
  eine rote LED blinkt während des aktiven Ladevorgangs.  
- **Voll geladen:** 5 blaue LEDs = voll (≥ 4.2 V); rote Lade-LED aus.  
- **Automatischer Rückwechsel:** Nach dem Trennen vom USB kehrt das System zum letzten Muster zurück.  
- **Laufzeit:** Je nach Muster und Helligkeit ca. 1 – 2,5 Stunden.

**Speicherfunktion (automatisch):**

- Muster und Helligkeit werden etwa alle 5 Minuten geprüft und bei Änderung gespeichert.  
- Beim nächsten Start werden diese Werte automatisch wiederhergestellt.

---

## 5. Wichtige Hinweise

- **Ruhige Initialisierung:** Nach dem Einschalten ruhig halten, bis LEDs aktiv sind.  
- **Wetterbedingungen:** Elektronik vor Feuchtigkeit schützen (kein Regen / Nebel).  
- **Sicherheit:** Bei Nachtflügen ausreichenden Abstand halten und freie Fläche wählen.  
- **Akku-Pflege:** Nur geeignete USB-Netzteile verwenden.  
  Akku nicht unbeaufsichtigt laden, nicht kurzschließen, nicht beschädigen oder tiefentladen.

---

**Kurzüberblick für den schnellen Einstieg:**  
`Einschalten → ruhig halten (Kalibrierung) → Kurztipp = Helligkeit → Doppelklick = Muster → Langdruck = Akkuanzeige`