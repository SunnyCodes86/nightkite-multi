import { ConnectionPanel } from "./features/connection/ConnectionPanel";
import { ConfigPanel } from "./features/config/ConfigPanel";
import { PatternsPanel } from "./features/patterns/PatternsPanel";
import { DiagnosticsPanel } from "./features/diagnostics/DiagnosticsPanel";
import { CalibrationPanel } from "./features/calibration/CalibrationPanel";
import type { ConfigSnapshot, DiagnosticSnapshot } from "./lib/types";

const config: ConfigSnapshot = {
  pattern: 1,
  brightness: 95,
  stripLength: 25,
  smoothing: 100,
  accelRange: 2,
  gyroRange: 2000,
  bootCalibration: "quick",
  enabledPatterns: [1, 2, 3, 4, 5, 6, 7],
};

const diagnostics: DiagnosticSnapshot = {
  batteryVoltage: "3.98 V",
  sensorState: "MPU connected, DMP ready",
  fps: "120 fps target",
  offsets: "Stored offsets loaded",
};

export default function App() {
  return (
    <main className="app-shell">
      <header className="hero">
        <div>
          <p className="eyebrow">NightKite</p>
          <h1>Configurator</h1>
          <p className="hero-copy">
            Desktop configuration tool for the NightKite Multi controller.
            This initial scaffold is prepared for Tauri, React and the
            existing serial CLI protocol.
          </p>
        </div>
      </header>

      <section className="layout-grid">
        <ConnectionPanel
          ports={["/dev/cu.usbmodem101", "COM5"]}
          selectedPort="/dev/cu.usbmodem101"
          connected={false}
        />
        <ConfigPanel config={config} />
        <PatternsPanel
          activePattern={config.pattern}
          enabledPatterns={config.enabledPatterns}
        />
        <DiagnosticsPanel diagnostics={diagnostics} />
        <CalibrationPanel />
      </section>
    </main>
  );
}
