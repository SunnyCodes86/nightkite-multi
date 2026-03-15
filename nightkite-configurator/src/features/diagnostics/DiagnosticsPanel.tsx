import type { DiagnosticSnapshot } from "../../lib/types";

interface DiagnosticsPanelProps {
  diagnostics: DiagnosticSnapshot;
}

export function DiagnosticsPanel({ diagnostics }: DiagnosticsPanelProps) {
  return (
    <section className="panel">
      <div className="panel-header">
        <h2>Diagnostics</h2>
        <span className="muted">Live device status</span>
      </div>

      <dl className="diagnostic-grid">
        <div>
          <dt>Battery</dt>
          <dd>{diagnostics.batteryVoltage}</dd>
        </div>
        <div>
          <dt>Sensor</dt>
          <dd>{diagnostics.sensorState}</dd>
        </div>
        <div>
          <dt>Timing</dt>
          <dd>{diagnostics.fps}</dd>
        </div>
        <div>
          <dt>Offsets</dt>
          <dd>{diagnostics.offsets}</dd>
        </div>
      </dl>

      <div className="button-row">
        <button type="button">Battery</button>
        <button type="button">Sensor</button>
        <button type="button">Timing</button>
        <button type="button">Offsets</button>
      </div>
    </section>
  );
}
