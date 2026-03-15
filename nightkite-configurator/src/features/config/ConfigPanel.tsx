import type { ConfigSnapshot } from "../../lib/types";

interface ConfigPanelProps {
  config: ConfigSnapshot;
}

export function ConfigPanel({ config }: ConfigPanelProps) {
  return (
    <section className="panel">
      <div className="panel-header">
        <h2>Configuration</h2>
        <span className="muted">Firmware values</span>
      </div>

      <div className="field-grid">
        <label className="field">
          <span>Active Pattern</span>
          <input value={config.pattern} readOnly />
        </label>
        <label className="field">
          <span>Brightness</span>
          <input value={config.brightness} readOnly />
        </label>
        <label className="field">
          <span>Strip Length</span>
          <input value={config.stripLength} readOnly />
        </label>
        <label className="field">
          <span>Smoothing</span>
          <input value={config.smoothing} readOnly />
        </label>
        <label className="field">
          <span>Accel Range</span>
          <input value={config.accelRange} readOnly />
        </label>
        <label className="field">
          <span>Gyro Range</span>
          <input value={config.gyroRange} readOnly />
        </label>
        <label className="field">
          <span>Boot Calibration</span>
          <input value={config.bootCalibration} readOnly />
        </label>
      </div>

      <div className="button-row">
        <button type="button">Read Device</button>
        <button type="button" className="primary">
          Apply Changes
        </button>
        <button type="button">Save to Device</button>
      </div>
    </section>
  );
}
