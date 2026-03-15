export function CalibrationPanel() {
  return (
    <section className="panel">
      <div className="panel-header">
        <h2>Calibration</h2>
        <span className="muted">Sensor maintenance</span>
      </div>

      <div className="button-row">
        <button type="button" className="primary">
          Quick Calibration
        </button>
        <button type="button">Precise Calibration</button>
        <button type="button">Reboot Device</button>
      </div>

      <div className="log-box">
        <div>Ready.</div>
        <div>Connect to the device to start reading CLI output.</div>
      </div>
    </section>
  );
}
