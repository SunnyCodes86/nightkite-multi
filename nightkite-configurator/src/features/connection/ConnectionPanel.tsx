interface ConnectionPanelProps {
  ports: string[];
  selectedPort: string;
  connected: boolean;
}

export function ConnectionPanel(props: ConnectionPanelProps) {
  const { ports, selectedPort, connected } = props;

  return (
    <section className="panel">
      <div className="panel-header">
        <h2>Connection</h2>
        <span className={connected ? "status status-ok" : "status status-off"}>
          {connected ? "Connected" : "Disconnected"}
        </span>
      </div>

      <div className="field-grid">
        <label className="field">
          <span>Serial Port</span>
          <select value={selectedPort} disabled>
            {ports.map((port) => (
              <option key={port} value={port}>
                {port}
              </option>
            ))}
          </select>
        </label>
      </div>

      <div className="button-row">
        <button type="button">Refresh Ports</button>
        <button type="button" className="primary">
          Connect
        </button>
        <button type="button">Disconnect</button>
      </div>
    </section>
  );
}
