interface PatternsPanelProps {
  enabledPatterns: number[];
  activePattern: number;
}

export function PatternsPanel(props: PatternsPanelProps) {
  const { enabledPatterns, activePattern } = props;

  return (
    <section className="panel">
      <div className="panel-header">
        <h2>Patterns</h2>
        <span className="muted">Button cycling filter</span>
      </div>

      <div className="pattern-grid">
        {Array.from({ length: 13 }, (_, index) => index + 1).map((pattern) => {
          const enabled = enabledPatterns.includes(pattern);
          const active = activePattern === pattern;
          return (
            <label
              key={pattern}
              className={`pattern-card${enabled ? " enabled" : ""}${active ? " active" : ""}`}
            >
              <input type="checkbox" checked={enabled} readOnly />
              <span>Pattern {pattern}</span>
            </label>
          );
        })}
      </div>

      <div className="button-row">
        <button type="button">Enable All</button>
        <button type="button">Read Pattern List</button>
      </div>
    </section>
  );
}
