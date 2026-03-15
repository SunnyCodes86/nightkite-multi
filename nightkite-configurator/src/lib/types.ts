export type BootCalibrationMode = "off" | "quick";

export interface ConfigSnapshot {
  pattern: number;
  brightness: number;
  stripLength: number;
  smoothing: number;
  accelRange: number;
  gyroRange: number;
  bootCalibration: BootCalibrationMode;
  enabledPatterns: number[];
}

export interface DiagnosticSnapshot {
  batteryVoltage: string;
  sensorState: string;
  fps: string;
  offsets: string;
}
