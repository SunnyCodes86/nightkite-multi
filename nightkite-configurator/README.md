# NightKite Configurator

Desktop configuration tool for the NightKite Multi firmware.

Current scaffold:
- `Tauri + React + TypeScript`
- basic desktop UI layout
- feature folders for connection, config, patterns, diagnostics and calibration
- Tauri shell ready for later serial/CLI integration

Planned next steps:
1. Install dependencies with `npm install`
2. Install Rust / Cargo toolchain
3. Add serial transport and CLI parsing
4. Connect UI controls to the NightKite firmware commands

## Development

Frontend only:

```bash
npm install
npm run dev
```

Full Tauri app:

```bash
npm install
npm run tauri dev
```

## Notes

This repository currently contains only the project scaffold. Serial communication and CLI integration are not implemented yet.
