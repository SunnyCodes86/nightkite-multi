#!/usr/bin/env python3
"""Run every assert-based host check without PlatformIO or additional dependencies."""
from pathlib import Path
import os
import subprocess
import tempfile
import re

ROOT = Path(__file__).resolve().parents[1]
SOURCES = {
    "audio_pattern": ["app/AudioPatternMath.cpp"],
    "audio_patterns_integration": ["app/AudioPatternMath.cpp"],
    "battery": ["app/Battery.cpp"],
    "sync_math": ["app/SyncMath.cpp"],
    "sync_codec": ["protocol/SyncBeaconCodec.cpp"],
    "show_control": ["app/ShowControl.cpp", "app/AudioPatternMath.cpp", "app/Battery.cpp",
                     "app/SyncEngine.cpp", "app/SyncMath.cpp", "app/PatternClock.cpp",
                     "protocol/ShowControlCodec.cpp", "protocol/SyncBeaconCodec.cpp"],
}

def runtime_source(output):
    source = (ROOT / "src/main.cpp").read_text()
    names = ["renderPatternClock", "isFollowingLiveMaster", "shouldRunAutoplayTick",
             "applyReceivedSyncBeacon", "tickShowControl", "runPatternEntry", "runPatternFrame",
             "runPatternExit", "switchToPattern", "PatternStateEntry", "PatternStateRunning",
             "PatternStateExit", "applyEffectiveBrightness", "applyBatteryBrightnessLimit",
             "syncLogicalToPhysicalLeds"]
    functions = []
    for name in names:
        # The top-level closing brace is unindented in these firmware functions.
        match = re.search(r"^[\w&]+ " + name + r"\([^\n]*\)\n\{.*?^\}", source, re.M | re.S)
        assert match, f"Missing firmware function: {name}"
        functions.append(match.group())
    (Path(output) / "runtime_under_test.inc").write_text("\n\n".join(functions))
    return ROOT / "test/host/runtime_harness.cpp"

def audio_patterns_source(output):
    source = (ROOT / "src/main.cpp").read_text()
    start = source.index("struct Pattern24Spark")
    end = source.index("\nconst PatternDefinition patternDefinitions[]", start)
    (Path(output) / "audio_patterns_under_test.inc").write_text(source[start:end])
    return ROOT / "test/host/audio_patterns_harness.cpp"

def radio_source(output):
    source = (ROOT / "src/wireless/SyncBeaconRadio.cpp").read_text()
    functions = [source[source.index("constexpr uint8_t ADV_TYPE_NONCONNECTABLE"):source.index("void formatHexBytes")]]
    for name in ["formatHexBytes", "copyV2SyncBasis", "updateAudioSyncState", "expireAudioSyncState", "handleGapReport",
                 "stopScan", "startScan", "leaveBeaconMode", "syncBeaconRadioTick"]:
        match = re.search(r"^void " + name + r"\([^\n]*\)\n\{.*?^\}", source, re.M | re.S)
        assert match, f"Missing radio function: {name}"
        functions.append(match.group())
    rm2 = (ROOT / "src/wireless/Rm2Ble.cpp").read_text()
    restore = re.search(r"^void rm2BleRestoreGattAdvertising\(\)\n\{.*?^\}", rm2, re.M | re.S)
    assert restore
    (Path(output) / "gatt_restore_under_test.inc").write_text(restore.group())
    (Path(output) / "radio_under_test.inc").write_text("\n\n".join(functions))
    return ROOT / "test/host/radio_harness.cpp"

def config_source(output):
    source = (ROOT / "src/main.cpp").read_text()
    # Execute the actual persisted field reads/writes and version migration against
    # byte-addressed EEPROM. Hardware commit/power-failure behavior is not emulated.
    begin = source.index("    EEPROM.get(EEPROM_ADDR_CONFIG_VERSION, storedConfigVersion);")
    end = source.index("  if (!loadedExtendedConfig)", begin)
    load = source[begin:end].rstrip()
    load = load[:load.rfind("}")]  # enclosing magic-valid branch
    begin = source.index("  EEPROM.put(EEPROM_ADDR_PATTERN, currentPattern);")
    end = source.index("\n  if (verbose)", begin)
    save = source[begin:end]
    variables = set(re.findall(r"EEPROM\.(?:get|put)\(\w+, (\w+)\)", load + save))
    variables -= {"EEPROM_MAGIC", "storedConfigVersion"}
    definitions = re.findall(r"^#define EEPROM_.*$", source, re.M)
    declarations = ["int " + name + " = 0;" for name in sorted(variables)]
    declarations += ["char currentDeviceUid[17] = {}, currentDeviceName[25] = {}, currentSyncMasterUid[17] = {};",
                     "constexpr int DEVICE_UID_LENGTH = 16, DEVICE_NAME_LENGTH = 24, EEPROM_MAGIC = 0x4E4B3434;"]
    generated = "\n".join(definitions + declarations)
    generated += "\nvoid writeFields() {\n" + save + "\n}\n"
    generated += "void readFields(int& storedConfigVersion, bool& migratedShowConfig) {\n"
    generated += "bool loadedExtendedConfig = false, migratedAudioPatterns = false;\n" + load
    generated += "\n(void)loadedExtendedConfig; (void)migratedAudioPatterns;\n}\n"
    (Path(output) / "config_under_test.inc").write_text(generated)
    return ROOT / "test/host/config_harness.cpp"

def main():
    tests = sorted((ROOT / "test").glob("*/test_*.cpp"))
    with tempfile.TemporaryDirectory(prefix="nightkite-host-") as output:
        tests.append(runtime_source(output))
        tests.append(audio_patterns_source(output))
        tests.append(radio_source(output))
        tests.append(config_source(output))
        for test in tests:
            name = test.stem.replace("_harness", "_integration") if test.parent.name == "host" else test.parent.name
            sources = SOURCES["show_control"] if name in ("runtime_integration", "radio_integration") else SOURCES.get(name, [])
            binary = str(Path(output) / name)
            subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11", "-Wall", "-Wextra", "-Werror",
                            "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                            "-I", str(ROOT / "test/host"), "-I", str(ROOT / "src"),
                            "-I", output, str(test), *[str(ROOT / "src" / s) for s in sources],
                            "-o", binary], check=True)
            subprocess.run([binary], check=True)
            print(f"PASS {name}", flush=True)
    print(f"PASS {len(tests)}/{len(tests)} host executables (ASan + UBSan)", flush=True)

if __name__ == "__main__":
    main()
