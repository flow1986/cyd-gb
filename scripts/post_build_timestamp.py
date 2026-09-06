from datetime import datetime
from pathlib import Path
import shutil

Import("env")


def copy_firmware_with_timestamp(source, target, env):
    build_dir = Path(env.subst("$BUILD_DIR"))
    project_dir = Path(env.subst("$PROJECT_DIR"))
    firmware_bin = build_dir / f"{env.subst('$PROGNAME')}.bin"

    if not firmware_bin.exists():
        print(f"[timestamp] firmware not found: {firmware_bin}")
        return

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = project_dir / "builds"
    out_dir.mkdir(parents=True, exist_ok=True)
    artifact_names = {
        "cyd": "gbscanner",
        "btlogger": "btlogger",
    }
    artifact_name = artifact_names.get(env.subst("$PIOENV"), env.subst("$PIOENV"))
    out_file = out_dir / f"{artifact_name}-{timestamp}.bin"

    shutil.copy2(firmware_bin, out_file)
    root_file = project_dir / out_file.name
    shutil.copy2(firmware_bin, root_file)
    print(f"[timestamp] copied to {out_file}")
    print(f"[timestamp] copied to {root_file}")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", copy_firmware_with_timestamp)
