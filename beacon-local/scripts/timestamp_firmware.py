Import("env")

from datetime import datetime
from pathlib import Path
import shutil


def copy_timestamped_firmware(source, target, env):
    firmware = Path(env.subst("$BUILD_DIR")) / "firmware.bin"
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    prefix = "btkiste" if env["PIOENV"] == "btkiste_timestamp" else "btsend"
    destination = Path(env.subst("$PROJECT_DIR")) / f"{prefix}-{timestamp}.bin"
    shutil.copy2(firmware, destination)


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", copy_timestamped_firmware)