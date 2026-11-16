"""명령 발행 전 로컬 x500 SITL을 읽기 전용으로 확인한다."""

from pathlib import Path
import subprocess

import config


def verify_local_sitl() -> int:
    binary = (config.PX4_ROOT / "build/px4_sitl_default/bin/px4").resolve()
    matches = []
    for process in Path("/proc").iterdir():
        if not process.name.isdigit():
            continue
        try:
            if (process / "comm").read_text().strip() == "px4":
                matches.append((process / "exe").resolve())
        except (OSError, PermissionError):
            continue
    if matches != [binary]:
        raise RuntimeError("로컬 PX4 SITL 한 대를 확인할 수 없습니다. 비행을 시작하지 않습니다.")

    def parameter(name: str) -> float:
        result = subprocess.run(
            [str(binary.parent / "px4-param"), "show", "-q", name],
            capture_output=True, text=True, check=True, timeout=3,
        )
        return float(result.stdout.strip())

    if parameter("SYS_AUTOSTART") != 4001 or parameter("SIM_GZ_EN") != 1:
        raise RuntimeError("기본 Gazebo x500 SITL 설정이 아닙니다.")
    if parameter("COM_DISARM_LAND") <= 0:
        raise RuntimeError("착륙 후 자동 Disarm이 비활성입니다. 설정을 확인하세요.")
    print(
        "SITL 확인: x500, "
        f"Offboard-loss timeout={parameter('COM_OF_LOSS_T')}s, "
        f"action={int(parameter('COM_OBL_RC_ACT'))} (0=Position, 4=Land)",
        flush=True,
    )
    return int(parameter("MAV_SYS_ID"))
