"""AUTO_LAND 감속 설정안 조회/적용/복원. 적용은 별도 --apply 실행이 필요하다.

PX4는 변경값을 자동 저장할 수 있으므로 적용 전에 원래 값을 logs에 백업한다.
"""

import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess

import config as c
from sitl_check import verify_local_sitl


def _run(module, *arguments):
    return subprocess.run(
        [str(c.PX4_ROOT / "build/px4_sitl_default/bin" / f"px4-{module}"), *arguments],
        capture_output=True, text=True, check=True, timeout=3,
    ).stdout.strip()


def make_plan(current, desired):
    """파라미터 범위와 순서를 검사하여 일시적인 상하한 역전을 피한다."""
    if set(desired) != set(c.LANDING_PROFILE):
        raise ValueError("착륙 프로파일의 네 파라미터만 변경할 수 있습니다.")
    import math
    if not all(math.isfinite(v) for v in desired.values()):
        raise ValueError("설정값은 유한해야 합니다.")
    if not 0 <= desired["MPC_LAND_ALT2"] < desired["MPC_LAND_ALT1"] <= 122:
        raise ValueError("착륙 고도는 0 <= ALT2 < ALT1 <= 122여야 합니다.")
    if not 0.6 <= desired["MPC_LAND_SPEED"] <= desired["MPC_Z_VEL_MAX_DN"]:
        raise ValueError("PX4 v1.16의 LAND_SPEED 최솟값 0.6m/s와 하강 속도 상한을 확인하세요.")
    heights = ["MPC_LAND_ALT1", "MPC_LAND_ALT2"]
    if desired[heights[0]] < current[heights[0]]:
        heights.reverse()
    speeds = ["MPC_Z_VEL_MAX_DN", "MPC_LAND_SPEED"]
    if desired[speeds[0]] < current["MPC_LAND_SPEED"]:
        speeds.reverse()
    return [(name, desired[name]) for name in heights + speeds if current[name] != desired[name]]


def main():
    parser = argparse.ArgumentParser(description="기본 x500 SITL 착륙 프로파일 (기본: 조회만)")
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--apply", action="store_true", help="config.py의 착륙 설정 적용")
    action.add_argument("--restore", type=Path, metavar="BACKUP_JSON", help="백업의 원래 값 복원")
    args = parser.parse_args()
    verify_local_sitl()
    current = {name: float(_run("param", "show", "-q", name)) for name in c.LANDING_PROFILE}
    desired = c.LANDING_PROFILE
    if args.restore:
        desired = json.loads(args.restore.read_text(encoding="utf-8"))["values"]
    plan = make_plan(current, desired)
    print("파라미터             현재 → 목표")
    for name in current:
        print(f"{name:20} {current[name]:g} → {desired[name]:g}")
    if not args.apply and not args.restore:
        print("조회만 수행했습니다. 승인 후 --apply로 적용할 수 있습니다.")
        return
    if "Disarmed" not in _run("commander", "status"):
        raise RuntimeError("Disarmed 상태에서만 설정할 수 있습니다.")
    if not plan:
        print("이미 목표 설정과 일치합니다.")
        return
    c.LOG_DIR.mkdir(parents=True, exist_ok=True)
    backup = c.LOG_DIR / f"landing_backup_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.json"
    with backup.open("x", encoding="utf-8") as stream:
        json.dump({"values": current}, stream, indent=2)
    print(f"원래 값 백업: {backup}", flush=True)
    for name, value in plan:
        _run("param", "set", name, str(value))
        actual = float(_run("param", "show", "-q", name))
        if abs(actual - value) > 1e-4:
            raise RuntimeError(f"설정 검증 실패: {name}. 백업으로 복원하세요: {backup}")
    print("적용 및 조회 검증 완료. PX4가 설정을 자동 저장할 수 있습니다.")


if __name__ == "__main__":
    main()
