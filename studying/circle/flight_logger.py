"""STEP 6 CSV 분석: python3 flight_logger.py logs/step6_....csv --plot.

비행 중 CSV 기록은 main.py가 담당하고, 이 모듈은 비행 후 읽기·요약·시각화를 한다.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics


def load_csv(path: Path):
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("CSV에 비행 데이터가 없습니다.")
    return rows


def summarize(rows):
    orbit = [r for r in rows if r["state"] in ("ORBIT", "ORBIT_SETTLE")]
    if not orbit:
        raise ValueError("원주비행 데이터가 없습니다. 비행이 시작 전에 중단됐는지 확인하세요.")
    last = rows[-1]
    target_turns = float(orbit[-1]["target_angle_rad"])/(2*math.pi)
    actual_turns = float(orbit[-1]["actual_angle_rad"])/(2*math.pi)
    phases = []
    for row in rows:
        if not phases or phases[-1]["state"] != row["state"]:
            phases.append({"state": row["state"], "elapsed_s": float(row["elapsed_s"])})
    tracking = [math.dist(
        tuple(float(r[k]) for k in ("x_ned_m", "y_ned_m", "z_ned_m")),
        tuple(float(r[k]) for k in ("target_x_ned_m", "target_y_ned_m", "target_z_ned_m")),
    ) for r in orbit]
    radial = [float(r["radial_error_m"]) for r in orbit]
    yaw = [math.degrees(float(r["yaw_error_rad"])) for r in orbit]
    inward = [math.degrees(float(r["inward_yaw_error_rad"])) for r in orbit]
    # 가감속을 제외하고 실제 수평 속도가 정속에 가까운 행으로 측면 이동을 확인한다.
    peak_speed = max(math.hypot(float(r["vx_ned_m_s"]), float(r["vy_ned_m_s"])) for r in orbit)
    cruise = [r for r in orbit if math.hypot(float(r["vx_ned_m_s"]), float(r["vy_ned_m_s"])) > .9*peak_speed]
    return {
        "completed": last["state"] == "DONE" and last["armed"] == "False"
                     and last["landed"] == "True" and not last["failure"],
        "final_state": last["state"], "failure": last["failure"],
        "duration_s": float(last["elapsed_s"]), "samples": len(rows), "phases": phases,
        "direction": "clockwise" if target_turns >= 0 else "counterclockwise",
        "target_turns": abs(target_turns),
        "actual_turns": actual_turns*(1 if target_turns >= 0 else -1),
        "signed_actual_turns": actual_turns,
        "radius_mean_m": statistics.mean(float(r["radius_m"]) for r in orbit),
        "max_abs_radial_error_m": max(map(abs, radial)),
        "radial_error_rms_m": math.sqrt(statistics.mean(v*v for v in radial)),
        "orbit_altitude_min_mean_max_m": [
            min(-float(r["z_mission_m"]) for r in orbit),
            statistics.mean(-float(r["z_mission_m"]) for r in orbit),
            max(-float(r["z_mission_m"]) for r in orbit),
        ],
        "target_position_error_mean_m": statistics.mean(tracking),
        "target_position_error_max_m": max(tracking),
        "max_abs_yaw_error_deg": max(map(abs, yaw)),
        "max_abs_inward_yaw_error_deg": max(map(abs, inward)),
        "yaw_error_rms_deg": math.sqrt(statistics.mean(v*v for v in yaw)),
        "cruise_heading_frd_vx_mean_m_s": statistics.mean(float(r["vx_heading_frd_m_s"]) for r in cruise) if cruise else None,
        "cruise_heading_frd_vy_mean_m_s": statistics.mean(float(r["vy_heading_frd_m_s"]) for r in cruise) if cruise else None,
        "max_control_gap_s": max(float(r["control_gap_s"]) for r in rows),
    }


def plot_csv(rows, path: Path, summary):
    # 상태 수신/비행에는 matplotlib이 필요하지 않다. 그래프 요청 시에만 불러온다.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    orbit = [r for r in rows if r["state"] in ("ORBIT", "ORBIT_SETTLE")]
    trajectory = [r for r in rows if r["state"] != "WAIT"]
    radius = statistics.mean(math.hypot(float(r["target_x_mission_m"]),
                                      float(r["target_y_mission_m"])) for r in orbit)
    times = [float(r["elapsed_s"]) for r in trajectory]
    orbit_times = [float(r["elapsed_s"]) for r in orbit]
    fig, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)
    xy, altitude, radial, yaw = axes.flat
    theta = [i*2*math.pi/360 for i in range(361)]
    # 위에서 보면 x(Forward)는 위, y(Right)는 오른쪽으로 배치한다.
    xy.plot([radius*math.sin(a) for a in theta], [radius*math.cos(a) for a in theta],
            "--", color="0.5", label="Target circle")
    xy.plot([float(r["y_mission_m"]) for r in trajectory],
            [float(r["x_mission_m"]) for r in trajectory], label="Actual flight")
    xy.plot(0, 0, "k+", markersize=10, label="Takeoff / centre")
    origin = next(r for r in rows if r["state"] == "PREPARE_OFFBOARD")
    outward = next(r for r in rows if r["state"] == "TURN_INWARD")
    psi0 = math.atan2(float(outward["target_y_ned_m"])-float(origin["target_y_ned_m"]),
                      float(outward["target_x_ned_m"])-float(origin["target_x_ned_m"]))
    arrows = orbit[::max(1, len(orbit)//18)]
    xy.quiver([float(r["y_mission_m"]) for r in arrows],
              [float(r["x_mission_m"]) for r in arrows],
              [.4*math.sin(float(r["yaw_ned_rad"])-psi0) for r in arrows],
              [.4*math.cos(float(r["yaw_ned_rad"])-psi0) for r in arrows],
              angles="xy", scale_units="xy", scale=1, color="tab:red", width=.004,
              label="Actual nose direction")
    xy.set(xlabel="Mission Right Y [m]", ylabel="Mission Forward X [m]", title="Top view")
    xy.axis("equal")
    xy.legend(fontsize=8)
    altitude.plot(times, [-float(r["z_mission_m"]) for r in trajectory], label="Actual")
    # AUTO_LAND 인수 이후에는 마지막 Offboard 목표를 새 착륙 목표처럼 표시하지 않는다.
    altitude.plot(times, [
        -float(r["target_z_mission_m"]) if r["nav_state"] != "18" and r["state"] not in ("DONE", "FAILED")
        else math.nan for r in trajectory
    ], "--", label="Offboard target")
    altitude.set(xlabel="Time [s]", ylabel="Relative altitude [m]", title="Altitude")
    altitude.legend()
    radial.plot(orbit_times, [float(r["radial_error_m"]) for r in orbit])
    radial.axhline(0, color="0.5", linestyle="--")
    radial.set(xlabel="Time [s]", ylabel="Radius error [m]", title="Orbit radius error")
    yaw.plot(orbit_times, [math.degrees(float(r["yaw_error_rad"])) for r in orbit], label="Setpoint error")
    if "fixed_yaw" not in path.stem:
        yaw.plot(orbit_times, [math.degrees(float(r["inward_yaw_error_rad"])) for r in orbit],
                 "--", label="Centre-facing error")
    yaw.set(xlabel="Time [s]", ylabel="Yaw error [deg]", title="Orbit yaw error")
    yaw.legend()
    for ax in axes.flat:
        ax.grid(alpha=.25)
    mode = "fixed yaw" if "fixed_yaw" in path.stem else "centre facing"
    fig.suptitle(f"PX4 SITL orbit: {mode}, {summary['actual_turns']:.3f} turns, {summary['final_state']}")
    output = path.with_name(path.stem+"_report.png")
    fig.savefig(output, dpi=140)
    plt.close(fig)
    return output


def main():
    parser = argparse.ArgumentParser(description="STEP 6 비행 CSV 요약과 그래프")
    parser.add_argument("csv", type=Path)
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()
    rows = load_csv(args.csv)
    summary = summarize(rows)
    output = args.csv.with_name(args.csv.stem+"_summary.json")
    output.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"요약: {output}")
    if args.plot:
        print(f"그래프: {plot_csv(rows, args.csv, summary)}")


if __name__ == "__main__":
    main()
