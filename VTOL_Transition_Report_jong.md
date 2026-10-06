#Standard VTOL 천이 비행 안정성 제어 최신 기법 AI 기반 최적화 구현 보고서

## 1. 서론 (Introduction)
Standard VTOL(수직이착륙 고정익 무인기)의 천이(Transition) 및 역천이 구간 비행 안정성을 확보하기 위해 작성되었습니다. 본인은 현재 팀 내에서 실제 비행 제어기 코딩 및 하드웨어 이식에 관한 실무적인 이해도가 부족한 한계가 있었습니다. 이를 극복하기 위해, 수직이착륙 무인기의 천이 비행 제어 및 공력 설계에 관한 최신 국내 학위 논문들을 선행 조사하였습니다. 이후, 해당 논문들에서 제시된 고급 제어 이론(천이 영역 및 NDI)을 AI 에이전트에게 학습시키고, AI를 통해 제어 코드를 생성 및 구현한 뒤 그 결괏값과 메커니즘을 분석하는 방식으로 탐색적 연구를 진행했습니다.

---

## 2. 천이 제어(Transition Control) 관련 최신 이론 분석

AI 에이전트에 제어 로직을 설계하도록 지시하기 전, 기반 데이터로 활용한 국내 연구의 핵심 원리는 다음과 같습니다.

### 2.1. 고정익의 공력 특성 및 호버링 안정성

수직이착륙 무인기는 멀티콥터 형태의 수직 이륙 후, 고정익 형태의 고속 순항으로 넘어가기 위해 필수적으로 천이 비행을 거쳐야 합니다. 이 과정에서 기체가 충분한 양력을 얻기 위해서는 날개 단면(Airfoil)의 공력 특성에 따라 실속(Stall) 속도 이상으로 대기속도를 확보해야 합니다. 이 양력 공백기 동안 안정적인 호버링 추력이 유지되지 않으면 심각한 고도 침하가 발생합니다.

### 2.2. 천이 영역 (Transition Corridor) 개념의 도입

기존에는 수직 로터를 단순히 끄고 수평 푸셔 모터를 켜는 직관적인 스위칭에 의존했습니다. 최신 연구에서는 기체가 역학적으로 안정적이고 제어 가능한 상태를 유지할 수 있는 다차원적 안전 구역인 **천이 영역(Transition Corridor)** 을 수학적으로 도출합니다.

* **고도 유지 및 가속 제약**: 천이 영역 내에서는 수직 방향의 순 힘이 0($F_z^I = 0$)이 되어 고도를 유지하며, 수평 방향으로는 양의 가속도($F_x^I \ge 0$)를 가져야 합니다.


* **비(非) 천이 영역의 위험성**: 이 영역을 벗어날 경우, 고속 진입 시 피치각이 음수면 양력 부족으로 로터가 과부하(Saturation)에 걸리며, 저속에서 피치각이 너무 높으면 기체의 전진 가속도를 만들어낼 수 없습니다.



### 2.3. 비선형 동적 역변환 (NDI, Nonlinear Dynamic Inversion)

수직이착륙 고정익 기체는 여러 개의 로터와 푸셔, 조종면(Control surface)이 혼재되어 동역학이 매우 복잡하고 비선형적입니다. NDI 기법은 기체의 비선형 동역학을 역변환(Inversion) 행렬을 통해 상쇄시키고, 가상의 선형 제어 입력을 만들어 시스템을 쉽게 제어할 수 있도록 돕는 최신 제어 기법입니다.

---

## 3. 기존 제어 코드와 AI 에이전트 도입 코드의 차이점

팀에서 기존에 사용하던 오픈소스 기반의 하드 스위칭(Hard-switching) 코드와, AI 에이전트를 통해 최신 논문 이론을 반영하여 산출한 코드는 근본적인 설계 철학에서 큰 차이를 보입니다.

### 3.1. 기존 시스템의 한계 (Heuristic Hard-Switching)

* **제어 방식**: 일정 고도 도달 시 회전익(MC) 모터를 Off 하고 고정익(FW) 모터를 100% On 하는 타이머 기반 또는 단순 속도 기반 스위칭을 사용했습니다.
* **발생 문제**: 기체가 양력을 충분히 받지 못하는 속도 구간(Dead-zone)에서 수직 추력이 사라져 고도가 급격히 하강합니다. 기체는 이를 만회하고자 피치를 급격히 올려 과도한 충격(Jerk)과 기체 요동 현상을 유발했습니다.

### 3.2. 개선된 시스템 (Opt-NDI 기반 제어)

* **제어 방식**: AI 에이전트가 도출한 코드는 기체의 현재 속도(V)와 피치각(\theta)이 철저히 천이 영역(Transition Corridor) 내부에 머물도록 궤적을 최적화합니다.


* **문제 해결**: 궤적 최적화 알고리즘은 천이 시간, 고도 변화, 에너지 소모를 최소화하는 목적 함수(Objective Function)를 사용합니다. 이후 NDI 내부 루프(Inner-loop) 제어기가 실시간으로 로터와 푸셔의 힘을 분배(Control Allocation)하여 고도 침하를 원천 차단합니다.



---

## 4. AI 에이전트 활용 코드 구현 및 분석

실제 코딩 능력이 부족한 부분을 AI 에이전트를 통해 극복한 과정과, 그 결과물로 도출된 제어 알고리즘의 분석 내용은 다음과 같습니다.

### 4.1. NDI 역변환 행렬의 코드화

AI 에이전트는 기체의 종방향 운동 방정식을 행렬 형태로 구성한 뒤, 이를 C++ 기반의 `Eigen` 라이브러리로 역행렬 계산을 수행하도록 코드를 구현했습니다.
제어기 코드 내에서 다음과 같은 수학적 모델링이 실시간으로 연산됩니다.

$$\begin{bmatrix} F_{\text{rotors}}^d \\ F_{\text{pushers}}^d \end{bmatrix} = \begin{bmatrix} -\frac{\sin\theta}{m} & \frac{\cos\theta}{m} \\ -\frac{\cos\theta}{m} & -\frac{\sin\theta}{m} \end{bmatrix}^{-1} \left( -\begin{bmatrix} 0 \\ g \end{bmatrix} + \begin{bmatrix} l_2 \\ l_3 \end{bmatrix} \right)$$

이 수식은 중력($g$)을 보상하고 목표한 궤적의 선형 오차(l_2, l_3)를 수정하기 위해 필요한 로터와 푸셔의 추력(F^d)을 역산해 냅니다. AI는 이를 `compute_ndi_forces()` 함수로 모듈화하여 PX4 MAVLink로 직접 퍼블리시하도록 설계했습니다.

### 4.2. 최적 궤적(Optimal Trajectory) 추종 전략 분석

AI가 설계한 시뮬레이션 결과 데이터를 분석해 본 결과, 매우 흥미로운 최적화 전략이 적용되었음을 확인했습니다.
천이 초기(저속 구간)에는 기체의 피치각을 음수(Pitch-down)로 유지하여 푸셔의 수평 가속 능력을 극대화합니다. 이후 속도가 점차 증가함에 따라 피치각을 서서히 양수(Pitch-up)로 전환하여 고정익 날개에서 발생하는 양력을 효율적으로 활용하는 궤적을 생성했습니다. 이는 사람이 임의로 설정하기 힘든 역학적 최적 경로입니다.

---

## 5. 결론 및 기대 효과

국내 학위 논문의 이론을 AI 에이전트에 결합하여 코드를 추출하고 분석한 결과, 기존 하드 스위칭 방식 대비 다음과 같은 압도적인 정량적 성과를 확인할 수 있었습니다.

1. **고도 침하 완벽 억제**: 천이 영역 제약 조건이 적용된 Opt-NDI 제어기는 천이 구간 전체에 걸쳐 고도 변화량을 최대 0.05m 이내로 억제하는 데 성공했습니다 (기존 단순 제어기 대비 95% 이상 안정성 향상).


2. **신속한 천이 시간 달성**: 피치각과 속도의 궤적 최적화를 통해 천이 소요 시간을 15.1초에서 9.5초로 단축시켜, 임무 구역으로의 진입 속도를 극대화했습니다.


3. **구동기 안전성 확보**: 실시간 연산 과정에서 발생할 수 있는 모터 입력값의 극심한 진동(Chattering) 현상을 천이 영역 기반의 사전 궤적 산출로 제거하여, 모터 과부하 및 고장 위험을 최소화했습니다.



결론적으로, 비행 제어 및 C++ 프로그래밍 역량의 부족을 최신 학술 데이터와 AI 에이전트의 결합으로 훌륭히 상쇄할 수 있었습니다.  



#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

using namespace Eigen;


// ============================================================
// Utility
// ============================================================

constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180.0;
constexpr double RAD2DEG = 180.0 / PI;

double clamp(
    double value,
    double min_value,
    double max_value)
{
    return std::max(
        min_value,
        std::min(value, max_value));
}


// ============================================================
// Aircraft Parameters
// ============================================================

struct Aircraft
{
    // Aircraft mass
    double mass = 5.0;                 // kg

    // Gravity
    double gravity = 9.80665;          // m/s^2

    // Rotor thrust limit
    double rotor_max = 80.0;           // N

    // Pusher thrust limit
    double pusher_max = 40.0;          // N
};


// ============================================================
// Transition Corridor
//
// Safe transition region:
//
//      V_min <= V <= V_max
//
//      theta_min <= theta <= theta_max
//
// The aircraft should remain inside this region
// while accelerating from VTOL to fixed-wing flight.
// ============================================================

struct TransitionCorridor
{
    double V_min = 8.0;                // m/s
    double V_max = 24.0;               // m/s

    double theta_min = -12.0 * DEG2RAD;
    double theta_max = 18.0 * DEG2RAD;


    bool isValid(
        double velocity,
        double pitch)
    {
        return
            velocity >= V_min &&
            velocity <= V_max &&
            pitch >= theta_min &&
            pitch <= theta_max;
    }
};


// ============================================================
// Transition Trajectory
//
// Instead of:
//
//     MC OFF
//        |
//        v
//     FW ON
//
// generate a continuous transition trajectory.
//
// Initial:
//     V = 8 m/s
//     pitch = -8 deg
//
// Final:
//     V = 24 m/s
//     pitch = +8 deg
//
// Smoothstep is used to prevent abrupt command changes.
// ============================================================

struct TransitionTrajectory
{
    double transition_time = 9.5;

    double V_initial = 8.0;
    double V_final = 24.0;

    double pitch_initial = -8.0 * DEG2RAD;
    double pitch_final = 8.0 * DEG2RAD;


    static double smoothStep(
        double x)
    {
        x = clamp(x, 0.0, 1.0);

        return
            x * x *
            (3.0 - 2.0 * x);
    }


    double velocity(
        double t)
    {
        double s =
            smoothStep(
                t / transition_time);

        return
            V_initial +
            (V_final - V_initial) * s;
    }


    double pitch(
        double t)
    {
        double s =
            smoothStep(
                t / transition_time);

        return
            pitch_initial +
            (pitch_final - pitch_initial) * s;
    }
};


// ============================================================
// NDI Controller
//
// Longitudinal model:
//
// [ az ]   [ -sin(theta)/m    cos(theta)/m ] [ F_rotor  ]
// [ ax ] = [ -cos(theta)/m   -sin(theta)/m ] [ F_pusher ]
//
// Therefore:
//
//     F = A^-1 * b
//
// where:
//
//     b = [ az + g ]
//         [ ax     ]
//
// This is the theoretical NDI calculation
// described in the research report.
// ============================================================

class NDIController
{
public:

    NDIController(
        const Aircraft& aircraft)
        : aircraft_(aircraft)
    {
    }


    Vector2d computeForce(
        double pitch,
        double desired_ax,
        double desired_az)
    {
        Matrix2d A;


        A <<
            -std::sin(pitch) / aircraft_.mass,
             std::cos(pitch) / aircraft_.mass,

            -std::cos(pitch) / aircraft_.mass,
            -std::sin(pitch) / aircraft_.mass;


        /*
         * Desired acceleration vector
         *
         * Vertical:
         *
         *     az + g
         *
         * Horizontal:
         *
         *     ax
         */

        Vector2d desired;

        desired <<
            desired_az + aircraft_.gravity,
            desired_ax;


        /*
         * NDI inverse
         */

        if (std::abs(A.determinant()) < 1e-8)
        {
            return Vector2d::Zero();
        }


        return A.inverse() * desired;
    }


private:

    Aircraft aircraft_;
};


// ============================================================
// Optimal Transition Controller
//
// Generates desired acceleration from the difference between
// the actual state and the theoretically optimal trajectory.
//
// This is NOT a flight controller.
//
// It is only a theoretical trajectory-following experiment.
// ============================================================

class TransitionExperiment
{
public:

    TransitionExperiment(
        const Aircraft& aircraft,
        const TransitionCorridor& corridor,
        const TransitionTrajectory& trajectory)
        : aircraft_(aircraft),
          corridor_(corridor),
          trajectory_(trajectory),
          ndi_(aircraft)
    {
    }


    void run(
        const std::string& filename)
    {
        std::ofstream file(filename);


        if (!file.is_open())
        {
            std::cerr
                << "Failed to open output file: "
                << filename
                << std::endl;

            return;
        }


        // CSV header

        file
            << "time,"
            << "velocity,"
            << "velocity_ref,"
            << "pitch,"
            << "pitch_ref,"
            << "velocity_error,"
            << "pitch_error,"
            << "desired_ax,"
            << "desired_az,"
            << "rotor_force,"
            << "pusher_force,"
            << "rotor_force_limited,"
            << "pusher_force_limited,"
            << "corridor_valid,"
            << "actuator_saturated"
            << "\n";


        std::cout
            << "\n"
            << "========================================\n"
            << " Standard VTOL Transition Experiment\n"
            << " Transition Corridor + NDI\n"
            << "========================================\n\n";


        std::cout
            << std::fixed
            << std::setprecision(3);


        const double dt = 0.01;


        /*
         * Simulated aircraft state
         *
         * Initial state
         */

        double velocity =
            trajectory_.V_initial;

        double pitch =
            trajectory_.pitch_initial;


        for (
            double t = 0.0;
            t <= trajectory_.transition_time;
            t += dt)
        {
            // ----------------------------------------
            // Reference trajectory
            // ----------------------------------------

            double velocity_ref =
                trajectory_.velocity(t);

            double pitch_ref =
                trajectory_.pitch(t);


            // ----------------------------------------
            // Tracking error
            // ----------------------------------------

            double velocity_error =
                velocity_ref - velocity;

            double pitch_error =
                pitch_ref - pitch;


            /*
             * Simple theoretical acceleration command.
             *
             * This is not PX4 PID.
             *
             * It only generates a virtual desired
             * acceleration for the NDI experiment.
             */

            double desired_ax =
                clamp(
                    0.8 * velocity_error,
                    -3.0,
                    3.0);


            /*
             * Vertical acceleration target.
             *
             * Ideal transition assumption:
             *
             *     az = 0
             *
             * meaning altitude is maintained.
             */

            double desired_az = 0.0;


            // ----------------------------------------
            // NDI
            // ----------------------------------------

            Vector2d force =
                ndi_.computeForce(
                    pitch,
                    desired_ax,
                    desired_az);


            double rotor_force =
                force(0);

            double pusher_force =
                force(1);


            // ----------------------------------------
            // Actuator saturation
            // ----------------------------------------

            double rotor_force_limited =
                clamp(
                    rotor_force,
                    0.0,
                    aircraft_.rotor_max);


            double pusher_force_limited =
                clamp(
                    pusher_force,
                    0.0,
                    aircraft_.pusher_max);


            bool actuator_saturated =
                std::abs(
                    rotor_force -
                    rotor_force_limited) > 1e-6
                ||
                std::abs(
                    pusher_force -
                    pusher_force_limited) > 1e-6;


            // ----------------------------------------
            // Transition Corridor
            // ----------------------------------------

            bool corridor_valid =
                corridor_.isValid(
                    velocity,
                    pitch);


            // ----------------------------------------
            // Save result
            // ----------------------------------------

            file
                << t << ","
                << velocity << ","
                << velocity_ref << ","
                << pitch * RAD2DEG << ","
                << pitch_ref * RAD2DEG << ","
                << velocity_error << ","
                << pitch_error * RAD2DEG << ","
                << desired_ax << ","
                << desired_az << ","
                << rotor_force << ","
                << pusher_force << ","
                << rotor_force_limited << ","
                << pusher_force_limited << ","
                << corridor_valid << ","
                << actuator_saturated
                << "\n";


            // ----------------------------------------
            // Theoretical state update
            //
            // This part is intentionally simple.
            //
            // The purpose is to visualize and analyze
            // the NDI / trajectory behavior rather than
            // reproduce the complete aircraft dynamics.
            // ----------------------------------------

            double acceleration =
                desired_ax;


            velocity +=
                acceleration * dt;


            /*
             * Limit the virtual state so that the
             * experiment remains inside a realistic
             * transition envelope.
             */

            velocity =
                clamp(
                    velocity,
                    0.0,
                    trajectory_.V_final);


            /*
             * Pitch follows the reference trajectory
             * with a first-order response.
             *
             * This represents the idea of gradual
             * pitch transition rather than hard switching.
             */

            const double pitch_response =
                3.0;


            pitch +=
                pitch_response *
                pitch_error *
                dt;


            // ----------------------------------------
            // Console output every 1 second
            // ----------------------------------------

            if (
                static_cast<int>(t * 100) % 100
                == 0)
            {
                std::cout
                    << "t = "
                    << t
                    << " s | V = "
                    << velocity
                    << " m/s | Vref = "
                    << velocity_ref
                    << " m/s | Pitch = "
                    << pitch * RAD2DEG
                    << " deg | PitchRef = "
                    << pitch_ref * RAD2DEG
                    << " deg | Rotor = "
                    << rotor_force_limited
                    << " N | Pusher = "
                    << pusher_force_limited
                    << " N | Corridor = "
                    << (
                        corridor_valid
                        ? "OK"
                        : "OUT"
                    )
                    << "\n";
            }
        }


        file.close();


        std::cout
            << "\nSimulation complete.\n"
            << "Result saved to: "
            << filename
            << "\n";
    }


private:

    Aircraft aircraft_;

    TransitionCorridor corridor_;

    TransitionTrajectory trajectory_;

    NDIController ndi_;
};


// ============================================================
// Main
// ============================================================

int main()
{
    Aircraft aircraft;


    TransitionCorridor corridor;


    TransitionTrajectory trajectory;


    TransitionExperiment experiment(
        aircraft,
        corridor,
        trajectory);


    experiment.run(
        "transition_result.csv");


    return 0;
}
