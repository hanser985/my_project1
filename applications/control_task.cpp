#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"

namespace
{
// CAN ID、方向符号和 PID 参数。
constexpr uint8_t MOTOR_A_ID = 1;
constexpr uint8_t MOTOR_B_ID = 2;

constexpr float CONTROL_DT = 0.001f;
constexpr float MAX_SPEED = 3.0f;             // rad/s，位置外环输出限幅
constexpr float MAX_TORQUE = 0.30f;           // N*m，速度内环输出限幅
constexpr float MAX_I_TORQUE = 0.10f;         // N*m，速度环积分限幅
constexpr float MANUAL_YAW_STEP = 0.001f;     // rad，认为 yaw 基本不变的阈值
constexpr float MANUAL_MOTOR_STEP = 0.0005f;  // rad，约小于 GM6020 一格编码器角度
constexpr float MANUAL_ANGLE_ERROR = 0.01f;   // rad，手动调整识别的最小误差

// 复位角需按实际箭头标定，先用 0 rad 占位。
constexpr float RESET_A_ANGLE = 0.0f;
constexpr float RESET_B_ANGLE = 0.0f;
constexpr float RESET_TOLERANCE = 0.03f;

float ratio_from_left_switch(sp::DBusSwitchMode left_switch)
{
  switch (left_switch) {
    case sp::DBusSwitchMode::DOWN:
      return 0.5f;

    case sp::DBusSwitchMode::MID:
      return -1.0f;

    case sp::DBusSwitchMode::UP:
      return 3.0f;
  }

  return -1.0f;
}

}  // namespace

sp::CAN can1(&hcan1);
extern sp::DBus remote;
sp::RM_Motor motor_a(MOTOR_A_ID, sp::RM_Motors::GM6020);
sp::RM_Motor motor_b(MOTOR_B_ID, sp::RM_Motors::GM6020);

// 位置外环：角度(rad) -> 速度(rad/s)。
sp::PID position_pid_a(CONTROL_DT, 1.0f, 0.0f, 0.0f, MAX_SPEED, 0.0f);
sp::PID position_pid_b(CONTROL_DT, 1.0f, 0.0f, 0.0f, MAX_SPEED, 0.0f);
// 速度内环：速度(rad/s) -> 力矩(N*m)。
sp::PID speed_pid_a(CONTROL_DT, 1.0f, 0.0f, 0.0f, MAX_TORQUE, MAX_I_TORQUE);
sp::PID speed_pid_b(CONTROL_DT, 1.0f, 0.0f, 0.0f, MAX_TORQUE, MAX_I_TORQUE);

extern sp::Mahony imu;

// Plotter 调试数据；单位分别为 rad、rad/s、N*m 和拨杆枚举值。
float yaw_continuous_debug = 0.0f;
float yaw_reference_debug = 0.0f;
float yaw_delta_debug = 0.0f;
float target_a_debug = 0.0f;
float target_b_debug = 0.0f;
float ratio_debug = 0.0f;

extern "C" void control_task()
{
  osDelay(500);
  can1.config();
  can1.start();

  float yaw_reference = 0.0f;
  float motor_a_reference = 0.0f;
  float motor_b_reference = 0.0f;
  bool reference_ready = false;
  sp::AngleUnwrapper yaw_unwrapper;
  float last_yaw_delta = 0.0f;
  float last_motor_a_angle = 0.0f;
  float last_motor_b_angle = 0.0f;

  while (true) {
    const auto now = osKernelSysTick();
    const bool motors_alive = motor_a.is_alive(now) && motor_b.is_alive(now);
    const bool remote_alive = remote.is_alive(now);
    const float yaw_continuous = yaw_unwrapper.update(imu.yaw);
    yaw_continuous_debug = yaw_continuous;
    float torque_a = 0.0f;
    float torque_b = 0.0f;

    if (motors_alive && remote_alive) {
      const auto mode = remote.sw_r;

      switch (mode) {
        case sp::DBusSwitchMode::DOWN:
          // 失能时持续更新参考零点；切到 MID 后从当前位置开始联动。
          yaw_reference = yaw_continuous;
          motor_a_reference = motor_a.angle;
          motor_b_reference = motor_b.angle;
          yaw_reference_debug = yaw_reference;
          yaw_delta_debug = 0.0f;
          target_a_debug = motor_a_reference;
          target_b_debug = motor_b_reference;
          ratio_debug = ratio_from_left_switch(remote.sw_l);
          last_yaw_delta = 0.0f;
          last_motor_a_angle = motor_a.angle;
          last_motor_b_angle = motor_b.angle;
          reference_ready = true;
          position_pid_a.clear();
          position_pid_b.clear();
          speed_pid_a.clear();
          speed_pid_b.clear();
          break;

        case sp::DBusSwitchMode::MID: {
          // 防止系统刚上线时还没有经过 DOWN 分支。
          if (!reference_ready) {
            yaw_reference = yaw_continuous;
            motor_a_reference = motor_a.angle;
            motor_b_reference = motor_b.angle;
            reference_ready = true;
            last_yaw_delta = 0.0f;
            last_motor_a_angle = motor_a.angle;
            last_motor_b_angle = motor_b.angle;
          }

          const float ratio = ratio_from_left_switch(remote.sw_l);
          float yaw_delta = yaw_continuous - yaw_reference;
          float target_a = motor_a_reference + yaw_delta;
          float target_b = motor_b_reference + ratio * yaw_delta;

          // yaw 基本不变而电机实际位置明显变化时，认为是手动调整。
          // 通过修改 yaw_reference 把这次手动位移吸收到新的联动零点，
          // 另一台电机的目标会按 ratio 同步变化。
          const float yaw_step = yaw_delta - last_yaw_delta;
          const float motor_a_step = motor_a.angle - last_motor_a_angle;
          const float motor_b_step = motor_b.angle - last_motor_b_angle;
          const float motor_a_error = motor_a.angle - target_a;
          const float motor_b_error = motor_b.angle - target_b;

          if (fabsf(yaw_step) < MANUAL_YAW_STEP) {
            if (
              fabsf(motor_a_step) > MANUAL_MOTOR_STEP &&
              fabsf(motor_a_error) > MANUAL_ANGLE_ERROR) {
              yaw_reference -= motor_a_error;
            }
            else if (
              fabsf(ratio) > 1e-3f && fabsf(motor_b_step) > MANUAL_MOTOR_STEP &&
              fabsf(motor_b_error) > MANUAL_ANGLE_ERROR) {
              yaw_reference -= motor_b_error / ratio;
            }

            yaw_delta = yaw_continuous - yaw_reference;
            target_a = motor_a_reference + yaw_delta;
            target_b = motor_b_reference + ratio * yaw_delta;
          }

          yaw_reference_debug = yaw_reference;
          yaw_delta_debug = yaw_delta;
          target_a_debug = target_a;
          target_b_debug = target_b;
          ratio_debug = ratio;

          position_pid_a.calc(target_a, motor_a.angle);
          position_pid_b.calc(target_b, motor_b.angle);
          speed_pid_a.calc(position_pid_a.out, motor_a.speed);
          speed_pid_b.calc(position_pid_b.out, motor_b.speed);
          torque_a = speed_pid_a.out;
          torque_b = speed_pid_b.out;
          last_yaw_delta = yaw_delta;
          last_motor_a_angle = motor_a.angle;
          last_motor_b_angle = motor_b.angle;
          break;
        }

        case sp::DBusSwitchMode::UP:
          yaw_reference_debug = yaw_reference;
          yaw_delta_debug = yaw_continuous - yaw_reference;
          target_a_debug = RESET_A_ANGLE;
          target_b_debug = RESET_B_ANGLE;
          ratio_debug = ratio_from_left_switch(remote.sw_l);
          position_pid_a.calc(RESET_A_ANGLE, motor_a.angle);
          position_pid_b.calc(RESET_B_ANGLE, motor_b.angle);
          speed_pid_a.calc(position_pid_a.out, motor_a.speed);
          speed_pid_b.calc(position_pid_b.out, motor_b.speed);
          torque_a = speed_pid_a.out;
          torque_b = speed_pid_b.out;
          if (fabsf(motor_a.angle - RESET_A_ANGLE) < RESET_TOLERANCE) torque_a = 0.0f;
          if (fabsf(motor_b.angle - RESET_B_ANGLE) < RESET_TOLERANCE) torque_b = 0.0f;
          break;
      }
    }
    else {
      // 遥控器或任一电机掉线时立即失能，并清除积分。
      position_pid_a.clear();
      position_pid_b.clear();
      speed_pid_a.clear();
      speed_pid_b.clear();
      reference_ready = false;
      yaw_reference_debug = 0.0f;
      yaw_delta_debug = 0.0f;
      target_a_debug = motor_a.angle;
      target_b_debug = motor_b.angle;
      ratio_debug = 0.0f;
    }

    motor_a.cmd(torque_a);
    motor_b.cmd(torque_b);
    motor_a.write(can1.tx_data);
    motor_b.write(can1.tx_data);
    can1.send(motor_a.tx_id);
    osDelay(1);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  const auto stamp_ms = osKernelSysTick();
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();
      if (can1.rx_id == motor_a.rx_id) motor_a.read(can1.rx_data, stamp_ms);
      if (can1.rx_id == motor_b.rx_id) motor_b.read(can1.rx_data, stamp_ms);
    }
  }
}
// #include "cmsis_os.h"
// #include "io/can/can.hpp"
// #include "motor/rm_motor/rm_motor.hpp"

// sp::CAN can1(&hcan1);
// sp::RM_Motor motor6020(1, sp::RM_Motors::GM6020);  // 一个电机ID为1, 电流控制模式的6020

// extern "C" void control_task()
// {
//   can1.config();
//   can1.start();

//   while (true) {
//     motor6020.cmd(0.0f);  // 调整为0.1f, 电机会旋转, 注意安全

//     motor6020.write(can1.tx_data);
//     can1.send(motor6020.tx_id);

//     osDelay(1);
//   }
// }

// extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
// {
//   auto stamp_ms = osKernelSysTick();

//   while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
//     if (hcan == &hcan1) {
//       can1.recv();

//       if (can1.rx_id == motor6020.rx_id) motor6020.read(can1.rx_data, stamp_ms);
//     }
//   }
// }
