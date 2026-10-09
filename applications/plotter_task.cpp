#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"

extern sp::RM_Motor motor_a, motor_b;
extern sp::DBus remote;
extern float target_a_debug;
extern float target_b_debug;

extern sp::Mahony imu;

sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (true) {
    // 通道 1：yaw；通道 2：A 目标角；通道 3：B 目标角，单位均为 rad。
    plotter.plot(imu.yaw, target_a_debug, target_b_debug);

    osDelay(10);
  }
}
