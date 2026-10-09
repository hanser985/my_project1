#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"

const float r_ab[3][3] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

// C板
sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab);

// 达妙
// sp::BMI088 bmi088(&hspi2, GPIOC, GPIO_PIN_0, GPIOC, GPIO_PIN_3, r_ab);

sp::Mahony imu(1e-3f);

extern "C" void imu_task()
{
  bmi088.init();

  while (true) {
    bmi088.update();
    imu.update(bmi088.acc, bmi088.gyro);

    // 使用调试(f5)查看bmi088和imu内部变量的变化

    osDelay(1);
  }
}