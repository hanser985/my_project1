// applications/buzzer_task.cpp
#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim4;

extern "C" void buzzer_task()
{
  // 【重点】对象定义放进函数内部，3个参数：定时器句柄、通道3、84M时钟
  sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84000000.0f);

  buzzer.start();

  while (true) {
    buzzer.set(523);  // C5 do，523Hz
    osDelay(200);
    buzzer.set(587);  // D5 re
    osDelay(200);
    buzzer.set(659);  // E5 mi
    osDelay(300);

    buzzer.set(0);  // 频率0，关闭蜂鸣器
    osDelay(3000);
  }
}