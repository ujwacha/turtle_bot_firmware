#include <cstdint>
#include "main.h"
#include "stm32f4xx_hal_tim.h"
class Driver{
  public:
  Driver(uint32_t dchannel, uint32_t dpin, TIM_HandleTypeDef* tim_pwm, uint32_t channel, int m_pwm) {
    dir_channel = dchannel;
    dir_pin = dpin;
    timer_channel = channel;
    timer_pwm = tim_pwm;
    max_pwm = m_pwm;
  }

  void run(float percentage, int dir) {

    uint32_t pwm = (percentage / 100) * max_pwm;

    __HAL_TIM_SET_COMPARE(timer_pwm, timer_channel, pwm);
  }

  private:
  uint32_t dir_channel;
  uint32_t dir_pin;

  TIM_HandleTypeDef* timer_pwm;
  uint32_t timer_channel;
  int max_pwm;

};
