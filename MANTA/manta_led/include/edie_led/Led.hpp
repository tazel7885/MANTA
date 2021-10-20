#ifndef LED_HPP
#define LED_HPP

#include <stdio.h>
#include <string>

#endif

namespace LED
{
  class Led
  {
    public:
      Led(int pi_num, int pin_num, int pwm_init, int pwm_range);
      ~Led();
      
      void SetPwmDutyCycle(float duty_cycle);

    private:
      int pin_num_;
      int pi_num_;

  };

  class LedManager
  {
    public:
      LedManager(int *pin_nums, int size_pin_nums,  int pwm_init, int pwm_range);
      ~LedManager();
      int ReadID(std::string id_file);
      void SetID(int id_input);
      void ReadLedColor(std::string led_color_file, int pwm_range);
      void SetLedColor();
      void StopLed();

    private:
      std::vector<Led> Leds_;
      std::vector<std::vector<int> > color_vector_;
      int id_;
      int rpi_number;
  };
};
