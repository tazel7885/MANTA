#ifndef LED_HPP
#define LED_HPP

#include <stdio.h>
#include <string>

#endif

typedef struct
{
  float h;
  float s;
  float v;
} hsv;

typedef struct
{
  float r;
  float b;
  float g;
} rgb;

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
      void SetTargetColor(float cycle, int target_id, int current_id);
      void StopLed();
      hsv rgb2hsv(rgb in);
      rgb hsv2rgb(hsv in);
      bool IDCheck(int id);
      
    private:
      std::vector<Led> Leds_;
      std::vector<std::vector<int> > color_vector_;
      std::vector<hsv> target_color_vector_hsv;
      
      int id_;
      int rpi_number;
      bool change_color;
      int current;
      double gradation_time;
  };
};