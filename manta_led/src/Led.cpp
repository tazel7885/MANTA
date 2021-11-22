#include <stdio.h>
#include <fstream>
#include <string>
#include <ros/ros.h>

#include <pigpiod_if2.h>

#include "../include/manta_led/Led.hpp"


namespace LED
{
  //class Led
  Led::Led(int pi_num, int pin_num, int pwm_init, int pwm_range)
  {
    pi_num_ = pi_num;
    pin_num_ = pin_num;
    
    set_mode(pi_num_, pin_num_, PI_OUTPUT);


    set_PWM_range(pi_num_,pin_num_, pwm_range);
    set_PWM_frequency(pi_num_, pin_num_, 8000);

    set_PWM_dutycycle(pi_num_, pin_num_, 0);
  }

  Led::~Led()
  {
    set_PWM_dutycycle(pi_num_, pin_num_, 0);
  }

  void Led::SetPwmDutyCycle(float duty_cycle)
  {
    set_PWM_dutycycle(pi_num_, pin_num_, duty_cycle);
  }

  //class LedManager
  LedManager::LedManager(int *pin_nums, int size_pin_nums, int pwm_init, int pwm_range)
  {

    rpi_number = pigpio_start(NULL, NULL);
    if (rpi_number < 0)
    {
      ROS_INFO("Setup failed");
    }
    ROS_INFO("Setup Fin");

    for(int i=0; i<size_pin_nums; i++)
    {
      Leds_.push_back(Led(rpi_number, pin_nums[i], pwm_init, pwm_range));
    }
    change_color = false;
    current = 0;
  }

  LedManager::~LedManager()
  {

  }

  int LedManager::ReadID(std::string id_file)
  {
    int id;
    int i = 0;
    std::size_t foundid;
       
    std::ifstream inFile;
    inFile.open(id_file.c_str());
    for(std::string line; std::getline(inFile,line);)
    { 
      foundid=line.find(":");
      switch(i)
      { 
        case 0: 
          id = atof(line.substr(foundid+2).c_str()); 
          break;
        default: 
          break;
      } 
      i += 1;
    }
    inFile.close();
    ROS_INFO("id : %d",id);
    id_ = id;
    return id_;
  }

  void LedManager::SetID(int id_input)
  {
    id_ = id_input;
  }
  
  void LedManager::ReadLedColor(std::string led_color_file, int pwm_range)
  {
    std::vector<int> temp_vector;
    std::size_t foundled;
    
    std::ifstream ledFile; 
    ledFile.open(led_color_file.c_str());
    for(std::string lineled; std::getline(ledFile,lineled);)
    { 
      if(lineled.size()!=0)
      { 
        foundled=lineled.find(":");
        
        std::istringstream lineledsub(lineled.substr(foundled+2).c_str());
        std::string raw_data_led;
        
        while(std::getline(lineledsub,raw_data_led,','))
        {
          temp_vector.push_back(int(atof(raw_data_led.c_str())*pwm_range));
        }

        color_vector_.push_back(temp_vector);
        temp_vector.clear();
      }

    }

    ledFile.close();


  }

  void LedManager::SetLedColor()
  {
    //ROS_INFO("%d",id_);
    //ROS_INFO("%d : %d %d %d",id_,color_vector_[id_-1][0],color_vector_[id_-1][1],color_vector_[id_-1][2]);
    for(int i=0; i<Leds_.size(); i++)
    {
      if(change_color){
        rgb rgb_ = hsv2rgb(target_color_vector_hsv[current]);
        float target_color_vector[3] = {rgb_.b, rgb_.g, rgb_.r};
        Leds_[i].SetPwmDutyCycle(target_color_vector[i]);
        current++;
        if(current == target_color_vector_B.size()){
          current = 0;
          change_color = false;
        }
      }
      else{
        Leds_[i].SetPwmDutyCycle(int(color_vector_[id_-1][i]));
      }
    }
  }

  void LedManager::SetTargetColor(float cycle, int target_id, int current_id)
  {
    rgb target_rgb;
    target_rgb.b = int(color_vector_[target_id-1][0]);
    target_rgb.g = int(color_vector_[target_id-1][1]);
    target_rgb.r = int(color_vector_[target_id-1][2]);
    hsv target_hsv = rgb2hsv(target_rgb);

    rgb current_rgb;
    current_rgb.b = int(color_vector_[target_id-1][0]);
    current_rgb.g = int(color_vector_[target_id-1][1]);
    current_rgb.r = int(color_vector_[target_id-1][2]);
    hsv current_hsv = hsv2rgb(current_rgb);

    float error[3] = {target_hsv.h - current_hsv.h, target_hsv.s - current_hsv.s, target_hsv.v - current_hsv.v};

    target_color_vector_hsv.clear();
    target_color_vector_rgb.clear();

    for(int i = 0; i < 2 / cycle; i++){
      hsv add_error;
      add_error.h = current_hsv.h + error[0] * i;
      add_error.s = current_hsv.s + error[1] * i;
      add_error.v = current_hsv.v + error[2] * i;
      target_color_vector_hsv.push_back(add_error);
    }
    change_color = true;
    current = 0;
  }
  
  void LedManager::StopLed()
  {
    for(int i=0; i<Leds_.size(); i++)
    {
      Leds_[i].SetPwmDutyCycle(0);
    }
  }

  hsv LedManager::rgb2hsv(rgb in)
  {
    hsv        out;
    double      min, max, delta;
   
    min = in.r < in.g ? in.r : in.g;
    min = min  < in.b ? min  : in.b;
   
    max = in.r > in.g ? in.r : in.g;
    max = max  > in.b ? max  : in.b;
   
    out.v = max;                                // v
    delta = max - min;
    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
        out.s = (delta / max);                  // s
    } else {
        // if max is 0, then r = g = b = 0
        // s = 0, v is undefined
        out.s = 0.0;
        out.h = 0.0;                            // its now undefined
        return out;
    }
    if( in.r >= max )                           // > is bogus, just keeps compilor happy
        if(delta == 0){
            out.h = 0.0;
        }
        else{
            out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
        }
    else
        if( in.g >= max )
            out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
        else
            out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan
   
    out.h *= 60.0;                              // degrees
   
    if( out.h < 0.0 )
        out.h += 360.0;
   
    return out;
  }

  rgb LedManager::hsv2rgb(hsv hsv_value)
  {
    double hh, p, q, t, ff;
    long i;
    rgb out;
    
    if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
        out.r = in.v;
        out.g = in.v;
        out.b = in.v;
        return out;
    }
    hh = in.h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));
    
    switch(i) {
        case 0:
            out.r = in.v;
            out.g = t;
            out.b = p;
            break;
        case 1:
            out.r = q;
            out.g = in.v;
            out.b = p;
            break;
        case 2:
            out.r = p;
            out.g = in.v;
            out.b = t;
            break;
            
        case 3:
            out.r = p;
            out.g = q;
            out.b = in.v;
            break;
        case 4:
            out.r = t;
            out.g = p;
            out.b = in.v;
            break;
        case 5:
        default:
            out.r = in.v;
            out.g = p;
            out.b = q;
            break;
    }
    return out;    
  }

};
