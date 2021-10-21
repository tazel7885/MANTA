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
  
      Leds_[i].SetPwmDutyCycle(int(color_vector_[id_-1][i]));
    }
  }
  
  void LedManager::StopLed()
  {
    for(int i=0; i<Leds_.size(); i++)
    {
      Leds_[i].SetPwmDutyCycle(0);
    }
  }

};
