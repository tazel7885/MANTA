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
        float target_color_vector[3] = {target_color_vector_B[current], target_color_vector_R[current], target_color_vector_G[current]};
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
    float error[3] = {0,};
    for(int i = 0; i < 3; i++)
      error[i] = (int(color_vector_[current_id-1][i]) - int(color_vector_[target_id-1][i])) / (2/cycle);

    target_color_vector_B.clear();
    target_color_vector_R.clear();
    target_color_vector_G.clear();

    for(int i = 0; i < 2 / cycle; i++){
      if(target_color_vector_B.size() == 0){
        target_color_vector_B.push_back(color_vector_[current_id-1][0] + error[0]);
        target_color_vector_R.push_back(color_vector_[current_id-1][1] + error[1]);
        target_color_vector_G.push_back(color_vector_[current_id-1][2] + error[2]);
      }
      else{
        target_color_vector_B.push_back(target_color_vector_B.back() + error[0]);
        target_color_vector_R.push_back(target_color_vector_R.back() + error[1]);
        target_color_vector_G.push_back(target_color_vector_G.back() + error[2]);
      }
    }
    change_color = true;
  }
  
  void LedManager::StopLed()
  {
    for(int i=0; i<Leds_.size(); i++)
    {
      Leds_[i].SetPwmDutyCycle(0);
    }
  }

};
