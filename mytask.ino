#include "Simulator.h"
#include <cmath>
Simulator sim;  // 定义仿真器

void setup() {
  sim.begin(Serial);  // 打开与仿真环境的串口通信
}

void loop() {
  if (sim.lidar_ready()){
    int res_angle =0,res_speed = 0,count = 0;
    float d_l = 0, d_r = 0,res_l = 0;
    for(int angle = 133; angle < 138; angle++){
      int temp = sim.read_lidar(angle);
      if(temp != 0){
        count++;
        d_r += temp;
      }
    }
    d_r = (count == 0 ? 1000 : (d_r / (float)count));
    count = 0;
    for(int angle = 160; angle < 201; angle++){
      int temp = sim.read_lidar(angle);
      if(temp != 0){
        count++;
        res_speed += temp;
      }
    }
    res_speed = 1+(count==0?200:res_speed/count)*7.5/150;
    count = 0;
    for(int angle = 223; angle < 228; angle++){
      int temp = sim.read_lidar(angle);
      if(temp != 0){
        count++;
        d_l += temp;
      }
    }
    d_l = (count == 0 ? 1000 : (d_l / (float)count));
    // get distances
    // std::cout << d_l << ' ' << d_r << std::endl;
    res_l = d_l/sqrt(d_l*d_l+d_r*d_r);
    res_angle = (std::acos(res_l)*57.3 - 45+0.5)/5.5;
    sim.send_command(res_speed, res_angle);

  }
}

/*******************************************************************************
Simulator成员函数说明:
1. void begin(HardwareSerial &serial): 开启串口serial与仿真环境的通信
2. bool lidar_ready(): 当一次完整的雷达数据(360个)接收完成后返回true, 否则返回false
3. int read_lidar(int angle): 返回角度值为angle对应的距离值, angle范围[0, 359]
4. void send_command(int velocity, int angle): 向仿真环境发送运动指令; velocity为
小车, 速度范围[-30, 30], velocity >= 0小车前进，反之后退; angle为小车舵机角度, 范围
[-30, 30], angle > 0小车右转, < 0左转, 仿真环境最高支持10Hz控制频率

基本代码框架:
#include "Simulator.h"
Simulator sim;  # 定义仿真器

void setup() {
  sim.begin(Serial);  # 打开与仿真环境的串口通信
}

void loop() {
  if (sim.lidar_ready()) {  # 接收到一次完整的雷达数据
    // 在此处处理雷达数据并发送运动指令
    // sim.read_lidar(angle);
    // sim.send_command(velocity, angle);
  }
}
******************************************************************************/
