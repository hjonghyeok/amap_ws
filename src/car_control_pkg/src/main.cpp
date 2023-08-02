#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"

#define RAD2DEG(x) ((x)*180./M_PI)
using namespace std;
int scan_detect = 0;

void Scan_Callback(const sensor_msgs::LaserScan& scan){
    int noise_error = 0;
    int count = (int)(360./RAD2DEG(scan.angle_increment));
    for(int i = 0; i < count; i++){
        int degree = RAD2DEG(scan.angle_min + scan.angle_increment*i);
        float distance = scan.ranges[i];
        if((degree <= 45) && (degree >= -45) && (distance != 0.0)){
            if(distance < 0.5){
                noise_error++;
            }
        }
    }
    if(noise_error >= 10){
        scan_detect = 1;
    }
    else{
        scan_detect = 0;
    }
    printf("%d\n",scan_detect);
}

int main(int argc , char **argv){
  ros::init(argc,argv,"Control_Main");
  ros::NodeHandle nh;
  std_msgs::Int8  scan_flag;

  ros::Subscriber scan_sub = nh.subscribe("scan", 1, &Scan_Callback);
  ros::Publisher obs_flag = nh.advertise<std_msgs::Int8>("/obs_flag", 5);
  ros::Rate loop_rate(5);


  while(ros::ok()){

    scan_flag.data = scan_detect;
    obs_flag.publish(scan_flag);
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
