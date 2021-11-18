#ifndef ROSHAND_GEN2_H
#define ROSHAND_GEN2_H

#include <ros/ros.h>
#include <ros/time.h>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <iostream>
#include <fstream>
#include <sstream>

#include "roshand_gen2_msgs/Hand.h"
#include "roshand_gen2_msgs/Finger.h"
#include "roshand_gen2_msgs/Motor.h"
#include "roshand_gen2_msgs/HandCommandAction.h"    
#include "roshand_gen2_msgs/SensorCalibrate.h"

#include <std_srvs/Empty.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <yaml-cpp/yaml.h>

#include <actionlib/server/simple_action_server.h>
#include "sensor_msgs/JointState.h"

namespace roshand_gen2
{     
   
       typedef union
       {

          unsigned char u[2];
          int16_t f;

       }sensor_value;

       typedef union
       {

          unsigned char u[4];
          int32_t f;

       }motor_position;

       typedef union
       {

          unsigned char u[2];
          int16_t f;

       }motor_speed;

       class roshand_gen2_hardware
       {
	    public:


                    roshand_gen2_msgs::Hand hand_data;

                    bool CALIBRATE_SENSOR_FINESHED = false;
                    bool FingerCloseWithSensor = false;
                    bool FingerCloseWithOutSensor = false;
                    bool FingerOpen = false;

                    bool SET_SENSOR_BIAS = false;
                    bool SET_SENSOR_THREADHOLD = false;
                    bool READ_DATA = false;

                    std::vector<int> sensor_thread_hold[2];
                    sensor_msgs::JointState jointstate;

            public:

		    roshand_gen2_hardware(void);
                    ~roshand_gen2_hardware();   
       		    bool init(std:: string port_name, int port_rate);

                    void handle_read( char *buf, boost::system::error_code ec, std::size_t bytes_transferred );

                    void read_data();

                    void close_without_sensor(float target_position);
                    void close_with_sensor(uint8_t close_step_mag);
                    void open_gripper(void);

                    void set_sensor_thread_hold();

                    void set_sensor_bias(uint16_t sensor_hi);



                    void listen_data(uint8_t data_number, int max_seconds);
                    void calibrate_data();


            private:
                  
                    ros::Time current_time, last_time;


                    boost::asio::serial_port *sp;
                    boost::asio::io_service iosev;
                    boost::system::error_code ec;

                    std::string port_name; 
                    int port_rate;

                    int READ_BUFFER_SIZE;
                    std::string joint_name;




        };


    
}

#endif /* SENSOR_H */
