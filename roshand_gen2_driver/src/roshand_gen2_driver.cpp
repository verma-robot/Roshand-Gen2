#include <vector>
#include "roshand_driver.h"

namespace roshand_gen2 {


   roshand_gen2_hardware::roshand_gen2_hardware():sp(NULL){ }//2021.07.28

   roshand_gen2_hardware::~roshand_gen2_hardware() {
     if(sp)delete sp;

     if (sp) {
     sp -> cancel();
     sp -> close();

     }

     iosev.stop();
     iosev.reset();

     delete sp;
  }


  bool roshand_gen2_hardware::init(std:: string port_name, int port_rate) {


         sp = new boost::asio::serial_port(iosev);

         try {

              sp -> open(port_name, ec); 
              
              if(sp -> is_open())
              {

                  sp -> set_option(boost::asio::serial_port::baud_rate(port_rate));
	          sp -> set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
	          sp -> set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
	          sp -> set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
	          sp -> set_option(boost::asio::serial_port::character_size(8));

                  tcflush(STDIN_FILENO, TCIFLUSH);
                  tcflush(STDOUT_FILENO, TCOFLUSH);
                  usleep(10000);
                  ros::Time::init();
	          current_time = ros::Time::now();
	          last_time = ros::Time::now();
                  ROS_INFO("Serial open succeed.......");
              }
              else ROS_ERROR( "Serial open failed, please retry .......") ;
         }
         catch(...) {

              ROS_ERROR( "Can't open serial port") ;
         }

         return true;
  }

void roshand_gen2_hardware::set_sensor_thread_hold( void ) {

    uint8_t data_read[33] = {0xFF, 0x01, 0x04, 0x1C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x6D};  

    data_read[4] = sensor_thread_hold[0][0];
    data_read[5] = sensor_thread_hold[0][1];
    data_read[6] = sensor_thread_hold[0][2];
    data_read[7] = sensor_thread_hold[0][3];
    data_read[8] = sensor_thread_hold[0][4];
    data_read[9] = sensor_thread_hold[0][5];
    data_read[10] = sensor_thread_hold[0][6];
    data_read[11] = sensor_thread_hold[0][7];
    data_read[12] = sensor_thread_hold[0][8];
    data_read[13] = sensor_thread_hold[0][9];
    data_read[14] = sensor_thread_hold[0][10];
    data_read[15] = sensor_thread_hold[0][11];
    data_read[16] = sensor_thread_hold[0][12];
    data_read[17] = sensor_thread_hold[0][13];

    data_read[18] = sensor_thread_hold[1][0];
    data_read[19] = sensor_thread_hold[1][1];
    data_read[20] = sensor_thread_hold[1][2];
    data_read[21] = sensor_thread_hold[1][3];
    data_read[22] = sensor_thread_hold[1][4];
    data_read[23] = sensor_thread_hold[1][5];
    data_read[24] = sensor_thread_hold[1][6];
    data_read[25] = sensor_thread_hold[1][7];
    data_read[26] = sensor_thread_hold[1][8];
    data_read[27] = sensor_thread_hold[1][9];
    data_read[28] = sensor_thread_hold[1][10];
    data_read[29] = sensor_thread_hold[1][11];
    data_read[30] = sensor_thread_hold[1][12];
    data_read[31] = sensor_thread_hold[1][13];


    boost::asio::write(*sp, boost::asio::buffer(&data_read[0], 33), ec);
//std::cout << "set threadhold...." << std::endl;
    listen_data(33, 1);
}


void roshand_gen2_hardware::handle_read( char buf[], boost::system::error_code ec, std::size_t bytes_transferred )
{


    READ_BUFFER_SIZE = bytes_transferred;
//std::cout << int16_t(buf[2]) << "," << READ_BUFFER_SIZE << std::endl;
    if(READ_BUFFER_SIZE == 5)
    {   

         if(buf[2] == 0x0A && buf[3] == 0x00 && buf[4] == 0x6D)while(CALIBRATE_SENSOR_FINESHED == false){CALIBRATE_SENSOR_FINESHED = true; }//calibrate sensor succeed
         else if(buf[2]==0x08 && buf[3]==0x00 && buf[4]==0x6D){while(FingerCloseWithSensor == true)FingerCloseWithSensor = false; }
         else if(buf[2]==0x08 && buf[3]==0x01 && buf[4]==0x6D){while(FingerCloseWithSensor == false)FingerCloseWithSensor = true; }

         else if(buf[2]==0x09 && buf[3]==0x00 && buf[4]==0x6D){while(FingerOpen == true)FingerOpen = false; }
         else if(buf[2]==0x09 && buf[3]==0x01 && buf[4]==0x6D){while(FingerOpen == false)FingerOpen = true; }

         else if(buf[2]==0x07 && buf[3]==0x00 && buf[4]==0x6D){while(FingerCloseWithOutSensor == true)FingerCloseWithOutSensor = false; }
         else if(buf[2]==0x07 && buf[3]==0x01 && buf[4]==0x6D){while(FingerCloseWithOutSensor == false)FingerCloseWithOutSensor = true; }
         else if(buf[2]==0x05 && buf[3]==0x00 && buf[4]==0x6D){while(SET_SENSOR_BIAS == false)SET_SENSOR_BIAS = true;}//set sensor bias succeed
         for(int i = 0 ; i < 5 ; i++ )buf[i] = 0;//clear the buf

    }  
    else if(READ_BUFFER_SIZE == 33)
    {
           //std::cout <<  READ_BUFFER_SIZE<<std::endl ;
           if(buf[2] == 0x04 && buf[3] == 0x1C && buf[32] == 0x6D)
           {


                 sensor_thread_hold[0][0] = buf[4 + 0 * 14 + 0];
                 sensor_thread_hold[0][1] = buf[4 + 0 * 14 + 1];
                 sensor_thread_hold[0][2] = buf[4 + 0 * 14 + 2];
                 sensor_thread_hold[0][3] = buf[4 + 0 * 14 + 3];
                 sensor_thread_hold[0][4] = buf[4 + 0 * 14 + 4];
                 sensor_thread_hold[0][5] = buf[4 + 0 * 14 + 5];
                 sensor_thread_hold[0][6] = buf[4 + 0 * 14 + 6];
                 sensor_thread_hold[0][7] = buf[4 + 0 * 14 + 7];
                 sensor_thread_hold[0][8] = buf[4 + 0 * 14 + 8];
                 sensor_thread_hold[0][9] = buf[4 + 0 * 14 + 9];
                 sensor_thread_hold[0][10] = buf[4 + 0 * 14 + 10];
                 sensor_thread_hold[0][11] = buf[4 + 0 * 14 + 11];
                 sensor_thread_hold[0][12] = buf[4 + 0 * 14 + 12];
                 sensor_thread_hold[0][13] = buf[4 + 0 * 14 + 13];

                 sensor_thread_hold[1][0] = buf[4 + 1 * 14 + 0];
                 sensor_thread_hold[1][1] = buf[4 + 1 * 14 + 1];
                 sensor_thread_hold[1][2] = buf[4 + 1 * 14 + 2];
                 sensor_thread_hold[1][3] = buf[4 + 1 * 14 + 3];
                 sensor_thread_hold[1][4] = buf[4 + 1 * 14 + 4];
                 sensor_thread_hold[1][5] = buf[4 + 1 * 14 + 5];
                 sensor_thread_hold[1][6] = buf[4 + 1 * 14 + 6];
                 sensor_thread_hold[1][7] = buf[4 + 1 * 14 + 7];
                 sensor_thread_hold[1][8] = buf[4 + 1 * 14 + 8];
                 sensor_thread_hold[1][9] = buf[4 + 1 * 14 + 9];
                 sensor_thread_hold[1][10] = buf[4 + 1 * 14 + 10];
                 sensor_thread_hold[1][11] = buf[4 + 1 * 14 + 11];
                 sensor_thread_hold[1][12] = buf[4 + 1 * 14 + 12];
                 sensor_thread_hold[1][13] = buf[4 + 1 * 14 + 13];

                 while(SET_SENSOR_THREADHOLD == false)SET_SENSOR_THREADHOLD = true;//set sensor threadhold succeed

           }
           for(int i = 0 ; i < 33 ; i++ )buf[i] = 0;//clear buf

    }
    else if (READ_BUFFER_SIZE == 99)
    {


           if(buf[2] == 0x03 && buf[3] == 0x5E && buf[98] == 0x6D)
           {

              uint8_t i,j;
              uint32_t position;
              uint16_t speed;
              uint16_t sensor[8];
              bool contact[8];
              double y_, m_, n_, b_;

              sensor_value data_raw;
              motor_position motor_position;
              motor_speed    motor_speed;


              data_raw.u[1] = buf[4 + 0 * 28 + 0 * 2];
              data_raw.u[0] = buf[5 + 0 * 28 + 0 * 2];
              hand_data.finger[0].sensor[0] = data_raw.f;

              hand_data.finger[0].contact[0] = (bool)((uint8_t)buf[60 + 0 * 14 + 0]);   
              hand_data.finger[0].threadhold[0] = (uint8_t)sensor_thread_hold[0][0];   


              data_raw.u[1] = buf[4 + 0 * 28 + 1 * 2];
              data_raw.u[0] = buf[5 + 0 * 28 + 1 * 2];
              hand_data.finger[0].sensor[1] = data_raw.f;

              hand_data.finger[0].contact[1] = (bool)((uint8_t)buf[60 + 0 * 14 + 1]);  
              hand_data.finger[0].threadhold[1] = (uint8_t)sensor_thread_hold[0][1];   



              data_raw.u[1] = buf[4 + 0 * 28 + 2 * 2];
              data_raw.u[0] = buf[5 + 0 * 28 + 2 * 2];
              hand_data.finger[0].sensor[2] = data_raw.f;

              hand_data.finger[0].contact[2] = (bool)((uint8_t)buf[60 + 0 * 14 + 2]); 
              hand_data.finger[0].threadhold[2] = (uint8_t)sensor_thread_hold[0][2];    


              data_raw.u[1] = buf[4 + 0 * 28 + 3 * 2];
              data_raw.u[0] = buf[5 + 0 * 28 + 3 * 2];
              hand_data.finger[0].sensor[3] = data_raw.f;

              hand_data.finger[0].contact[3] = (bool)((uint8_t)buf[60 + 0 * 14 + 3]);  
              hand_data.finger[0].threadhold[3] = (uint8_t)sensor_thread_hold[0][3];    

              data_raw.u[1] = buf[4 + 0 * 28 + 4 * 2];
              data_raw.u[0] = buf[5 + 0 * 28 + 4 * 2];
              hand_data.finger[0].sensor[4] = data_raw.f;

              hand_data.finger[0].contact[4] = (bool)((uint8_t)buf[60 + 0 * 14 + 4]);  
              hand_data.finger[0].threadhold[4] = (uint8_t)sensor_thread_hold[0][4];    

              data_raw.u[1] = buf[4 + 0 * 28 + 5 * 2];
              data_raw.u[0] = buf[5 + 0 * 28 + 5 * 2];
              hand_data.finger[0].sensor[5] = data_raw.f;
              hand_data.finger[0].contact[5] = (bool)((uint8_t)buf[60 + 0 * 14 + 5]);  
              hand_data.finger[0].threadhold[5] = (uint8_t)sensor_thread_hold[0][5];    


              data_raw.u[1] = buf[4 + 0 * 28 + 6 * 2];
              data_raw.u[0] = buf[5 + 0 * 28 + 6 * 2];
              hand_data.finger[0].sensor[6] = data_raw.f;
              hand_data.finger[0].contact[6] = (bool)((uint8_t)buf[60 + 0 * 14 + 6]); 
              hand_data.finger[0].threadhold[6] = (uint8_t)sensor_thread_hold[0][6];     

              data_raw.u[1] = buf[4 + 0 * 28 + 7 * 2];
              data_raw.u[0] = buf[5 + 0 * 28 + 7 * 2];
              hand_data.finger[0].sensor[7] = data_raw.f;
              hand_data.finger[0].contact[7] = (bool)((uint8_t)buf[60 + 0 * 14 + 7]);  
              hand_data.finger[0].threadhold[7] = (uint8_t)sensor_thread_hold[0][7]; 


              data_raw.u[1] = buf[4 + 0 * 28 + 8 * 2];
              data_raw.u[0] = buf[5 + 0 * 28 + 8 * 2];
              hand_data.finger[0].sensor[8] = data_raw.f;
              hand_data.finger[0].contact[8] = (bool)((uint8_t)buf[60 + 0 * 14 + 8]); 
              hand_data.finger[0].threadhold[8] = (uint8_t)sensor_thread_hold[0][8];      

              data_raw.u[1] = buf[4 + 0 * 28 + 9 * 2];
              data_raw.u[0] = buf[5 + 0 * 28 + 9 * 2];
              hand_data.finger[0].sensor[9] = data_raw.f;
              hand_data.finger[0].contact[9] = (bool)((uint8_t)buf[60 + 0 * 14 + 9]); 
              hand_data.finger[0].threadhold[9] = (uint8_t)sensor_thread_hold[0][9]; 

              data_raw.u[1] = buf[4 + 0 * 28 + 10 * 2];
              data_raw.u[0] = buf[5 + 0 * 28 + 10 * 2];
              hand_data.finger[0].sensor[10] = data_raw.f;
              hand_data.finger[0].contact[10] = (bool)((uint8_t)buf[60 + 0 * 14 + 10]);  
              hand_data.finger[0].threadhold[10] = (uint8_t)sensor_thread_hold[0][10];    


              data_raw.u[1] = buf[4 + 0 * 28 + 11 * 2];
              data_raw.u[0] = buf[5 + 0 * 28 + 11 * 2];
              hand_data.finger[0].sensor[11] = data_raw.f;
              hand_data.finger[0].contact[11] = (bool)((uint8_t)buf[60 + 0 * 14 + 11]); 
              hand_data.finger[0].threadhold[11] = (uint8_t)sensor_thread_hold[0][11];     


              data_raw.u[1] = buf[4 + 0 * 28 + 12 * 2];
              data_raw.u[0] = buf[5 + 0 * 28 + 12 * 2];
              hand_data.finger[0].sensor[12] = data_raw.f;
              hand_data.finger[0].contact[12] = (bool)((uint8_t)buf[60 + 0 * 14 + 12]);  
              hand_data.finger[0].threadhold[12] = (uint8_t)sensor_thread_hold[0][12];    

              data_raw.u[1] = buf[4 + 0 * 28 + 13 * 2];
              data_raw.u[0] = buf[5 + 0 * 28 + 13 * 2];
              hand_data.finger[0].sensor[13] = data_raw.f;
              hand_data.finger[0].contact[13] = (bool)((uint8_t)buf[60 + 0 * 14 + 13]);
              hand_data.finger[0].threadhold[13] = (uint8_t)sensor_thread_hold[0][13];      

              data_raw.u[1] = buf[4 + 1 * 28 + 0 * 2];
              data_raw.u[0] = buf[5 + 1 * 28 + 0 * 2];
              hand_data.finger[1].sensor[0] = data_raw.f;

              hand_data.finger[1].contact[0] = (bool)((uint8_t)buf[60 + 1 * 14 + 0]);   
              hand_data.finger[1].threadhold[0] = (uint8_t)sensor_thread_hold[1][0];   


              data_raw.u[1] = buf[4 + 1 * 28 + 1 * 2];
              data_raw.u[0] = buf[5 + 1 * 28 + 1 * 2];
              hand_data.finger[1].sensor[1] = data_raw.f;
              hand_data.finger[1].contact[1] = (bool)((uint8_t)buf[60 + 1 * 14 + 1]);  
              hand_data.finger[1].threadhold[1] = (uint8_t)sensor_thread_hold[1][1];   



              data_raw.u[1] = buf[4 + 1 * 28 + 2 * 2];
              data_raw.u[0] = buf[5 + 1 * 28 + 2 * 2];
              hand_data.finger[1].sensor[2] = data_raw.f;
              hand_data.finger[1].contact[2] = (bool)((uint8_t)buf[60 + 1 * 14 + 2]); 
              hand_data.finger[1].threadhold[2] = (uint8_t)sensor_thread_hold[1][2];    


              data_raw.u[1] = buf[4 + 1 * 28 + 3 * 2];
              data_raw.u[0] = buf[5 + 1 * 28 + 3 * 2];
              hand_data.finger[1].sensor[3] = data_raw.f;
              hand_data.finger[1].contact[3] = (bool)((uint8_t)buf[60 + 1 * 14 + 3]);  
              hand_data.finger[1].threadhold[3] = (uint8_t)sensor_thread_hold[1][3];    

              data_raw.u[1] = buf[4 + 1 * 28 + 4 * 2];
              data_raw.u[0] = buf[5 + 1 * 28 + 4 * 2];
              hand_data.finger[1].sensor[4] = data_raw.f;
              hand_data.finger[1].contact[4] = (bool)((uint8_t)buf[60 + 1 * 14 + 4]);  
              hand_data.finger[1].threadhold[4] = (uint8_t)sensor_thread_hold[1][4];    

              data_raw.u[1] = buf[4 + 1 * 28 + 5 * 2];
              data_raw.u[0] = buf[5 + 1 * 28 + 5 * 2];
              hand_data.finger[1].sensor[5] = data_raw.f;
              hand_data.finger[1].contact[5] = (bool)((uint8_t)buf[60 + 1 * 14 + 5]);  
              hand_data.finger[1].threadhold[5] = (uint8_t)sensor_thread_hold[1][5];    


              data_raw.u[1] = buf[4 + 1 * 28 + 6 * 2];
              data_raw.u[0] = buf[5 + 1 * 28 + 6 * 2];
              hand_data.finger[1].sensor[6] = data_raw.f;
              hand_data.finger[1].contact[6] = (bool)((uint8_t)buf[60 + 1 * 14 + 6]); 
              hand_data.finger[1].threadhold[6] = (uint8_t)sensor_thread_hold[1][6];     

              data_raw.u[1] = buf[4 + 1 * 28 + 7 * 2];
              data_raw.u[0] = buf[5 + 1 * 28 + 7 * 2];
              hand_data.finger[1].sensor[7] = data_raw.f;
              hand_data.finger[1].contact[7] = (bool)((uint8_t)buf[60 + 1 * 14 + 7]);  
              hand_data.finger[1].threadhold[7] = (uint8_t)sensor_thread_hold[1][7]; 


              data_raw.u[1] = buf[4 + 1 * 28 + 8 * 2];
              data_raw.u[0] = buf[5 + 1 * 28 + 8 * 2];
              hand_data.finger[1].sensor[8] = data_raw.f;
              hand_data.finger[1].contact[8] = (bool)((uint8_t)buf[60 + 1 * 14 + 8]); 
              hand_data.finger[1].threadhold[8] = (uint8_t)sensor_thread_hold[1][8];      

              data_raw.u[1] = buf[4 + 1 * 28 + 9 * 2];
              data_raw.u[0] = buf[5 + 1 * 28 + 9 * 2];
              hand_data.finger[1].sensor[9] = data_raw.f;
              hand_data.finger[1].contact[9] = (bool)((uint8_t)buf[60 + 1 * 14 + 9]); 
              hand_data.finger[1].threadhold[9] = (uint8_t)sensor_thread_hold[1][9]; 

              data_raw.u[1] = buf[4 + 1 * 28 + 10 * 2];
              data_raw.u[0] = buf[5 + 1 * 28 + 10 * 2];
              hand_data.finger[1].sensor[10] = data_raw.f;
              hand_data.finger[1].contact[10] = (bool)((uint8_t)buf[60 + 1 * 14 + 10]);  
              hand_data.finger[1].threadhold[10] = (uint8_t)sensor_thread_hold[1][10];    


              data_raw.u[1] = buf[4 + 1 * 28 + 11 * 2];
              data_raw.u[0] = buf[5 + 1 * 28 + 11 * 2];
              hand_data.finger[1].sensor[11] = data_raw.f;
              hand_data.finger[1].contact[11] = (bool)((uint8_t)buf[60 + 1 * 14 + 11]); 
              hand_data.finger[1].threadhold[11] = (uint8_t)sensor_thread_hold[1][11];     


              data_raw.u[1] = buf[4 + 1 * 28 + 12 * 2];
              data_raw.u[0] = buf[5 + 1 * 28 + 12 * 2];
              hand_data.finger[1].sensor[12] = data_raw.f;
              hand_data.finger[1].contact[12] = (bool)((uint8_t)buf[60 + 1 * 14 + 12]);  
              hand_data.finger[1].threadhold[12] = (uint8_t)sensor_thread_hold[1][12];    

              data_raw.u[1] = buf[4 + 1 * 28 + 13 * 2];
              data_raw.u[0] = buf[5 + 1 * 28 + 13 * 2];
              hand_data.finger[1].sensor[13] = data_raw.f;
              hand_data.finger[1].contact[13] = (bool)((uint8_t)buf[60 + 1 * 14 + 13]);
              hand_data.finger[1].threadhold[13] = (uint8_t)sensor_thread_hold[1][13];      

              
              motor_position.u[3] = buf[88];
              motor_position.u[2] = buf[89];
              motor_position.u[1] = buf[90];
              motor_position.u[0] = buf[91];             

              if(motor_position.f < 0)motor_position.f = 0;
              float motor_up_distance = motor_position.f / 16384;
              motor_up_distance = 36.568 - motor_up_distance;
              float m = (motor_up_distance * motor_up_distance - 7.9199) / (38 * motor_up_distance);
              float n = 8.49 / motor_up_distance;

              float linear_angle = ( m * n + sqrt(n * n - m * m + 1 )) / (n * n + 1);
              linear_angle = acos(linear_angle);
              linear_angle = 2.397 - linear_angle;//linear finger 与角度
              float finger_distance = 2 * (7.23 - 50 * cos(linear_angle));

              motor_speed.u[1] = buf[92];
              motor_speed.u[0] = buf[93];   
                  
              hand_data.motor_data.motor_speed = motor_speed.f * 0.1;
              
              if(hand_data.motor_data.motor_speed < 1)hand_data.motor_data.motor_speed = 0.00;

              hand_data.motor_data.motor_supply_voltage = ((uint8_t)buf[94]) * 0.2;
              hand_data.motor_data.motor_current = ((uint8_t)buf[95]) * 0.03;
              hand_data.motor_data.motor_tempreture = ((uint8_t)buf[96]) * 0.4;
              hand_data.motor_data.motor_error_code = buf[97];  

              hand_data.motor_data.motor_turn = (float)(motor_position.f); 
              hand_data.motor_data.fingertips_distance = finger_distance;               

              linear_angle = 2.497 - linear_angle;

              hand_data.motor_data.inner_finger_angle = linear_angle; 


              jointstate.header.frame_id = "";
              jointstate.header.stamp = ros::Time::now();
              joint_name = "left_inner_finger_joint";
              jointstate.name.clear();
              jointstate.position.clear();
              jointstate.name.push_back(joint_name);
              jointstate.position.push_back(linear_angle);


        
              while(READ_DATA == false)READ_DATA = true;

           }       

         for(int i = 0 ; i < 99 ; i++ )buf[i] = 0;
      }
}


void roshand_gen2_hardware::listen_data(uint8_t data_number, int max_seconds)
{

       if(data_number == 5)
       {

         char buf5[5];
         memset(&buf5, 0, 5);
         iosev.reset();
         boost::asio::async_read(*sp, boost::asio::buffer(buf5, sizeof(buf5)), boost::bind(&roshand_gen2_hardware::handle_read, this, buf5, _1, _2)) ;

        } 
      else if(data_number == 33)
      {
         char buf33[33];
         memset(&buf33, 0, 33);
         iosev.reset();
         boost::asio::async_read(*sp,  boost::asio::buffer(buf33, sizeof(buf33)),  boost::bind(&roshand_gen2_hardware::handle_read, this, buf33, _1, _2)) ;


        }
      else if(data_number == 99)
      {

          char buf99[99];
          memset(&buf99, 0, 99);
          iosev.reset();
          boost::asio::async_read(*sp,  boost::asio::buffer(buf99, sizeof(buf99)), boost::bind(&roshand_gen2_hardware::handle_read, this, buf99, _1, _2)) ;


        } 
       boost::asio::deadline_timer timer( iosev); 
       int ti = timer.expires_from_now(boost::posix_time::millisec(max_seconds)) ;      


       //timer.async_wait(boost::bind(&boost::asio::serial_port::cancel,boost::ref(*sp)));
       timer.wait(ec);
if(ti > 0)iosev.reset();

//std:: cout << "timer: " << int16_t(ti) << std::endl;
       try{

                 iosev.run();

       }
       catch(boost::system::system_error& ecc) {
                 std::cerr << ecc.what() << std::endl;

       }

}

//set sensor bias
void roshand_gen2_hardware::set_sensor_bias(uint16_t sensor_hi)
{
   uint8_t data_read[6] = {0xFF, 0x01, 0x05, 0x01, 0x00, 0x6D};  
   data_read[4] = ((uint16_t)sensor_hi) & 0xFF;

   tcflush(STDIN_FILENO, TCIFLUSH);
   tcflush(STDOUT_FILENO, TCOFLUSH);
   usleep(5000);
   boost::asio::write(*sp, boost::asio::buffer(&data_read[0], 6), ec);
   //std::cout << "send bias" << std::endl;
   listen_data(5, 1);

}


void roshand_gen2_hardware::close_with_sensor(uint8_t close_step_mag, uint8_t open_step_mag)
{

   uint8_t data_read[6] = {0xFF, 0x01, 0x08, 0x01, 0x00, 0x6D};  

   data_read[4] = close_step_mag;  

   boost::asio::write(*sp, boost::asio::buffer(&data_read[0], 6), ec);
   //std::cout << "close with sensor " << std::endl;
   listen_data(5, 1);

}


void roshand_gen2_hardware::close_without_sensor(float msg)
{

   uint32_t target_position;   

   uint8_t data_read[8] = {0xFF, 0x01, 0x07, 0x03, 0x00, 0x00, 0x00, 0x6D}; 


   float linear_finger_angle = acos((14.46- msg) / 100.00);

   linear_finger_angle = 2.397 - linear_finger_angle;
   float m = 2 * 19 * sin(linear_finger_angle);
   float n = -7.9199 - 322.62 * cos(linear_finger_angle);
   float up_distance = (m + sqrt(m * m - 4 * n)) / 2;
   up_distance = 36.568 - up_distance;
   float motor_counter = up_distance * 16384;
   if(motor_counter < 0 )motor_counter = 0;
   target_position = floor(motor_counter);

   data_read[4] = (target_position >> 16 ) & 0xFF;
   data_read[5] = (target_position >> 8 ) & 0xFF;
   data_read[6] = (target_position ) & 0xFF;

   boost::asio::write(*sp, boost::asio::buffer(&data_read[0], 8),ec);
  // std::cout << "close with out sensor " << std::endl;
   listen_data(5, 1);

}
void roshand_gen2_hardware::read_data()
{


   uint8_t data_read[5] = {0xFF, 0x01, 0x03, 0x00,0x6D};  

   boost::asio::write(*sp, boost::asio::buffer(&data_read[0],5),ec);
   //std::cout << "read data" << std::endl;
   listen_data(99,1);

}

void roshand_gen2_hardware::calibrate_data()
{

   uint8_t data_read[5] = {0xFF, 0x01, 0x0A, 0x00,0x6D};  

   boost::asio::write(*sp, boost::asio::buffer(&data_read[0],5),ec);
//std::cout << "calibrate data" << std::endl;
   listen_data(5, 1);

}

void roshand_gen2_hardware::open_gripper()
{

   uint8_t data_read[5] = {0xFF, 0x01, 0x09, 0x00,0x6D};  

   boost::asio::write(*sp, boost::asio::buffer(&data_read[0],5),ec);
//std::cout << "open gripper" << std::endl;
   listen_data(5, 1);


}


}
