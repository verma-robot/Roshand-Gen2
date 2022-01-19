#include <vector>
#include "roshand_driver.h"

namespace roshand_gen2 {


   roshand_gen2_hardware::roshand_gen2_hardware():sp(NULL){ }//2021.07.28

   roshand_gen2_hardware::~roshand_gen2_hardware() 
   {
      if(sp)delete sp;

      if (sp) 
      {
         sp -> cancel();
         sp -> close();

      }

      iosev.stop();
      iosev.reset();

      delete sp;
  }


  bool roshand_gen2_hardware::init(std:: string port_name, int port_rate) {


      sp = new boost::asio::serial_port(iosev);

      try 
      {

         sp -> open(port_name, ec); 
              
         if(sp -> is_open())
         {

            usleep(20000); 

            tcflush(sp->lowest_layer().native_handle(), TCIOFLUSH);//清空串口输入输出缓存

            sp -> set_option(boost::asio::serial_port::baud_rate(port_rate));
	         sp -> set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
	         sp -> set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
	         sp -> set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
	         sp -> set_option(boost::asio::serial_port::character_size(8));

                
            usleep(10000);
            ros::Time::init();
	         current_time = ros::Time::now();
	         last_time = ros::Time::now();

            if(GET_PRODUCE_INFO = true)GET_PRODUCE_INFO = false;
            int try_time = 0 ;

            while(GET_PRODUCE_INFO == false && try_time <= 5)
            {

               read_produced_info();
               try_time++;
            }

            if(GET_PRODUCE_INFO == true)
            {
               ROS_INFO("Serial open succeed.......");

            }
            else ROS_ERROR( "Serial open failed, please retry .......") ;
         }
      }
      catch(...) 
      {

         ROS_ERROR( "Can't open serial port") ;
      }

      return true;
  }

void roshand_gen2_hardware::set_sensor_threshold( void ) {

    uint8_t data_read[6] = {0xFF, 0x01, 0x04, 0x01, 0x00, 0x6D};  

    data_read[4] = sensor_threshold;    

    boost::asio::write(*sp, boost::asio::buffer(&data_read[0], 6), ec);

    listen_data(10, 1);
}


void roshand_gen2_hardware::handle_read( char buf[], boost::system::error_code ec, std::size_t bytes_transferred )
{

   READ_BUFFER_SIZE = bytes_transferred;
   if(READ_BUFFER_SIZE == 5)
   {
      if(buf[1] == 0x01 && buf[2] == 0x05 && buf[3] == 0x00 && buf[4] == 0x6D)//bis
      {
         SET_SENSOR_BIAS = true;
      }
      else if(buf[1] == 0x01 && buf[2] == 0x08 && buf[3] == 0x00 && buf[4] == 0x6D)
      {
         FingerCloseWithSensor = false;       
      }
      else if(buf[1] == 0x01 && buf[2] == 0x08 && buf[3] == 0x01 && buf[4] == 0x6D)
      {
         FingerCloseWithSensor = true;        
      }
      tcflush(sp->lowest_layer().native_handle(), TCIFLUSH);//清空串口输入输出缓存
   }
   else if(READ_BUFFER_SIZE == 7)
   {

      if(buf[1] == 0x01 && buf[2] == 0x0A && buf[3] == 0x02 && buf[6] == 0x6D)//calibrate
      {
         CALIBRATE_SENSOR_FINESHED = true;

      }
      tcflush(sp->lowest_layer().native_handle(), TCIFLUSH);//清空串口输入输出缓存
   }
   else if(READ_BUFFER_SIZE == 9)
   {
      if(buf[1] == 0x01 && buf[2] == 0x07 && buf[3] == 0x04 && buf[8] == 0x6D)//position
      {
         if(buf[4] == 0x00)FingerCloseWithOutSensor = false;
         else if(buf[4] == 0x01 && buf[5] == 0x01)FingerCloseWithOutSensor = true;
      }
      else if(buf[1] == 0x01 && buf[2] == 0x09 && buf[3] == 0x04 && buf[8] == 0x6D)
      {
         if(buf[4] == 0x00)FingerOpen = false;
         else if(buf[4] == 0x01 && buf[5] == 0x01)FingerOpen = true;
      }
      tcflush(sp->lowest_layer().native_handle(), TCIFLUSH);//清空串口输入输出缓存
   }
   else if(READ_BUFFER_SIZE == 10)
   {
      if(buf[1] == 0x01 && buf[2] == 0x04 && buf[3] == 0x05 && buf[9] == 0x6D)//UPDATA THREASHLD
      {
         SET_SENSOR_THRESHOLD = true;
      }
      tcflush(sp->lowest_layer().native_handle(), TCIFLUSH);//清空串口输入输出缓存
   }
   else if(READ_BUFFER_SIZE == 12)
   {
      if(buf[1] == 0x01 && buf[2] == 0x0B && buf[3] == 0x08 && buf[11] == 0x6D)
      {
         year PROYEAR;
         PROYEAR.u[1] = buf[4];
         PROYEAR.u[0] = buf[5];

         YEAR = PROYEAR.f;

         MONTH = buf[6];
         DAY =buf[7];
         NUMBER = buf[8];       

         std::cout << "year : "<< YEAR << " , month : " << (uint16_t)MONTH << " , day : " << (uint16_t)DAY << " , number : " << (uint16_t)NUMBER <<std::endl;
         
         GET_PRODUCE_INFO = true;
      }
      tcflush(sp->lowest_layer().native_handle(), TCIFLUSH);//清空串口输入输出缓存

   }
   else if(READ_BUFFER_SIZE == 99)
   {

      if(buf[1] == 0x01 && buf[2] == 0x03 && buf[3] == 0x5E && buf[98] == 0x6D)
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
         hand_data.finger[0].threshold[0] = sensor_threshold;   


         data_raw.u[1] = buf[4 + 0 * 28 + 1 * 2];
         data_raw.u[0] = buf[5 + 0 * 28 + 1 * 2];
         hand_data.finger[0].sensor[1] = data_raw.f;

         hand_data.finger[0].contact[1] = (bool)((uint8_t)buf[60 + 0 * 14 + 1]);  
         hand_data.finger[0].threshold[1] = sensor_threshold;   



         data_raw.u[1] = buf[4 + 0 * 28 + 2 * 2];
         data_raw.u[0] = buf[5 + 0 * 28 + 2 * 2];
         hand_data.finger[0].sensor[2] = data_raw.f;

         hand_data.finger[0].contact[2] = (bool)((uint8_t)buf[60 + 0 * 14 + 2]); 
         hand_data.finger[0].threshold[2] = sensor_threshold;    


         data_raw.u[1] = buf[4 + 0 * 28 + 3 * 2];
         data_raw.u[0] = buf[5 + 0 * 28 + 3 * 2];
         hand_data.finger[0].sensor[3] = data_raw.f;

         hand_data.finger[0].contact[3] = (bool)((uint8_t)buf[60 + 0 * 14 + 3]);  
         hand_data.finger[0].threshold[3] = sensor_threshold;    

         data_raw.u[1] = buf[4 + 0 * 28 + 4 * 2];
         data_raw.u[0] = buf[5 + 0 * 28 + 4 * 2];
         hand_data.finger[0].sensor[4] = data_raw.f;

         hand_data.finger[0].contact[4] = (bool)((uint8_t)buf[60 + 0 * 14 + 4]);  
         hand_data.finger[0].threshold[4] = sensor_threshold;    

         data_raw.u[1] = buf[4 + 0 * 28 + 5 * 2];
         data_raw.u[0] = buf[5 + 0 * 28 + 5 * 2];
         hand_data.finger[0].sensor[5] = data_raw.f;
         hand_data.finger[0].contact[5] = (bool)((uint8_t)buf[60 + 0 * 14 + 5]);  
         hand_data.finger[0].threshold[5] = sensor_threshold;    


         data_raw.u[1] = buf[4 + 0 * 28 + 6 * 2];
         data_raw.u[0] = buf[5 + 0 * 28 + 6 * 2];
         hand_data.finger[0].sensor[6] = data_raw.f;
         hand_data.finger[0].contact[6] = (bool)((uint8_t)buf[60 + 0 * 14 + 6]); 
         hand_data.finger[0].threshold[6] = sensor_threshold;     

         data_raw.u[1] = buf[4 + 0 * 28 + 7 * 2];
         data_raw.u[0] = buf[5 + 0 * 28 + 7 * 2];
         hand_data.finger[0].sensor[7] = data_raw.f;
         hand_data.finger[0].contact[7] = (bool)((uint8_t)buf[60 + 0 * 14 + 7]);  
         hand_data.finger[0].threshold[7] = sensor_threshold; 


         data_raw.u[1] = buf[4 + 0 * 28 + 8 * 2];
         data_raw.u[0] = buf[5 + 0 * 28 + 8 * 2];
         hand_data.finger[0].sensor[8] = data_raw.f;
         hand_data.finger[0].contact[8] = (bool)((uint8_t)buf[60 + 0 * 14 + 8]); 
         hand_data.finger[0].threshold[8] = sensor_threshold;      

         data_raw.u[1] = buf[4 + 0 * 28 + 9 * 2];
         data_raw.u[0] = buf[5 + 0 * 28 + 9 * 2];
         hand_data.finger[0].sensor[9] = data_raw.f;
         hand_data.finger[0].contact[9] = (bool)((uint8_t)buf[60 + 0 * 14 + 9]); 
         hand_data.finger[0].threshold[9] = sensor_threshold; 

         data_raw.u[1] = buf[4 + 0 * 28 + 10 * 2];
         data_raw.u[0] = buf[5 + 0 * 28 + 10 * 2];
         hand_data.finger[0].sensor[10] = data_raw.f;
         hand_data.finger[0].contact[10] = (bool)((uint8_t)buf[60 + 0 * 14 + 10]);  
         hand_data.finger[0].threshold[10] = sensor_threshold;    


         data_raw.u[1] = buf[4 + 0 * 28 + 11 * 2];
         data_raw.u[0] = buf[5 + 0 * 28 + 11 * 2];
         hand_data.finger[0].sensor[11] = data_raw.f;
         hand_data.finger[0].contact[11] = (bool)((uint8_t)buf[60 + 0 * 14 + 11]); 
         hand_data.finger[0].threshold[11] = sensor_threshold;     


         data_raw.u[1] = buf[4 + 0 * 28 + 12 * 2];
         data_raw.u[0] = buf[5 + 0 * 28 + 12 * 2];
         hand_data.finger[0].sensor[12] = data_raw.f;
         hand_data.finger[0].contact[12] = (bool)((uint8_t)buf[60 + 0 * 14 + 12]);  
         hand_data.finger[0].threshold[12] = sensor_threshold;    

         data_raw.u[1] = buf[4 + 0 * 28 + 13 * 2];
         data_raw.u[0] = buf[5 + 0 * 28 + 13 * 2];
         hand_data.finger[0].sensor[13] = data_raw.f;
         hand_data.finger[0].contact[13] = (bool)((uint8_t)buf[60 + 0 * 14 + 13]);
         hand_data.finger[0].threshold[13] = sensor_threshold;      

         data_raw.u[1] = buf[4 + 1 * 28 + 0 * 2];
         data_raw.u[0] = buf[5 + 1 * 28 + 0 * 2];
         hand_data.finger[1].sensor[0] = data_raw.f;

         hand_data.finger[1].contact[0] = (bool)((uint8_t)buf[60 + 1 * 14 + 0]);   
         hand_data.finger[1].threshold[0] = sensor_threshold;   


         data_raw.u[1] = buf[4 + 1 * 28 + 1 * 2];
         data_raw.u[0] = buf[5 + 1 * 28 + 1 * 2];
         hand_data.finger[1].sensor[1] = data_raw.f;
         hand_data.finger[1].contact[1] = (bool)((uint8_t)buf[60 + 1 * 14 + 1]);  
         hand_data.finger[1].threshold[1] = sensor_threshold;   



         data_raw.u[1] = buf[4 + 1 * 28 + 2 * 2];
         data_raw.u[0] = buf[5 + 1 * 28 + 2 * 2];
         hand_data.finger[1].sensor[2] = data_raw.f;
         hand_data.finger[1].contact[2] = (bool)((uint8_t)buf[60 + 1 * 14 + 2]); 
         hand_data.finger[1].threshold[2] = sensor_threshold;    


         data_raw.u[1] = buf[4 + 1 * 28 + 3 * 2];
         data_raw.u[0] = buf[5 + 1 * 28 + 3 * 2];
         hand_data.finger[1].sensor[3] = data_raw.f;
         hand_data.finger[1].contact[3] = (bool)((uint8_t)buf[60 + 1 * 14 + 3]);  
         hand_data.finger[1].threshold[3] = sensor_threshold;    

         data_raw.u[1] = buf[4 + 1 * 28 + 4 * 2];
         data_raw.u[0] = buf[5 + 1 * 28 + 4 * 2];
         hand_data.finger[1].sensor[4] = data_raw.f;
         hand_data.finger[1].contact[4] = (bool)((uint8_t)buf[60 + 1 * 14 + 4]);  
         hand_data.finger[1].threshold[4] = sensor_threshold;    

         data_raw.u[1] = buf[4 + 1 * 28 + 5 * 2];
         data_raw.u[0] = buf[5 + 1 * 28 + 5 * 2];
         hand_data.finger[1].sensor[5] = data_raw.f;
         hand_data.finger[1].contact[5] = (bool)((uint8_t)buf[60 + 1 * 14 + 5]);  
         hand_data.finger[1].threshold[5] = sensor_threshold;    


         data_raw.u[1] = buf[4 + 1 * 28 + 6 * 2];
         data_raw.u[0] = buf[5 + 1 * 28 + 6 * 2];
         hand_data.finger[1].sensor[6] = data_raw.f;
         hand_data.finger[1].contact[6] = (bool)((uint8_t)buf[60 + 1 * 14 + 6]); 
         hand_data.finger[1].threshold[6] = sensor_threshold;     

         data_raw.u[1] = buf[4 + 1 * 28 + 7 * 2];
         data_raw.u[0] = buf[5 + 1 * 28 + 7 * 2];
         hand_data.finger[1].sensor[7] = data_raw.f;
         hand_data.finger[1].contact[7] = (bool)((uint8_t)buf[60 + 1 * 14 + 7]);  
         hand_data.finger[1].threshold[7] = sensor_threshold; 


         data_raw.u[1] = buf[4 + 1 * 28 + 8 * 2];
         data_raw.u[0] = buf[5 + 1 * 28 + 8 * 2];
         hand_data.finger[1].sensor[8] = data_raw.f;
         hand_data.finger[1].contact[8] = (bool)((uint8_t)buf[60 + 1 * 14 + 8]); 
         hand_data.finger[1].threshold[8] = sensor_threshold;      

         data_raw.u[1] = buf[4 + 1 * 28 + 9 * 2];
         data_raw.u[0] = buf[5 + 1 * 28 + 9 * 2];
         hand_data.finger[1].sensor[9] = data_raw.f;
         hand_data.finger[1].contact[9] = (bool)((uint8_t)buf[60 + 1 * 14 + 9]); 
         hand_data.finger[1].threshold[9] = sensor_threshold; 

         data_raw.u[1] = buf[4 + 1 * 28 + 10 * 2];
         data_raw.u[0] = buf[5 + 1 * 28 + 10 * 2];
         hand_data.finger[1].sensor[10] = data_raw.f;
         hand_data.finger[1].contact[10] = (bool)((uint8_t)buf[60 + 1 * 14 + 10]);  
         hand_data.finger[1].threshold[10] = sensor_threshold;    


         data_raw.u[1] = buf[4 + 1 * 28 + 11 * 2];
         data_raw.u[0] = buf[5 + 1 * 28 + 11 * 2];
         hand_data.finger[1].sensor[11] = data_raw.f;
         hand_data.finger[1].contact[11] = (bool)((uint8_t)buf[60 + 1 * 14 + 11]); 
         hand_data.finger[1].threshold[11] = sensor_threshold;     


         data_raw.u[1] = buf[4 + 1 * 28 + 12 * 2];
         data_raw.u[0] = buf[5 + 1 * 28 + 12 * 2];
         hand_data.finger[1].sensor[12] = data_raw.f;
         hand_data.finger[1].contact[12] = (bool)((uint8_t)buf[60 + 1 * 14 + 12]);  
         hand_data.finger[1].threshold[12] = sensor_threshold;    

         data_raw.u[1] = buf[4 + 1 * 28 + 13 * 2];
         data_raw.u[0] = buf[5 + 1 * 28 + 13 * 2];
         hand_data.finger[1].sensor[13] = data_raw.f;
         hand_data.finger[1].contact[13] = (bool)((uint8_t)buf[60 + 1 * 14 + 13]);
         hand_data.finger[1].threshold[13] = sensor_threshold;      

              
         motor_position.u[3] = buf[88];
         motor_position.u[2] = buf[89];
         motor_position.u[1] = buf[90];
         motor_position.u[0] = buf[91];  

//解析数据
         if(motor_position.f < 0)motor_position.f = 0;
         float up_delat = motor_position.f / 16384;
         float up_distance = 36.47 - up_delat;
         float m = up_distance * up_distance - 7.9199;
         double n = -25.91594 * up_distance + 235.94933;
         double p = -27.79144 * up_distance - 220.026311;

         double p_n = n * n + p * p;
         double sqrt_p = n * n * p * p + p * p * p * p - p * p * m * m ;

         if(sqrt_p >= 0 )sqrt_p = sqrt(sqrt_p);
         else sqrt_p = 0.00;

         double cos_thelta = 0.00;
         if(p_n > 0 )
         {
            cos_thelta = (- m * n - sqrt_p) / p_n;
         }

         float linear_angle = acos(cos_thelta);
         float finger_distance = -100 * cos_thelta + 14.47;

         hand_data.motor_data.motor_supply_voltage = ((uint8_t)buf[94]) * 0.2;
         hand_data.motor_data.motor_current = ((uint8_t)buf[95]) * 0.03;
         hand_data.motor_data.motor_tempreture = ((uint8_t)buf[96]) * 0.4;
         hand_data.motor_data.motor_error_code = buf[97];  

         hand_data.motor_data.motor_turn = (float)(motor_position.f); 
         hand_data.motor_data.fingertips_distance = finger_distance;               

         linear_angle =  2.397 - linear_angle;

         hand_data.motor_data.inner_finger_angle = linear_angle; 

         jointstate.header.frame_id = "";
         jointstate.header.stamp = ros::Time::now();
         joint_name = "left_inner_finger_joint";
         jointstate.name.clear();
         jointstate.position.clear();
         jointstate.name.push_back(joint_name);
         jointstate.position.push_back(linear_angle);

         READ_DATA = true;
      }
      tcflush(sp->lowest_layer().native_handle(), TCIFLUSH);//清空串口输入输出缓存

   }
   else
   {
      tcflush(sp->lowest_layer().native_handle(), TCIFLUSH);//清空串口输入输出缓存
   }


}


void roshand_gen2_hardware::listen_data(uint8_t data_number, int max_seconds)
{


   char buffer_nu[data_number];
   memset(&buffer_nu, 0, data_number);
   iosev.reset();
   boost::asio::async_read(*sp, boost::asio::buffer(buffer_nu, sizeof(buffer_nu)), boost::bind(&roshand_gen2_hardware::handle_read, this, buffer_nu, _1, _2)) ;

   boost::asio::deadline_timer timer( iosev); 
   int ti = timer.expires_from_now(boost::posix_time::millisec(max_seconds)) ;     

//timer.async_wait(boost::bind(&boost::asio::serial_port::cancel,boost::ref(*sp)));
   timer.wait(ec);
   if(ti > 0)iosev.reset();
   try
   {
      iosev.run();
   }
   catch(boost::system::system_error& ecc) 
   {
      std::cerr << ecc.what() << std::endl;
      tcflush(sp->lowest_layer().native_handle(), TCIFLUSH);//清空串口输入输出缓存
   }

}

//set sensor bias
void roshand_gen2_hardware::set_sensor_bias(uint8_t sensor_hi)
{
   uint8_t data_read[6] = {0xFF, 0x01, 0x05, 0x01, 0x00, 0x6D};

   data_read[4] = sensor_hi;

   boost::asio::write(*sp, boost::asio::buffer(&data_read[0], 6), ec);

   listen_data(5, 1);

   tcflush(sp->lowest_layer().native_handle(), TCOFLUSH);//清空串口输入输出缓存
}


void roshand_gen2_hardware::close_with_sensor(uint8_t close_step_mag)
{

   uint8_t data_read[6] = {0xFF, 0x01, 0x08, 0x01, 0x00, 0x6D};  

   data_read[4] = close_step_mag;

   boost::asio::write(*sp, boost::asio::buffer(&data_read[0], 6), ec);

   listen_data(5, 1);
   tcflush(sp->lowest_layer().native_handle(), TCOFLUSH);//清空串口输入输出缓存
}


void roshand_gen2_hardware::close_without_sensor(float msg)
{
   uint8_t data_read[9] = {0xFF, 0x01, 0x07, 0x04, 0x00, 0x00, 0x00, 0x00,0x6D}; 

   float cos_thelta = 0.1447 - 0.01 * msg;
   float sin_thelta = 1 - cos_thelta * cos_thelta;

   if(sin_thelta >= 0)sin_thelta = sqrt(sin_thelta);
   else sin_thelta = 0.00;

   double m = 25.915938 * cos_thelta + 27.79144 * sin_thelta;
   double n = 235.94933 * cos_thelta - 220.02631 * sin_thelta - 7.9199;

   double squrt = m * m - 4 * n;
   if(squrt >= 0)squrt = sqrt(squrt);
   else squrt = 0.00;

   float delt_distance = (m + squrt) / 2;

   delt_distance = 36.47 - delt_distance;

   if(delt_distance < 0)delt_distance = 0;
   else if(delt_distance >= 19)delt_distance = 19;

   uint32_t target_position = floor(delt_distance * 16384);

   data_read[4] = (target_position >> 16 ) & 0xFF;
   data_read[5] = (target_position >> 8 ) & 0xFF;
   data_read[6] = (target_position ) & 0xFF;

   data_read[7] = 0x01;

   boost::asio::write(*sp, boost::asio::buffer(&data_read[0], 9),ec);
   listen_data(9, 1);
   tcflush(sp->lowest_layer().native_handle(), TCOFLUSH);//清空串口输入输出缓存
}
void roshand_gen2_hardware::read_data()
{


   uint8_t data_read[5] = {0xFF, 0x01, 0x03, 0x00,0x6D};  

   boost::asio::write(*sp, boost::asio::buffer(&data_read[0],5),ec);
   //std::cout << "read data" << std::endl;
   listen_data(99,1);
   tcflush(sp->lowest_layer().native_handle(), TCOFLUSH);//清空串口输入输出缓存
}

void roshand_gen2_hardware::calibrate_data()
{

   uint8_t data_read[5] = {0xFF, 0x01, 0x0A, 0x00,0x6D};  

   boost::asio::write(*sp, boost::asio::buffer(&data_read[0],5),ec);

   listen_data(7, 1);
   tcflush(sp->lowest_layer().native_handle(), TCOFLUSH);//清空串口输入输出缓存
}

void roshand_gen2_hardware::open_gripper()
{

   uint8_t data_read[6] = {0xFF, 0x01, 0x09, 0x01, 0x01, 0x6D};  

   boost::asio::write(*sp, boost::asio::buffer(&data_read[0],6),ec);

   listen_data(9, 1);
   tcflush(sp->lowest_layer().native_handle(), TCOFLUSH);//清空串口输入输出缓存
}

void roshand_gen2_hardware::read_produced_info()
{

   uint8_t data_read[5] = {0xFF, 0x01, 0x0B, 0x00, 0x6D};  

   boost::asio::write(*sp, boost::asio::buffer(&data_read[0],5),ec);
   listen_data(12, 1);
   tcflush(sp->lowest_layer().native_handle(), TCOFLUSH);//清空串口输入输出缓存
}

}
