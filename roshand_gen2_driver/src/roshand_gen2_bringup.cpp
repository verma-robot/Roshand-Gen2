#include "roshand_driver.h"

uint8_t max_close_step_mag = 32;
uint8_t max_open_setp_mag = 4;

roshand_gen2::roshand_gen2_hardware hand;	

ros::Publisher pub;
ros::Publisher joint_pub;

typedef actionlib::SimpleActionServer<roshand_gen2_msgs::HandCommandAction> FingerActionServer;


// move without sensor
void execute_hand_server(const roshand_gen2_msgs::HandCommandGoalConstPtr& demo_goal, FingerActionServer* as)
{
    ros::Rate r(50); 

    uint8_t close_step_mag;
    uint8_t open_step_mag;

    roshand_gen2_msgs::HandCommandFeedback feedback;   
    roshand_gen2_msgs::HandCommandResult result_;    /* 创建一个feedback对象 */    
    
    int count = 0;

    while(result_.finish == true)result_.finish = false;
    if(demo_goal -> use_sensor == true)//采用触觉控制
    {

        close_step_mag =  demo_goal -> close_step_mag;
        open_step_mag = demo_goal -> open_step_masg;
        if(close_step_mag > max_close_step_mag)close_step_mag = max_close_step_mag;
        if(open_step_mag > max_open_setp_mag)open_step_mag = max_open_setp_mag;

        while(hand.FingerCloseWithSensor == true)hand.FingerCloseWithSensor = false;
        while(hand.FingerOpen == true)hand.FingerOpen = false;
        while(hand.CALIBRATE_SENSOR_FINESHED == true)hand.CALIBRATE_SENSOR_FINESHED = false;
        while(hand.SET_SENSOR_THREADHOLD == true)hand.SET_SENSOR_THREADHOLD = false;

        while(hand.FingerOpen == false)hand.open_gripper();//张开手抓
        while(hand.CALIBRATE_SENSOR_FINESHED == false)hand.calibrate_data();//校准手抓     

     
        while(hand.SET_SENSOR_THREADHOLD == false)
        {

            for(int i = 0; i < 14; i++)hand.sensor_thread_hold[0][i] = demo_goal -> sensor_threadhold;
            for(int i = 0; i < 14; i++)hand.sensor_thread_hold[1][i] = demo_goal -> sensor_threadhold;

            hand.set_sensor_thread_hold();
        }

        while(hand.FingerCloseWithSensor == false)
        {

           hand.close_with_sensor(close_step_mag, open_step_mag);

           feedback.state = hand.hand_data;
           as ->  publishFeedback(feedback);


           r.sleep();
        }


        while(result_.finish == false)result_.finish = hand.FingerCloseWithSensor;
        as -> setSucceeded(result_); 
        while(hand.FingerCloseWithSensor == true)hand.FingerCloseWithSensor = false;
        while(hand.FingerOpen == true)hand.FingerOpen = false;
        while(hand.CALIBRATE_SENSOR_FINESHED == true)hand.CALIBRATE_SENSOR_FINESHED = false;
        while(hand.SET_SENSOR_THREADHOLD == true)hand.SET_SENSOR_THREADHOLD = false;


    }
    else //采用非触控的模式
    {

        while(hand.FingerCloseWithOutSensor == true)hand.FingerCloseWithOutSensor = false;
        while(hand.FingerCloseWithOutSensor == false)
        {

           float distance;
           distance = demo_goal -> goal_finger_distance;

           hand.close_without_sensor(distance);

           feedback.state = hand.hand_data;
           as ->  publishFeedback(feedback);
           count += 1;

           r.sleep();
        }

        while(result_.finish == false)result_.finish = hand.FingerCloseWithOutSensor;
        as -> setSucceeded(result_); 
        while(hand.FingerCloseWithOutSensor == true)hand.FingerCloseWithOutSensor = false;

    }

}



bool calibrate_senser( roshand_gen2_msgs::SensorCalibrate::Request& req,  roshand_gen2_msgs::SensorCalibrate::Response& res)
{   
   int command_account = 0;


   while(hand.CALIBRATE_SENSOR_FINESHED == false &&  command_account < 30 ){hand.calibrate_data(); command_account++ ;}
   if(hand.CALIBRATE_SENSOR_FINESHED == true){
        ROS_INFO("SENSOR CALIBRATE FINISHED............"); 
        while(res.CALIBRATE_FINISHED == false)res.CALIBRATE_FINISHED = hand.CALIBRATE_SENSOR_FINESHED ;
   }
   else {
        while(res.CALIBRATE_FINISHED == true)hand.CALIBRATE_SENSOR_FINESHED = false;
        while(res.CALIBRATE_FINISHED == true)res.CALIBRATE_FINISHED = hand.CALIBRATE_SENSOR_FINESHED ;
   }
   while(hand.CALIBRATE_SENSOR_FINESHED == true)hand.CALIBRATE_SENSOR_FINESHED=false;
   return true;

}



int main(int argc, char** argv)
{

	ros::init(argc, argv, "roshand_gen2_driver_node");	
	
        ros::NodeHandle nh("~");


        std::string port_name; 
        int port_rate;
        std::string dir; 
        int sensor_bias;
        std::string ns;

        std::vector<int> all_sensor_thread_hold[2];

        nh.getParam("port_name", port_name);
        nh.getParam("port_rate", port_rate);

        nh.getParam("sensor_bias", sensor_bias);

        nh.getParam("left_fingertips_thread_hold", all_sensor_thread_hold[0]);
        nh.getParam("right_fingertips_thread_hold", all_sensor_thread_hold[1]);


        nh.getParam("ns", ns);
	 
        hand.init(port_name, port_rate);

        while(hand.SET_SENSOR_BIAS == true)hand.SET_SENSOR_BIAS = false;
        while(hand.SET_SENSOR_BIAS == false){hand.set_sensor_bias(sensor_bias); }
        if(hand.SET_SENSOR_BIAS == true)ROS_INFO("Sensor bias set succeed....................");
        else ROS_ERROR("Sensor bias set faild, please restart........");
        while(hand.SET_SENSOR_BIAS == true)hand.SET_SENSOR_BIAS = false;

        hand.sensor_thread_hold[0] = all_sensor_thread_hold[0];
        hand.sensor_thread_hold[1] = all_sensor_thread_hold[1];
   
        while(hand.SET_SENSOR_THREADHOLD == true)hand.SET_SENSOR_THREADHOLD = false;
        while(hand.SET_SENSOR_THREADHOLD == false){hand.set_sensor_thread_hold(); }
        if(hand.SET_SENSOR_THREADHOLD == true)ROS_INFO("Sensor threadhold Set succeed......");
        else ROS_ERROR("Sensor threadhold Set faild, please restart......");
        while(hand.SET_SENSOR_THREADHOLD == true)hand.SET_SENSOR_THREADHOLD = false;

        while(hand.FingerOpen == true)hand.FingerOpen = false;
        while(hand.FingerOpen == false)hand.open_gripper();//张开手抓
        while(hand.FingerOpen == true)hand.FingerOpen = false;

        while(hand.CALIBRATE_SENSOR_FINESHED == true)hand.CALIBRATE_SENSOR_FINESHED = false;
        while(hand.CALIBRATE_SENSOR_FINESHED == false ){hand.calibrate_data(); }
        if(hand.CALIBRATE_SENSOR_FINESHED == true)ROS_INFO("Sensor calibrated succeed......");
        else ROS_ERROR("Sensor calibrated failed");
//std::cout << hand.CALIBRATE_SENSOR_FINESHED << std::endl;
        while(hand.CALIBRATE_SENSOR_FINESHED == true)hand.CALIBRATE_SENSOR_FINESHED=false;

        ros::ServiceServer calibrate_sensor = nh.advertiseService(ns + "/calibrate_sensor_service", calibrate_senser);

        pub = nh.advertise<roshand_gen2_msgs::Hand>(ns + "/RosHand_Data", 100);
        joint_pub = nh.advertise<sensor_msgs::JointState>(ns + "/RosHand_JointState", 100);

        FingerActionServer finger_move_server(nh, "HandCommand", boost::bind(&execute_hand_server, _1, &finger_move_server), false);
        finger_move_server.start();


	ros::Rate loop_rate(50);

	while (ros::ok()) 
	{

            try
            {

               hand.read_data();

               pub.publish(hand.hand_data); 
               joint_pub.publish(hand.jointstate);

               ros::spinOnce();
	       loop_rate.sleep();
 


            }
            catch(const std::exception& e)
            {
		ROS_ERROR("//////////////ERROR/////////////.");


            }

	}

	return 0;
}


