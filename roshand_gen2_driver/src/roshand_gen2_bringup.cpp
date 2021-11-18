#include "roshand_driver.h"

uint8_t max_close_step_mag = 8;

roshand_gen2::roshand_gen2_hardware hand;	

ros::Publisher pub;
ros::Publisher joint_pub;
bool gripper_on_action;

typedef actionlib::SimpleActionServer<roshand_gen2_msgs::HandCommandAction> FingerActionServer;


// move without sensor
void execute_hand_server(const roshand_gen2_msgs::HandCommandGoalConstPtr& demo_goal, FingerActionServer* as)
{


    uint8_t close_step_mag;

    roshand_gen2_msgs::HandCommandFeedback feedback;   
    roshand_gen2_msgs::HandCommandResult result_;    /* 创建一个feedback对象 */    
    while(gripper_on_action == true)gripper_on_action = false;
    while(result_.finish == true)result_.finish = false;
    if(demo_goal -> use_sensor == true)//采用触觉控制
    {
        int count = 0;
        while(gripper_on_action == false)gripper_on_action = true;
        if(gripper_on_action == true)
        {
            close_step_mag =  demo_goal -> close_step_mag;
            if(close_step_mag > max_close_step_mag)close_step_mag = max_close_step_mag;

            while(hand.FingerCloseWithSensor == true)hand.FingerCloseWithSensor = false;
            while(hand.FingerOpen == true)hand.FingerOpen = false;
            while(hand.CALIBRATE_SENSOR_FINESHED == true)hand.CALIBRATE_SENSOR_FINESHED = false;
            while(hand.SET_SENSOR_THREADHOLD == true)hand.SET_SENSOR_THREADHOLD = false;

            count = 0;
            while(hand.FingerOpen == false && count < 50){hand.open_gripper(); count++; }//张开手抓
            count = 0;
            while(hand.CALIBRATE_SENSOR_FINESHED == false && count < 5){hand.calibrate_data(); count++; }//校准手抓     

            count = 0;
            while(hand.SET_SENSOR_THREADHOLD == false && count < 5)
            {

                for(int i = 0; i < 14; i++)hand.sensor_thread_hold[0][i] = demo_goal -> sensor_threadhold;
                for(int i = 0; i < 14; i++)hand.sensor_thread_hold[1][i] = demo_goal -> sensor_threadhold;

                hand.set_sensor_thread_hold();
                count++;
            }

            count = 0;
            ros::Rate r(20); 
            while(hand.FingerCloseWithSensor == false && count < 50)
            {


                int grasp_read_count = 0;
                while(hand.READ_DATA == true)hand.READ_DATA = false;
                grasp_read_count = 0;
                while(hand.READ_DATA == false && grasp_read_count < 5)
                {
           
                     tcflush(STDIN_FILENO, TCIFLUSH);
                     tcflush(STDOUT_FILENO, TCOFLUSH);   
                     hand.read_data(); 
                     grasp_read_count++; 

                }
               // if(hand.READ_DATA == true)
               // {

                     pub.publish(hand.hand_data); 
                     joint_pub.publish(hand.jointstate);    

               // }

                hand.close_with_sensor(close_step_mag);

                feedback.state = hand.hand_data;
                as ->  publishFeedback(feedback);

                count++;
                r.sleep();
            }

            if(hand.FingerCloseWithSensor == true)
            {
                result_.finish = hand.FingerCloseWithSensor;
                as -> setSucceeded(result_); 
            }
            else
            {
                result_.finish = hand.FingerCloseWithSensor;
                as -> setAborted(result_); 

            }
            
            while(hand.FingerCloseWithSensor == true)hand.FingerCloseWithSensor = false;
            while(hand.FingerOpen == true)hand.FingerOpen = false;
            while(hand.CALIBRATE_SENSOR_FINESHED == true)hand.CALIBRATE_SENSOR_FINESHED = false;
            while(hand.SET_SENSOR_THREADHOLD == true)hand.SET_SENSOR_THREADHOLD = false;
            while(gripper_on_action == true)gripper_on_action = false;
        }
    }
    else //采用非触控的模式
    {
        int count = 0;
        while(gripper_on_action == false)gripper_on_action = true;
        if(gripper_on_action == true)
        {
            while(hand.FingerCloseWithOutSensor == true)hand.FingerCloseWithOutSensor = false;
            ros::Rate r(20); 
            while(hand.FingerCloseWithOutSensor == false && count < 50)
            {

                int grasp_read_count = 0;
                while(hand.READ_DATA == true)hand.READ_DATA = false;
                grasp_read_count = 0;
                while(hand.READ_DATA == false && grasp_read_count < 5)
                {
   
                     tcflush(STDIN_FILENO, TCIFLUSH);
                     tcflush(STDOUT_FILENO, TCOFLUSH);           
                     hand.read_data(); 
                     grasp_read_count++; 

                }
                //if(hand.READ_DATA == true)
               // {
                     pub.publish(hand.hand_data); 
                     joint_pub.publish(hand.jointstate);         

                //}

                float distance;
                distance = demo_goal -> goal_finger_distance;
 
                hand.close_without_sensor(distance);

                feedback.state = hand.hand_data;
                as ->  publishFeedback(feedback);

                while(hand.READ_DATA == true)hand.READ_DATA = false;
                count += 1;
                r.sleep();
            }

            while(result_.finish == false)result_.finish = hand.FingerCloseWithOutSensor;
            as -> setSucceeded(result_); 
            while(hand.FingerCloseWithOutSensor == true)hand.FingerCloseWithOutSensor = false;
            while(gripper_on_action == true)gripper_on_action = false;
        }
    }
    while(gripper_on_action == true)gripper_on_action = false;
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
        int retry_count = 0;

        std::vector<int> all_sensor_thread_hold[2];

        nh.getParam("port_name", port_name);
        nh.getParam("port_rate", port_rate);

        nh.getParam("sensor_bias", sensor_bias);

        nh.getParam("left_fingertips_thread_hold", all_sensor_thread_hold[0]);
        nh.getParam("right_fingertips_thread_hold", all_sensor_thread_hold[1]);


        nh.getParam("ns", ns);
	 
        hand.init(port_name, port_rate);

        tcflush(STDIN_FILENO, TCIFLUSH);
        tcflush(STDOUT_FILENO, TCOFLUSH);
        usleep(10000);

        while(hand.SET_SENSOR_BIAS == true)hand.SET_SENSOR_BIAS = false;
        while(hand.SET_SENSOR_BIAS == false && retry_count < 10){hand.set_sensor_bias(sensor_bias); retry_count++; }
        if(hand.SET_SENSOR_BIAS == true)ROS_INFO("Sensor bias set succeed....................");
        else ROS_ERROR("Sensor bias set faild, please restart........");
        while(hand.SET_SENSOR_BIAS == true)hand.SET_SENSOR_BIAS = false;

        hand.sensor_thread_hold[0] = all_sensor_thread_hold[0];
        hand.sensor_thread_hold[1] = all_sensor_thread_hold[1];
   
        retry_count = 0;
        while(hand.SET_SENSOR_THREADHOLD == true)hand.SET_SENSOR_THREADHOLD = false;
        while(hand.SET_SENSOR_THREADHOLD == false && retry_count < 10){hand.set_sensor_thread_hold(); retry_count++; }
        if(hand.SET_SENSOR_THREADHOLD == true)ROS_INFO("Sensor threadhold Set succeed......");
        else ROS_ERROR("Sensor threadhold Set faild, please restart......");
        while(hand.SET_SENSOR_THREADHOLD == true)hand.SET_SENSOR_THREADHOLD = false;

        retry_count = 0;
        while(hand.FingerOpen == true)hand.FingerOpen = false;
        while(hand.FingerOpen == false && retry_count < 50){hand.open_gripper(); retry_count++;}//张开手抓
        if(hand.FingerOpen == true)ROS_INFO("Gripper opend succeed......");
        else ROS_ERROR("Gripper open failed......");
        while(hand.FingerOpen == true)hand.FingerOpen = false;

        retry_count = 0;
        while(hand.CALIBRATE_SENSOR_FINESHED == true)hand.CALIBRATE_SENSOR_FINESHED = false;
        while(hand.CALIBRATE_SENSOR_FINESHED == false && retry_count < 10){hand.calibrate_data(); retry_count++; }
        if(hand.CALIBRATE_SENSOR_FINESHED == true)ROS_INFO("Sensor calibrated succeed......");
        else ROS_ERROR("Sensor calibrated failed");
//std::cout << hand.CALIBRATE_SENSOR_FINESHED << std::endl;
        while(hand.CALIBRATE_SENSOR_FINESHED == true)hand.CALIBRATE_SENSOR_FINESHED=false;

        while(gripper_on_action == true)gripper_on_action = false;

        ros::ServiceServer calibrate_sensor = nh.advertiseService(ns + "/calibrate_sensor_service", calibrate_senser);

        pub = nh.advertise<roshand_gen2_msgs::Hand>(ns + "/RosHand_Data", 100);
        joint_pub = nh.advertise<sensor_msgs::JointState>(ns + "/RosHand_JointState", 100);

        FingerActionServer finger_move_server(nh, "HandCommand", boost::bind(&execute_hand_server, _1, &finger_move_server), false);
        finger_move_server.start();


	ros::Rate loop_rate(20);

	while (ros::ok()) 
	{

            try
            {
               if(gripper_on_action == false)
               {
                   retry_count = 0;
                   while(hand.READ_DATA == true)hand.READ_DATA = false;
                   while(hand.READ_DATA == false && retry_count < 5)
                   {


                       tcflush(STDIN_FILENO, TCIFLUSH);
                       tcflush(STDOUT_FILENO, TCOFLUSH);

                       hand.read_data();
                       //usleep(20000);
                       retry_count++;
                       //co++;

                   }          
                   if(hand.READ_DATA == true)
                   {
                      pub.publish(hand.hand_data); 
                      joint_pub.publish(hand.jointstate);
                   }
                   while(hand.READ_DATA == true)hand.READ_DATA = false;
                  
               }
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


