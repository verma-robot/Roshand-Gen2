#include "roshand_driver.h"

uint8_t max_close_step_mag = 16;

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

           //写入阈值
            count = 0;
            while(hand.SET_SENSOR_THRESHOLD == false && count < 5)
            {

                hand.sensor_threshold = demo_goal -> sensor_threshold;
                hand.set_sensor_threshold();
                count++;
            }

            count = 0;
            ros::Rate r(20); 
            while(hand.FingerCloseWithSensor == false && count < 5000)
            {
                int grasp_read_count = 0;
                
                while(hand.READ_DATA == true)hand.READ_DATA = false;
                grasp_read_count = 0;
                while(hand.READ_DATA == false && grasp_read_count < 5)
                {
           
                    hand.read_data(); 
                    grasp_read_count++; 

                }
                if(hand.READ_DATA == true)
                {

                    pub.publish(hand.hand_data); 
                    joint_pub.publish(hand.jointstate);    

                }

                while(hand.READ_DATA == true)hand.READ_DATA = false;

                hand.close_with_sensor(close_step_mag);

                feedback.state = hand.FingerCloseWithSensor;
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
            while(hand.SET_SENSOR_THRESHOLD == true)hand.SET_SENSOR_THRESHOLD = false;
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

            while(hand.FingerCloseWithOutSensor == false && count < 5000)
            {

                int grasp_read_count = 0;
                while(hand.READ_DATA == true)hand.READ_DATA = false;
                grasp_read_count = 0;
                while(hand.READ_DATA == false && grasp_read_count < 5)
                {   
                    hand.read_data(); 
                    grasp_read_count++; 
                }
                if(hand.READ_DATA == true)
                {
                    pub.publish(hand.hand_data); 
                    joint_pub.publish(hand.jointstate);         

                }

                while(hand.READ_DATA == true)hand.READ_DATA = false;

                float distance;
                distance = demo_goal -> target_finger_distance;
 
                hand.close_without_sensor(distance);

                feedback.state = hand.FingerCloseWithOutSensor;
                as ->  publishFeedback(feedback);

                count += 1;
                r.sleep();
            }

            while(result_.finish == false)result_.finish = hand.FingerCloseWithOutSensor;
            if(hand.FingerCloseWithOutSensor)
            {
                result_.finish = hand.FingerCloseWithOutSensor;
                as -> setSucceeded(result_); 
            }
            else
            {
                result_.finish = hand.FingerCloseWithOutSensor;
                as -> setAborted(result_); 

            }
            while(hand.FingerCloseWithOutSensor == true)hand.FingerCloseWithOutSensor = false;
            while(gripper_on_action == true)gripper_on_action = false;
        }
    }
    while(gripper_on_action == true)gripper_on_action = false;
}



bool calibrate_senser( roshand_gen2_msgs::SensorCalibrate::Request& req,  roshand_gen2_msgs::SensorCalibrate::Response& res)
{   
    int command_account = 0;

    while(hand.CALIBRATE_SENSOR_FINESHED == false &&  command_account < 30 )
    {
        hand.calibrate_data(); 
        command_account++ ;
    }

    if(hand.CALIBRATE_SENSOR_FINESHED == true)
    {
        ROS_INFO("SENSOR CALIBRATE FINISHED............"); 
        while(res.CALIBRATE_FINISHED == false)res.CALIBRATE_FINISHED = hand.CALIBRATE_SENSOR_FINESHED ;
    }
    else 
    {
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

    int sensor_threshold;

    nh.getParam("port_name", port_name);
    nh.getParam("port_rate", port_rate);

    nh.getParam("sensor_bias", sensor_bias);

    nh.getParam("sensor_threshold", sensor_threshold);


    nh.getParam("ns", ns);
	 
    hand.init(port_name, port_rate);

    while(hand.SET_SENSOR_BIAS == true)hand.SET_SENSOR_BIAS = false;
    retry_count = 0;
    while(hand.SET_SENSOR_BIAS == false && retry_count < 10)
    {
        hand.set_sensor_bias(sensor_bias); 
        retry_count++; 
    }
    if(hand.SET_SENSOR_BIAS == true)ROS_INFO("Sensor bias set succeed.......");
    else ROS_ERROR("Sensor bias set faild, please restart........");
    while(hand.SET_SENSOR_BIAS == true)hand.SET_SENSOR_BIAS = false;


    hand.sensor_threshold = (uint8_t)sensor_threshold;
   
    while(hand.SET_SENSOR_THRESHOLD == true)hand.SET_SENSOR_THRESHOLD = false;
    retry_count = 0;
    while(hand.SET_SENSOR_THRESHOLD == false && retry_count < 10)
    {
        hand.set_sensor_threshold(); 
        retry_count++; 
    }
    if(hand.SET_SENSOR_THRESHOLD == true)ROS_INFO("Sensor threadhold Set succeed......");
    else ROS_ERROR("Sensor threadhold Set faild, please restart......");    
    while(hand.SET_SENSOR_THRESHOLD == true)hand.SET_SENSOR_THRESHOLD = false;


    while(hand.FingerOpen == true)hand.FingerOpen = false;
    retry_count = 0;
    while(hand.FingerOpen == false && retry_count < 5000)
    {
        hand.open_gripper(); 
        retry_count++;
    }//张开手抓
    if(hand.FingerOpen == true)ROS_INFO("Gripper opend succeed......");
    else ROS_ERROR("Gripper open failed......");
    while(hand.FingerOpen == true)hand.FingerOpen = false;

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
                    hand.read_data();
                    retry_count++;
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


