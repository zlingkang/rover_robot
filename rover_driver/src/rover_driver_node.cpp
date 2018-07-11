#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

const double CTRL_SCALE = 4.0;

class RoverRobot
{
    public:
        static RoverRobot *getInstance()
        {
            if(!s_instance_)
                s_instance_ = new RoverRobot;
            return s_instance_;
        }
       
        RoverRobot(const RoverRobot&)=delete;
        RoverRobot& operator=(const RoverRobot&)=delete;

        void motor1SpeedCB(const std_msgs::Float64::ConstPtr& msg);
        void motor2SpeedCB(const std_msgs::Float64::ConstPtr& msg);
        void velCmdCB(const geometry_msgs::Twist& msg);

        void odomPub();
    private:
        RoverRobot();
        static RoverRobot *s_instance_;

        ros::NodeHandle *node_;

        ros::Subscriber motor1_speed_sub_;
        ros::Subscriber motor2_speed_sub_;
        ros::Publisher odom_pub_;
        tf::TransformBroadcaster odom_broadcaster;
        float motor1_measured_speed_;
        float motor2_measured_speed_;
        ros::Time current_time_;
        ros::Time last_time_;
        double x_;
        double y_;
        double th_;
        double vx_;
        double vy_;
        double vth_;

        ros::Subscriber velocity_cmd_sub_;
        ros::Publisher motor1_speed_pub_;
        ros::Publisher motor2_speed_pub_;
        geometry_msgs::Twist vel_cmd_;

        float BASE_WIDTH_;

};

void RoverRobot::motor1SpeedCB(const std_msgs::Float64::ConstPtr& msg)
{
    motor1_measured_speed_ = msg->data/CTRL_SCALE;
}
void RoverRobot::motor2SpeedCB(const std_msgs::Float64::ConstPtr& msg)
{
    motor2_measured_speed_ = msg->data/CTRL_SCALE;
}

void RoverRobot::velCmdCB(const geometry_msgs::Twist& msg)
{
    geometry_msgs::Twist trgt_twist = msg;
    double v_linear = trgt_twist.linear.x * CTRL_SCALE;
    double vth = trgt_twist.angular.z * CTRL_SCALE;

	ROS_INFO_STREAM("[ROV] vel_cmd: " << v_linear << " " << vth);
	
    std_msgs::Float64 motor1_speed;
    std_msgs::Float64 motor2_speed;
    motor1_speed.data = vth*BASE_WIDTH_/2.0 + v_linear;
    motor2_speed.data = 2*v_linear - motor1_speed.data;
    motor1_speed_pub_.publish(motor1_speed);
    motor2_speed_pub_.publish(motor2_speed);
}

void RoverRobot::odomPub()
{
    current_time_ = ros::Time::now();
    double v_linear = (motor1_measured_speed_ + motor2_measured_speed_) / 2.0;
    double v_th = (motor1_measured_speed_ - motor2_measured_speed_) / BASE_WIDTH_;
    vx_ = v_linear;
    vy_ = 0;
    vth_ = v_th; 

    double dt = (current_time_ - last_time_).toSec();
    /* 
    ROS_INFO_STREAM("vx_ : " << vx_);
    ROS_INFO_STREAM("vy_ : " << vy_);
    ROS_INFO_STREAM("vth_ : " << vth_);
    ROS_INFO_STREAM("dt : " << dt);
  */ 
    double delta_th = vth_ * dt;
    double delta_x = (vx_ * cos(th_ + delta_th/2) - vy_*sin(th_ + delta_th/2)) * dt;
    double delta_y = (vx_ * sin(th_ + delta_th/2) - vy_*cos(th_ + delta_th/2)) * dt;

    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;
/*
    ROS_INFO_STREAM("x_ : " << x_);
    ROS_INFO_STREAM("y_ : " << y_);
    ROS_INFO_STREAM("th_ : " << th_);
*/

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);
    
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);
    
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time_;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx_;
    odom.twist.twist.linear.y = vy_;
    odom.twist.twist.angular.z = vth_;
    odom_pub_.publish(odom);

    last_time_ = current_time_;
}

template<class T>
inline void get_param(const ros::NodeHandle* nh, const std::string& param_name, T& var, const T default_value)
{
    nh->param(param_name, var, default_value);
    ROS_INFO_STREAM("[ROV] Param " << param_name << " : " << var);
}

RoverRobot::RoverRobot()
{
    node_ = new ros::NodeHandle("~");

    motor1_speed_sub_ = node_->subscribe("/motor1_measured_speed", 100, &RoverRobot::motor1SpeedCB, this);
    motor2_speed_sub_ = node_->subscribe("/motor2_measured_speed", 100, &RoverRobot::motor2SpeedCB, this);
    odom_pub_ = node_->advertise<nav_msgs::Odometry>("/odom", 50);

    velocity_cmd_sub_ = node_->subscribe("/vel_cmd", 100, &RoverRobot::velCmdCB, this);
    motor1_speed_pub_ = node_->advertise<std_msgs::Float64>("/motor1", 50);
    motor2_speed_pub_ = node_->advertise<std_msgs::Float64>("/motor2", 50);

    get_param<float>(node_, "base_width", BASE_WIDTH_, 0.16);

    x_ = 0.0;
    y_ = 0.0;
    th_ = 0.0;
    vx_ = 0.0;
    vy_ = 0.0;
    vth_ = 0.0;
    current_time_ = ros::Time::now();
    last_time_ = ros::Time::now();
}

RoverRobot *RoverRobot::s_instance_ = 0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rover_driver_node");

    RoverRobot* rover_ptr = RoverRobot::getInstance(); 

    ros::Rate r(30);

    while(ros::ok())
    {
        rover_ptr->odomPub();
        ros::spinOnce();
    }
}
