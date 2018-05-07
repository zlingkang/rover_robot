
#include <stdio.h>
#include "hardwareserial.h"
#include "gy85.h"
#include "motor.h"
#include "encoder.h"
#include "led.h"
#include "battery.h"
#include "servo.h"
//#include "intencoder.h"
#include "sonar.h"

#include <ros.h>
#include <ros/time.h>
#include <time.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define DEBUG_RATE 120

const float BASE_WIDTH = 0.16; // meter
int Motor::counts_per_rev = 390;
float Motor::wheel_diameter = 0.06; //meter
int Motor::max_rpm = 1.5 * 60 / (3.14 * 0.06); //maximum speed 1.5m/s 
float Motor::Kp = 0.4;
float Motor::Ki = 0;
float Motor::Kd = 0;

Led led;
void led_cb(const std_msgs::Float64& cmd_msg){
	static bool stat_led = true;
	if(stat_led){
		led.on_off(ON);
		stat_led = false;
	}else{
		led.on_off(OFF);
		stat_led = true;
	}
}

int motor1_pwm = 0;
int motor2_pwm = 0;
float motor1_speed = 0;
float motor2_speed = 0;

void motor1_cb(const std_msgs::Float64& cmd_msg)
{
    motor1_speed = cmd_msg.data; //meters per second
}
void motor2_cb(const std_msgs::Float64& cmd_msg)
{
    motor2_speed = cmd_msg.data; //meters per second
}

void kp_cb(const std_msgs::Float64& cmd_msg)
{
    Motor::Kp = cmd_msg.data;
}
void ki_cb(const std_msgs::Float64& cmd_msg)
{
    Motor::Ki = cmd_msg.data;
}
void kd_cb(const std_msgs::Float64& cmd_msg)
{
    Motor::Kd = cmd_msg.data;
}

int main(void) 
{
	SystemInit();
	initialise();

    float length_per_rev = 3.14 * Motor::wheel_diameter;
	uint32_t previous_debug_time = 0;
	uint32_t previous_trans_time = 0;
	char buffer[50];
	std_msgs::String str_msg;
    std_msgs::Float64 motor1_speed_msg;
    std_msgs::Float64 motor2_speed_msg;
    ros::NodeHandle  nh; 
	nh.initNode();
    ros::Publisher battery("battery", &str_msg);
	ros::Publisher motor1_pub("motor1_measured_speed", &motor1_speed_msg);
	ros::Publisher motor2_pub("motor2_measured_speed", &motor2_speed_msg);
/*
    tf::TransformBroadcaster odom_broadcaster;
    nav_msgs::Odometry odom;
    ros::Publisher odom_pub("odom", &odom);
*/
	ros::Subscriber<std_msgs::Float64> sub("led", led_cb);
	ros::Subscriber<std_msgs::Float64> motor1_sub("motor1", motor1_cb);
	ros::Subscriber<std_msgs::Float64> motor2_sub("motor2", motor2_cb);
    ros::Subscriber<std_msgs::Float64> kp_sub("kp", kp_cb);
	ros::Subscriber<std_msgs::Float64> ki_sub("ki", ki_cb);
	ros::Subscriber<std_msgs::Float64> kd_sub("kd", kd_cb);
  
    nh.advertise(battery);
    //nh.advertise(odom_pub);
    nh.advertise(motor1_pub);
    nh.advertise(motor2_pub);
    nh.subscribe(sub);
    nh.subscribe(kp_sub);
    nh.subscribe(ki_sub);
    nh.subscribe(kd_sub);
    nh.subscribe(motor1_sub);
    nh.subscribe(motor2_sub);

	Battery bat(25, 10.6, 12.6);
	bat.init();
	led.init();
    //Serial.print("init motor \r\n");
	Motor motor1(MOTOR1, 254, 575);
	Motor motor2(MOTOR2, 254, 575);
	motor1.init();
	motor2.init();
	motor1.spin(0);
	motor2.spin(0);
	Encoder encoder1(ENCODER1, 0xffff, 0);
	Encoder encoder2(ENCODER2, 0xffff, 0);
	encoder1.init();
	encoder2.init();

    Vector gyros, acc, mag; 
	Gy85 imu;
	imu.init();

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0.0;
    double vy = 0.0;
    double vth = 0.0;

    //ros::Time current_time;
    int trans_count = 0;
	while(1){
		if ((millis() - previous_debug_time) >= (1000 / DEBUG_RATE)) {
			//Serial.print("encoder1 count : %ld\r\n", encoder1.read());
			//Serial.print("encoder2 count : %ld\r\n", encoder2.read());
	    	sprintf (buffer, "Current borad volt is : %f", bat.get_volt());
			str_msg.data = buffer;
			//battery.publish( &str_msg );
            
            // read and publish encoder reading            
            motor1.calculate_rpm(encoder2.read());
            motor2.calculate_rpm(encoder1.read());
            // calculate required pwm
            motor1.required_rpm = motor1_speed * 60 / length_per_rev;
            motor2.required_rpm = motor2_speed * 60 / length_per_rev;
            motor1_pwm = motor1.calculate_pwm();
            motor2_pwm = motor2.calculate_pwm();
            motor1.spin(-motor1_pwm);// (-(int)motor1_speed);  //(-motor1_pwm);
            motor2.spin(-motor2_pwm);// (-(int)motor1_speed);  //(-motor1_pwm);
            //pwm1_msg.data = motor1_pwm;
            //pwm1_pub.publish( &pwm1_msg );
            

			if(imu.check_gyroscope()){
				gyros = imu.measure_gyroscope();
				//Serial.print("gyros x: %f, y : %f, z: %f \r\n", gyros.x, gyros.y, gyros.z);
			}
			if(imu.check_accelerometer()){
				acc = imu.measure_acceleration();
				//Serial.print("acceleration x: %f, y : %f, z: %f \r\n", acc.x, acc.y, acc.z);
			}
			if(imu.check_magnetometer()){
				mag = imu.measure_magnetometer();
				//Serial.print("magnetometer x: %f, y : %f, z: %f \r\n", mag.x, mag.y, mag.z);
			}
            
            trans_count ++;
            if(trans_count >= 0){
            
                trans_count = 0;
                motor1_speed_msg.data = motor1.current_rpm * length_per_rev/60.0;
                motor2_speed_msg.data = motor2.current_rpm * length_per_rev/60.0;
                motor1_pub.publish(&motor1_speed_msg);
                motor2_pub.publish(&motor2_speed_msg);
                /* 
                double v_linear = (motor1.current_rpm + motor2.current_rpm) * length_per_rev/60.0;
                double v_th = (motor1.current_rpm - motor2.current_rpm) * length_per_rev/60.0 / BASE_WIDTH;
                vx = v_linear;
                vy = 0;
                vth = v_th; 

                double dt = (double)(millis() - previous_trans_time) / 1000.0;
                
                double delta_th = vth * dt;
                double delta_x = (vx * cos(th + delta_th/2) - vy*sin(th + delta_th/2)) * dt;
                double delta_y = (vx * sin(th + delta_th/2) - vy*cos(th + delta_th/2)) * dt;

                x += delta_x;
                y += delta_y;
                th += delta_th;

                //current_time = ros::Time::now();

                geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(th);
                
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = nh.now();//current_time;
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "base_link";
                odom_trans.transform.translation.x = x;
                odom_trans.transform.translation.y = y;
                odom_trans.transform.translation.z = 0.0;
                odom_trans.transform.rotation = odom_quat;
                odom_broadcaster.sendTransform(odom_trans);

                odom.header.stamp = nh.now();
                odom.header.frame_id = "odom";
                odom.pose.pose.position.x = x;
                odom.pose.pose.position.y = y;
                odom.pose.pose.position.z = 0.0;
                odom.pose.pose.orientation = odom_quat;
                odom.child_frame_id = "base_link";
                odom.twist.twist.linear.x = vx;
                odom.twist.twist.linear.y = vy;
                odom.twist.twist.angular.z = vth;
                odom_pub.publish(&odom);
                */
                previous_trans_time = millis(); 

            }

            previous_debug_time = millis();    
		}
		nh.spinOnce();
	}


}


