#include <ros/ros.h>
#include <deadreckoning/enc.h>
#include <geometry_msgs/PoseStamped.h>
#include <accel_decel/result.h>
#include <nemcon/pid_param.h>

#include <signal.h>

#include <nemcon/motor.h>

#include <stdio.h>
#include <stdlib.h>
#include <cmath>	// http://faithandbrave.hateblo.jp/entry/2016/12/12/181115

using namespace std;

float speed_X = 0.0f, speed_Y = 0.0f, speed = 0.0f;
int front;	//旋回：-1，前：1，右：2，後：3，左：4
int pattern; //0：加減速，1：等速

float imu_P;
float imu_I;
float imu_D;

float enc_P;
float enc_I;
float enc_D;

float v_P;
float v_I;
float v_D;

float vs_P;
float vs_I;
float vs_D;

float speedFR = 0.0f, speedRL = 0.0f, speedFL = 0.0f, speedRR = 0.0f;
float turn_imu = 0.0f, turn_enc_x = 0.0f, turn_enc_y = 0.0f, turn_lrf = 0.0f;
float enc_vx = 0.0f, enc_vy = 0.0f;
float tar_x = 0.0f, tar_y = 0.0f;
float enc_x = 0.0f;

ros::Time current_time , last_time;
double dt = 0.0;

ros::Publisher pub;
nemcon::motor msg_m;

float clamp(float input, float min, float max)
{
	float output = 0.00000f;
	if(input <= min)
	{
		output = min;
	}
	if(min < input && input < max)
	{
		output = input;
	}
	if(input >= max)
	{
		output = max;
	}
	if(max > 0 && 1 <= input && input < 3)
		{
			output = 2;
		}
	if(min < 0 && -3 < input && input <= -1)
	{
		output = -2;
	}
  if(input <= 0.0)
  {
    output = 8080;
  }
	return output;
}

void param_cb(const nemcon::pid_param& msg)
{
	speed = msg.speed;
	pattern = msg.pattern;
	front = msg.front;
	tar_x = 0.0f;
	tar_y = 0.0f;
}

void enc_cb(const deadreckoning::enc& msg)
{
  enc_vx = msg.speed_X;
  enc_vy = msg.speed_Y;
}

void pid_lrf(const geometry_msgs::PoseStamped& msg)
{
	float lasterror = 0, integral = 0, error = 0;

	error = msg.pose.orientation.z - /*0.00000f*/0.0261769406497;

	integral += (error + lasterror) / 2.0 * dt;

	turn_lrf = imu_P * error + imu_I * integral + imu_D * (error - lasterror) / dt;

	lasterror = error;
}

void pid_pose(const geometry_msgs::PoseStamped& msg)
{
	float lasterror_imu = 0, integral_imu = 0, error_imu = 0;
	float lasterror_x = 0, lasterror_y = 0, integral_x = 0, integral_y = 0, error_x = 0, error_y = 0;

	error_imu = msg.pose.orientation.z - 0.00000f;
	error_x = msg.pose.position.x - tar_x;
	error_y = msg.pose.position.y - tar_y;

	integral_imu += (error_imu + lasterror_imu) / 2.0 * dt;
	integral_x += (error_x + lasterror_x) / 2.0 * dt;
	integral_y += (error_y + lasterror_y) / 2.0 * dt;

	turn_imu = imu_P * error_imu + imu_I * integral_imu + imu_D * (error_imu - lasterror_imu) / dt;
	turn_enc_x = enc_P * error_x + enc_I * integral_x + enc_D * (error_x - lasterror_x) / dt;
	turn_enc_y = enc_P * error_y + enc_I * integral_y + enc_D * (error_y - lasterror_y) / dt;

	lasterror_imu = error_imu;
	lasterror_x = error_x;
	lasterror_y = error_y;
}

void pid_v(const accel_decel::result& msg)
{
	float lasterror_x = 0, lasterror_y = 0, integral_x = 0, integral_y = 0, error_x = 0, error_y = 0;

	error_x = msg.V - abs(enc_vx);
	error_y = msg.V - abs(enc_vy);

  printf("%f\n", abs(enc_vy));

	integral_x += (error_x + lasterror_x) / 2.0 * dt;
	integral_y += (error_y + lasterror_y) / 2.0 * dt;

	if(!msg.Vmax)	//加減速用
	{
		speed_X= v_P * error_x + v_I * integral_x + v_D * (error_x - lasterror_x) / dt;
		speed_Y = v_P * error_y + v_I * integral_y + v_D * (error_y - lasterror_y) / dt;
	}
	else			//等速直進用
	{
		speed_X= vs_P * error_x + vs_I * integral_x + vs_D * (error_x - lasterror_x) / dt;
		speed_Y = vs_P * error_y + vs_I * integral_y + vs_D * (error_y - lasterror_y) / dt;
	}
  //printf("%f\n", speed_Y);

	lasterror_x = error_x;
	lasterror_y = error_y;

}

void mySigintHandler(int sig)
{
	msg_m.motor_FR = 0;
	msg_m.motor_FL = 0;
	msg_m.motor_RR = 0;
	msg_m.motor_RL = 0;
	pub.publish(msg_m);
	ros::shutdown();
}

/*****************************************************************************************************/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pid_param", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	ros::Rate loop_rate(40);
	ros::NodeHandle local_nh("~");

	/*****************************************************************************/

	if(!local_nh.hasParam("front"))
	{
		ROS_INFO("Parameter front is not defind. Now, it is set default value.");
		local_nh.setParam("front", 1);
	}
	if(!local_nh.getParam("front", front))
	{
		ROS_ERROR("parameter front is invalid.");
		return -1;
	}
	ROS_INFO("front: %d", front);
  
  /*****************************************************************************/

	if(!local_nh.hasParam("pattern"))
	{
		ROS_INFO("Parameter pattern is not defind. Now, it is set default value.");
		local_nh.setParam("pattern", 1);
	}
	if(!local_nh.getParam("pattern", pattern))
	{
		ROS_ERROR("parameter pattern is invalid.");
		return -1;
	}
	ROS_INFO("pattern: %d", pattern);
  
  /*****************************************************************************/

	if(!local_nh.hasParam("imu_P"))
	{
		ROS_INFO("Parameter imu_P is not defind. Now, it is set default value.");
		local_nh.setParam("imu_P", 20);
	}
	if(!local_nh.getParam("imu_P", imu_P))
	{
		ROS_ERROR("parameter front is invalid.");
		return -1;
	}
	ROS_INFO("imu_P: %f", imu_P);

	if(!local_nh.hasParam("imu_I"))
	{
		ROS_INFO("Parameter imu_I is not defind. Now, it is set default value.");
		local_nh.setParam("imu_I", 2.00);
	}
	if(!local_nh.getParam("imu_I", imu_I))
	{
		ROS_ERROR("parameter front is invalid.");
		return -1;
	}
	ROS_INFO("imu_I: %f", imu_I);

	if(!local_nh.hasParam("imu_D"))
	{
		ROS_INFO("Parameter imu_D is not defind. Now, it is set default value.");
		local_nh.setParam("imu_D", 0.05);
	}
	if(!local_nh.getParam("imu_D", imu_D))
	{
		ROS_ERROR("parameter front is invalid.");
		return -1;
	}
	ROS_INFO("imu_D: %f", imu_D);

	/************************************************************************/

	if(!local_nh.hasParam("enc_P"))
	{
		ROS_INFO("Parameter enc_P is not defind. Now, it is set default value.");
		local_nh.setParam("enc_P", 0);
	}
	if(!local_nh.getParam("enc_P", enc_P))
	{
		ROS_ERROR("parameter front is invalid.");
		return -1;
	}
	ROS_INFO("enc_P: %f", enc_P);

	if(!local_nh.hasParam("enc_I"))
	{
		ROS_INFO("Parameter enc_I is not defind. Now, it is set default value.");
		local_nh.setParam("enc_I", 0);
	}
	if(!local_nh.getParam("enc_I", enc_I))
	{
		ROS_ERROR("parameter front is invalid.");
		return -1;
	}
	ROS_INFO("enc_I: %f", enc_I);

	if(!local_nh.hasParam("enc_D"))
	{
		ROS_INFO("Parameter enc_D is not defind. Now, it is set default value.");
		local_nh.setParam("enc_D", 0);
	}
	if(!local_nh.getParam("enc_D", enc_D))
	{
		ROS_ERROR("parameter front is invalid.");
		return -1;
	}
	ROS_INFO("enc_D: %f", enc_D);

	/************************************************************************/

	if(!local_nh.hasParam("v_P"))
	{
		ROS_INFO("Parameter v_P is not defind. Now, it is set default value.");
		local_nh.setParam("v_P", 0);
	}
	if(!local_nh.getParam("v_P", v_P))
	{
		ROS_ERROR("parameter front is invalid.");
		return -1;
	}
	ROS_INFO("v_P: %f", v_P);

	if(!local_nh.hasParam("v_I"))
	{
		ROS_INFO("Parameter v_I is not defind. Now, it is set default value.");
		local_nh.setParam("v_I", 0);
	}
	if(!local_nh.getParam("v_I", v_I))
	{
		ROS_ERROR("parameter front is invalid.");
		return -1;
	}
	ROS_INFO("v_I: %f", v_I);

	if(!local_nh.hasParam("v_D"))
	{
		ROS_INFO("Parameter v_D is not defind. Now, it is set default value.");
		local_nh.setParam("v_D", 0);
	}
	if(!local_nh.getParam("v_D", v_D))
	{
		ROS_ERROR("parameter front is invalid.");
		return -1;
	}
	ROS_INFO("v_D: %f", v_D);

	/************************************************************************/

	if(!local_nh.hasParam("vs_P"))
	{
		ROS_INFO("Parameter vs_P is not defind. Now, it is set default value.");
		local_nh.setParam("vs_P", 0);
	}
	if(!local_nh.getParam("vs_P", vs_P))
	{
		ROS_ERROR("parameter front is invalid.");
		return -1;
	}
	ROS_INFO("vs_P: %f", vs_P);

	if(!local_nh.hasParam("vs_I"))
	{
		ROS_INFO("Parameter vs_I is not defind. Now, it is set default value.");
		local_nh.setParam("vs_I", 0);
	}
	if(!local_nh.getParam("vs_I", vs_I))
	{
		ROS_ERROR("parameter front is invalid.");
		return -1;
	}
	ROS_INFO("vs_I: %f", vs_I);

	if(!local_nh.hasParam("vs_D"))
	{
		ROS_INFO("Parameter vs_D is not defind. Now, it is set default value.");
		local_nh.setParam("vs_D", 0);
	}
	if(!local_nh.getParam("vs_D", vs_D))
	{
		ROS_ERROR("parameter front is invalid.");
		return -1;
	}
	ROS_INFO("vs_D: %f", vs_D);

	/************************************************************************/

	ros::Subscriber sub_dis = nh.subscribe("/pid_param", 1000, param_cb);
	ros::Subscriber sub_enc = nh.subscribe("/robot/pose", 1000, pid_pose);
	ros::Subscriber sub_accel = nh.subscribe("/accel_decel/result", 1000, pid_v);
	ros::Subscriber sub_speed = nh.subscribe("/enc", 1000, enc_cb);
	ros::Subscriber sub_lrf = nh.subscribe("/lrf_pose", 1000, pid_lrf);

	pub = nh.advertise<nemcon::motor>("motor", 100);



	while(ros::ok())
	{
		signal(SIGINT, mySigintHandler);
		current_time = ros::Time::now();
		dt = (current_time - last_time).toSec();

		if(pattern == 0) //加減速
		{
			switch(front)
			{
				case 1:	//前
					speedFR = clamp(nearbyint( speed_Y - turn_imu + turn_enc_x), 0, 60);
					speedFL = clamp(nearbyint( speed_Y + turn_imu - turn_enc_x), 0, 60);
					speedRL = clamp(nearbyint( speed_Y + turn_imu + turn_enc_x), 0, 60);
					speedRR = clamp(nearbyint( speed_Y - turn_imu - turn_enc_x), 0, 60);
					break;

				case 2:	//右
					speedFR = clamp(nearbyint(-(speed_X + turn_imu + turn_enc_y )), -20, 0);
					speedFL = clamp(nearbyint( speed_X + turn_imu - turn_enc_y), 0, 20);
					speedRL = clamp(nearbyint(-(speed_X - turn_imu + turn_enc_y)), -20, 0);
					speedRR = clamp(nearbyint( speed_X - turn_imu - turn_enc_y), 0, 20);
					break;

				case 3:	//後
					speedFR = clamp(nearbyint( -(speed_Y + turn_imu + turn_enc_x)), -20, 0);
					speedFL = clamp(nearbyint( -(speed_Y - turn_imu - turn_enc_x)), -20, 0);
					speedRL = clamp(nearbyint( -(speed_Y - turn_imu + turn_enc_x)), -20, 0);
					speedRR = clamp(nearbyint( -(speed_Y + turn_imu - turn_enc_x)), -20, 0);
					break;

				case 4:	//左
					speedFR = clamp(nearbyint( speed_X - turn_imu - turn_enc_y ), 0, 20);
					speedFL = clamp(nearbyint( -(speed_X - turn_imu + turn_enc_y)), -20, 0);
					speedRL = clamp(nearbyint( speed_X + turn_imu - turn_enc_y), 0, 20);
					speedRR = clamp(nearbyint( -(speed_X + turn_imu + turn_enc_y)), -20, 0);
					break;

				default:
					speedFR = 0;
					speedFL = 0;
					speedRL = 0;
					speedRR = 0;
					break;
			}
		}

		if(pattern == 1)	//等速
		{
			switch(front)
			{
				case 1:	//前
					speedFR = clamp(nearbyint( speed_Y - turn_lrf), 0, 20);
					speedFL = clamp(nearbyint( speed_Y + turn_lrf), 0, 20);
					speedRL = clamp(nearbyint( speed_Y + turn_lrf), 0, 20);
					speedRR = clamp(nearbyint( speed_Y - turn_lrf), 0, 20);
					break;

				case 2:	//右
					speedFR = clamp(nearbyint(-(speed_X + turn_lrf)), -20, 0);
					speedFL = clamp(nearbyint( speed_X + turn_lrf), 0, 20);
					speedRL = clamp(nearbyint(-(speed_X - turn_lrf)), -20, 0);
					speedRR = clamp(nearbyint( speed_X - turn_lrf), 0, 20);
					break;

				case 3:	//後
					speedFR = clamp(nearbyint( -(speed_Y + turn_lrf)), -20, 0);
					speedFL = clamp(nearbyint( -(speed_Y - turn_lrf)), -20, 0);
					speedRL = clamp(nearbyint( -(speed_Y - turn_lrf)), -20, 0);
					speedRR = clamp(nearbyint( -(speed_Y + turn_lrf)), -20, 0);
					break;

				case 4:	//左
					speedFR = clamp(nearbyint( speed_X - turn_lrf), 0, 20);
					speedFL = clamp(nearbyint( -(speed_X - turn_lrf)), -20, 0);
					speedRL = clamp(nearbyint( speed_X + turn_lrf), 0, 20);
					speedRR = clamp(nearbyint( -(speed_X + turn_lrf)), -20, 0);
					break;

				default:
					speedFR = 0;
					speedFL = 0;
					speedRL = 0;
					speedRR = 0;
					break;
			}
		}
		if(pattern == 3)
		{
			speedFR = 8080;
			speedFL = 8080;
			speedRL = 8080;
			speedRR = 8080;
		}

		msg_m.motor_FR = speedFR;
		msg_m.motor_FL = speedFL;
		msg_m.motor_RR = speedRR;
		msg_m.motor_RL = speedRL;
		pub.publish(msg_m);

		last_time = current_time;

		loop_rate.sleep();
		ros::spinOnce();
	}
}
