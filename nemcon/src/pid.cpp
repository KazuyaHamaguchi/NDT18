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
int pattern;	//0：加減速，1：等速

bool lrf = false;

float imu_P;
float imu_I;
float imu_D;

float lrf_P;
float lrf_I;
float lrf_D;

float enc_P;
float enc_I;
float enc_D;

float v_P;
float v_I;
float v_D;

float vs_P;
float vs_I;
float vs_D;

float lrfv_P;
float lrfv_I;
float lrfv_D;

float speedFR = 0.0f, speedRL = 0.0f, speedFL = 0.0f, speedRR = 0.0f;
float turn_imu = 0.0f, turn_enc_x = 0.0f, turn_enc_y = 0.0f, turn_lrf = 0.0f;
float enc_vx = 0.0f, enc_vy = 0.0f;
float tar_x = 0.0f, tar_y = 0.0f;
float enc_x = 0.0f;

bool vflag = false;

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
	if(max > 0 && 0.1 <= input && input < 3)
	{
		output = 5;
	}
	if(min < 0 && -3 < input && input <= -0.1)
	{
		output = -5;
	}
	if(max > 0 && input <= 0.0 && vflag)
	{
		output = 8080;
	}
	if(min < 0 && 0.0 <= input && vflag)
	{
		output = 8080;
	}
	if(!vflag)
	{
		output = 0;
	}
	return output;
}
float clamp2(float input, float min, float max)
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
	/*if(max > 0 && 1 <= input && input < 3)
	{
		output = 2;
	}
	if(min < 0 && -3 < input && input <= -1)
	{
		output = -2;
	}*/
	return output;
}

void param_cb(const nemcon::pid_param& msg)
{
	speed = msg.speed;
	pattern = msg.pattern;
	front = msg.front;
	tar_x = msg.tar_x;
	tar_y = msg.tar_y;
	if(pattern == 1)
	{
		lrf = true;
	}
	else
	{
		lrf = false;
	}
}

void enc_cb(const deadreckoning::enc& msg)
{
  enc_vx = msg.speed_X;
  enc_vy = msg.speed_Y;
}

void pid_lrf(const geometry_msgs::PoseStamped& msg)
{
	float lasterror = 0, integral = 0, error = 0;

	error = msg.pose.orientation.z - /*0.00000f*/0.0218;

	integral += (error + lasterror) / 2.0 * dt;

	turn_lrf = lrf_P * error + lrf_I * integral + lrf_D * (error - lasterror) / dt;

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

	if(!(msg.V == 0.0))
	{
		vflag = true;
	}
	else
	{
		vflag = false;
	}

	integral_y += (error_y + lasterror_y) / 2.0 * dt;

	if(!lrf)
	{
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
	}
	else
	{
		speed_X= lrfv_P * error_x + lrfv_I * integral_x + lrfv_D * (error_x - lasterror_x) / dt;
		speed_Y = lrfv_P * error_y + lrfv_I * integral_y + lrfv_D * (error_y - lasterror_y) / dt;
	}

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
	ros::init(argc, argv, "pid_control", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	ros::Rate loop_rate(40);
	ros::NodeHandle local_nh("~");

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

	if(!local_nh.hasParam("lrf_P"))
	{
		ROS_INFO("Parameter lrf_P is not defind. Now, it is set default value.");
		local_nh.setParam("lrf_P", 0);
	}
	if(!local_nh.getParam("lrf_P", lrf_P))
	{
		ROS_ERROR("parameter front is invalid.");
		return -1;
	}
	ROS_INFO("lrf_P: %f", lrf_P);

	if(!local_nh.hasParam("lrf_I"))
	{
		ROS_INFO("Parameter lrf_I is not defind. Now, it is set default value.");
		local_nh.setParam("lrf_I", 0);
	}
	if(!local_nh.getParam("lrf_I", lrf_I))
	{
		ROS_ERROR("parameter front is invalid.");
		return -1;
	}
	ROS_INFO("lrf_I: %f", lrf_I);

	if(!local_nh.hasParam("lrf_D"))
	{
		ROS_INFO("Parameter lrf_D is not defind. Now, it is set default value.");
		local_nh.setParam("lrf_D", 0);
	}
	if(!local_nh.getParam("lrf_D", lrf_D))
	{
		ROS_ERROR("parameter front is invalid.");
		return -1;
	}
	ROS_INFO("lrf_D: %f", lrf_D);

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

	if(!local_nh.hasParam("lrfv_P"))
	{
		ROS_INFO("Parameter lrfv_P is not defind. Now, it is set default value.");
		local_nh.setParam("lrfv_P", 0);
	}
	if(!local_nh.getParam("lrfv_P", lrfv_P))
	{
		ROS_ERROR("parameter front is invalid.");
		return -1;
	}
	ROS_INFO("lrfv_P: %f", lrfv_P);

	if(!local_nh.hasParam("lrfv_I"))
	{
		ROS_INFO("Parameter lrfv_I is not defind. Now, it is set default value.");
		local_nh.setParam("lrfv_I", 0);
	}
	if(!local_nh.getParam("lrfv_I", lrfv_I))
	{
		ROS_ERROR("parameter front is invalid.");
		return -1;
	}
	ROS_INFO("lrfv_I: %f", lrfv_I);

	if(!local_nh.hasParam("lrfv_D"))
	{
		ROS_INFO("Parameter lrfv_D is not defind. Now, it is set default value.");
		local_nh.setParam("lrfv_D", 0);
	}
	if(!local_nh.getParam("lrfv_D", lrfv_D))
	{
		ROS_ERROR("parameter front is invalid.");
		return -1;
	}
	ROS_INFO("lrfv_D: %f", lrfv_D);

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

		if(pattern == -1)	//その場旋回
		{
			speedFR = clamp2(nearbyint( - turn_imu), -20, 20);
			speedFL = clamp2(nearbyint( + turn_imu), -20, 20);
			speedRL = clamp2(nearbyint( + turn_imu), -20, 20);
			speedRR = clamp2(nearbyint( - turn_imu), -20, 20);
    }

		if(pattern == 0) //加減速
		{
			switch(front)
			{
				case 1:	//前
					speedFR = clamp(nearbyint( speed_Y - turn_imu + turn_enc_x), 0, 90);
					speedFL = clamp(nearbyint( speed_Y + turn_imu - turn_enc_x), 0, 90);
					speedRL = clamp(nearbyint( speed_Y + turn_imu + turn_enc_x), 0, 90);
					speedRR = clamp(nearbyint( speed_Y - turn_imu - turn_enc_x), 0, 90);
					break;

				case 2:	//右
					speedFR = clamp(nearbyint(-(speed_X + turn_imu + turn_enc_y )), -90, 0);
					speedFL = clamp(nearbyint( speed_X + turn_imu - turn_enc_y), 0, 90);
					speedRL = clamp(nearbyint(-(speed_X - turn_imu + turn_enc_y)), -90, 0);
					speedRR = clamp(nearbyint( speed_X - turn_imu - turn_enc_y), 0, 90);
					break;

				case 3:	//後
					speedFR = clamp(nearbyint( -(speed_Y + turn_imu + turn_enc_x)), -90, 0);
					speedFL = clamp(nearbyint( -(speed_Y - turn_imu - turn_enc_x)), -90, 0);
					speedRL = clamp(nearbyint( -(speed_Y - turn_imu + turn_enc_x)), -90, 0);
					speedRR = clamp(nearbyint( -(speed_Y + turn_imu - turn_enc_x)), -90, 0);
					break;

				case 4:	//左
					speedFR = clamp(nearbyint( speed_X - turn_imu - turn_enc_y ), 0, 90);
					speedFL = clamp(nearbyint( -(speed_X - turn_imu + turn_enc_y)), -90, 0);
					speedRL = clamp(nearbyint( speed_X + turn_imu - turn_enc_y), 0, 90);
					speedRR = clamp(nearbyint( -(speed_X + turn_imu + turn_enc_y)), -90, 0);
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
					speedFR = clamp2(nearbyint( speed_Y - turn_lrf), 0, 20);
					speedFL = clamp2(nearbyint( speed_Y + turn_lrf), 0, 20);
					speedRL = clamp2(nearbyint( speed_Y + turn_lrf), 0, 20);
					speedRR = clamp2(nearbyint( speed_Y - turn_lrf), 0, 20);
					break;

				case 2:	//右
					speedFR = clamp2(nearbyint(-(speed_X + turn_lrf)), -20, 0);
					speedFL = clamp2(nearbyint( speed_X + turn_lrf), 0, 20);
					speedRL = clamp2(nearbyint(-(speed_X - turn_lrf)), -20, 0);
					speedRR = clamp2(nearbyint( speed_X - turn_lrf), 0, 20);
					break;

				case 3:	//後
					speedFR = clamp2(nearbyint( -(speed_Y + turn_lrf)), -20, 0);
					speedFL = clamp2(nearbyint( -(speed_Y - turn_lrf)), -20, 0);
					speedRL = clamp2(nearbyint( -(speed_Y - turn_lrf)), -20, 0);
					speedRR = clamp2(nearbyint( -(speed_Y + turn_lrf)), -20, 0);
					break;

				case 4:	//左
					speedFR = clamp2(nearbyint( speed_X - turn_lrf), 0, 20);
					speedFL = clamp2(nearbyint( -(speed_X - turn_lrf)), -20, 0);
					speedRL = clamp2(nearbyint( speed_X + turn_lrf), 0, 20);
					speedRR = clamp2(nearbyint( -(speed_X + turn_lrf)), -20, 0);
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
