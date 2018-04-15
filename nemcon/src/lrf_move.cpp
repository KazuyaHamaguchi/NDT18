#include <ros/ros.h>
#include <nemcon/pid_param.h>
#include <geometry_msgs/PoseStamped.h>
#include <accel_decel/result.h>
#include <nemcon/lrf_flag.h>
#include <std_msgs/Int8.h>

void lrf_cb(const geometry_msgs::PoseStamped& msg);
void flag_cb(const nemcon::lrf_flag& msg);

bool flag = false;
bool flag_x = false;
bool flag_y = false;
bool flag_z = false;

float lrf_x = 0.0f;
float lrf_y = 0.0f;
float lrf_z = 0.0f;

float gain = 0.0f;
float offset = 0.0f;

float t = 0.0f;

ros::Time current_time, last_time;

accel_decel::result msg_acc;
nemcon::pid_param msg_pid_param;
std_msgs::Int8 msg_lrf;
ros::Publisher pub_tar_dis;
ros::Publisher pub_acc;
ros::Publisher pub_lrf;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lrf_move");
	ros::NodeHandle nh;

	ros::Rate loop_rate(40);

	ros::Subscriber sub_lrf = nh.subscribe("/lrf_pose", 1000, lrf_cb);
	ros::Subscriber sub_flag = nh.subscribe("lrf_flag", 1000, flag_cb);

	pub_tar_dis = nh.advertise<nemcon::pid_param>("pid_param", 1000);
	pub_acc = nh.advertise<accel_decel::result>("accel_decel/result", 1000);
	pub_lrf = nh.advertise<std_msgs::Int8>("lrf", 1000);

	msg_pid_param.pattern = 1;
	msg_pid_param.speed = 1;
	msg_acc.Vmax = false;

	while(ros::ok())
	{
		current_time = ros::Time::now();
		t += (current_time - last_time).toSec();

		if(flag)
		{
			if(lrf_x > gain + offset && !flag_x)
			{
				ROS_INFO("lrf_x:%f", lrf_x);
				msg_acc.V = 0.05;
				msg_pid_param.front = 4;
				pub_tar_dis.publish(msg_pid_param);
				pub_acc.publish(msg_acc);
				flag_x = false;
			}
			if(lrf_x < - gain + offset && !flag_x)
			{
				ROS_INFO("-lrf_x:%f", lrf_x);
				msg_acc.V = 0.05;
				msg_pid_param.front = 2;
				pub_tar_dis.publish(msg_pid_param);
				pub_acc.publish(msg_acc);
				flag_x = false;
			}
			if(- gain + offset <= lrf_x && lrf_x <= gain + offset)
			{
				ROS_INFO("lrf_x OK");
				msg_acc.V = 0;
				pub_tar_dis.publish(msg_pid_param);
				pub_acc.publish(msg_acc);
				flag_x = true;
			}
			else
			{
			  flag_x = false;
			}
		}

		if(flag_x)
		{
			if(lrf_y > gain && !flag_y)
			{
				ROS_INFO("lrf_y:%f", lrf_y);
				msg_acc.V = 0.05;
				msg_pid_param.front = 3;
				pub_tar_dis.publish(msg_pid_param);
				pub_acc.publish(msg_acc);
				flag_y = false;
			}
			if(lrf_y < -gain && !flag_y)
			{
				ROS_INFO("-lrf_y:%f", lrf_y);
				msg_acc.V = 0.05;
				msg_pid_param.front = 1;
				pub_tar_dis.publish(msg_pid_param);
				pub_acc.publish(msg_acc);
				flag_y = false;
			}
			if(-gain <= lrf_y && lrf_y <= gain)
			{
				ROS_INFO("lrf_y OK");
				msg_acc.V = 0;
				pub_tar_dis.publish(msg_pid_param);
				pub_acc.publish(msg_acc);
				if(t >= 1.2)
				{
          ROS_INFO("lrf stop");
					msg_lrf.data = -50;
					pub_lrf.publish(msg_lrf);
					flag = false;
					flag_x = false;
					flag_y = true;;
				}
			}
			else
			{
			  flag_y = false;
			  t = 0.0f;
			}
		}

		last_time = current_time;

		loop_rate.sleep();
		ros::spinOnce();
	}
}

void lrf_cb(const geometry_msgs::PoseStamped& msg)
{
	lrf_x = msg.pose.position.x;
	lrf_y = msg.pose.position.y;
	lrf_z = msg.pose.orientation.z;
	//ROS_INFO("%f\t %f\t %f\t", lrf_x, lrf_y, lrf_z);
}

void flag_cb(const nemcon::lrf_flag& msg)
{
	flag = msg.flag;

	switch(msg.TZ)
	{
		case 1: 
      gain = 0.02;
			offset = 0.0f;
			break;

		case 2:
      gain = 0.02;
			offset = 3.27503521586;
			break;

    case 3:
      gain = 0.01;
      offset = 0.0f;
      break;

		default:
			offset = 0.0f;
			break;
	}
}
