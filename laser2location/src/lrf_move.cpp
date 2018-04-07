#include <ros/ros.h>
#include <nemcon/pid_param.h>
#include <geometry_msgs/PoseStamped.h>

void lrf_cb(const geometry_msgs::PoseStamped& msg);

bool flag_x = false;
bool flag_y = false;

nemcon::pid_param msg_pid_param;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "servo_test");
	ros::NodeHandle nh;

	ros::Rate loop_rate(40);

	ros::Subscriber sub_lrf = nh.subscribe("/lrf_pose", 1000, lrf_cb);

	pub_tar_dis = nh.advertise<nemcon::pid_param>("pid_param", 1000);

	while(ros::ok())
	{
		if(flag)
		{
			if(lrf_x > 0 && !flag_x)
			{
				ROS_INFO("%f\n", lrf_x);
				msg_pid_param.front = 4;
				pub_tar_dis.publish(msg_pid_param);
			}
			if(lrf_x < 0 && !flag_x)
			{
				msg_pid_param.front = 2;
				pub_tar_dis.publish(msg_pid_param);
			}
			if(-0.01 < lrf_x && lrf_x < 0.01)
			{
				msg_pid_param.speed = 0;
				pub_tar_dis.publish(msg_pid_param);
				flag_x = true;
			}
			else
			{
				flag_x = false;
			}

			if(lrf_y > 0 && flag_x && !flag_y)
			{
				msg_pid_param.front = 3;
				pub_tar_dis.publish(msg_pid_param);
			}
			if(lrf_y < 0 && flag_x && !flag_y)
			{
				msg_pid_param.front = 1;
				pub_tar_dis.publish(msg_pid_param);
			}
			if(-0.01 < lrf_y && lrf_y < 0.01)
			{
				msg_pid_param.speed = 0;
				pub_tar_dis.publish(msg_pid_param);
				flag_y = true;
			}
			else
			{
				flag_y = false;
			}

			if(!(-0.01 < lrf_z && lrf_z < 0.01) && flag_x && flag_y)
			{
				msg_pid_param.speed = -1;
				pub_tar_dis.publish(msg_pid_param);
			}
			else
			{
				msg_pid_param.speed = 0;
				pub_tar_dis.publish(msg_pid_param);
				flag = false;
			}
		}

		loop_rate.sleep();
		ros::spinOnce();
	}
}

void lrf_cb(const geometry_msgs::PoseStamped& msg)
{
	lrf_x = msg.pose.position.x;
	lrf_y = msg.pose.position.y;
	lrf_y = msg.pose.orientation.z;
}
