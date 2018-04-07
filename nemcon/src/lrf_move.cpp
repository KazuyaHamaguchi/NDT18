#include <ros/ros.h>
#include <nemcon/pid_param.h>
#include <geometry_msgs/PoseStamped.h>
#include <accel_decel/result.h>

void lrf_cb(const geometry_msgs::PoseStamped& msg);

bool flag = true;
bool flag_x = false;
bool flag_y = false;
bool flag_z = false;

float lrf_x = 0.0f;
float lrf_y = 0.0f;
float lrf_z = 0.0f;

accel_decel::result msg_acc;
nemcon::pid_param msg_pid_param;
ros::Publisher pub_tar_dis;
ros::Publisher pub_acc;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lrf_move");
	ros::NodeHandle nh;

	ros::Rate loop_rate(40);

	ros::Subscriber sub_lrf = nh.subscribe("/lrf_pose", 1000, lrf_cb);

	pub_tar_dis = nh.advertise<nemcon::pid_param>("pid_param", 1000);
	pub_acc = nh.advertise<accel_decel::result>("accel_decel/result", 1000);

	msg_pid_param.pattern = 1;
	msg_acc.Vmax = false;
	flag = true;

	while(ros::ok())
	{
		if(flag)
		{
			if(!flag_x)
			{
				if(lrf_x > 0.01)
				{
					ROS_INFO("lrf_x:%f", lrf_x);
					msg_acc.V = 0.05;
					msg_pid_param.front = 4;
					pub_tar_dis.publish(msg_pid_param);
					pub_acc.publish(msg_acc);
					flag_x = false;
				}
				if(lrf_x < -0.01)
				{
					ROS_INFO("-lrf_x:%f", lrf_x);
					msg_acc.V = 0.05;
					msg_pid_param.front = 2;
					pub_tar_dis.publish(msg_pid_param);
					pub_acc.publish(msg_acc);
					flag_x = false;
				}
				if(-0.01 <= lrf_x && lrf_x <= 0.01)
				{
					ROS_INFO("lrf_x OK");
					msg_acc.V = 0;
					pub_tar_dis.publish(msg_pid_param);
					pub_acc.publish(msg_acc);
					flag_x = true;
				}
			}

			if(flag_x)
			{
				if(lrf_y > 0 && !flag_y)
				{
					ROS_INFO("lrf_y:%f", lrf_y);
					msg_acc.V = 0.05;
					msg_pid_param.front = 3;
					pub_tar_dis.publish(msg_pid_param);
					pub_acc.publish(msg_acc);
					flag_y = false;
				}
				if(lrf_y < 0 && flag_x && !flag_y)
				{
					ROS_INFO("-lrf_y:%f", lrf_y);
					msg_acc.V = 0.05;
					msg_pid_param.front = 1;
					pub_tar_dis.publish(msg_pid_param);
					pub_acc.publish(msg_acc);
					flag_y = false;
				}
				if(-0.01 < lrf_y && lrf_y < 0.01)
				{
					ROS_INFO("lrf_y OK");
					msg_acc.V = 0;
					pub_tar_dis.publish(msg_pid_param);
					pub_acc.publish(msg_acc);
					flag_y = true;
				}
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
	lrf_z = msg.pose.orientation.z;

  //ROS_INFO("%f\t %f\t %f\t", lrf_x, lrf_y, lrf_z);
}
