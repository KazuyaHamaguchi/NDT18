#include <ros/ros.h>
#include <nemcon/TZ_judg.h>
#include <nemcon/object_in.h>
#include <std_msgs/Int8.h>

ros::Time current_time, last_time;

bool objR = false;
bool objT = false;
bool objL = false;

bool leave = false;
bool judg = true;

float t = 0.0f;

void object_cb(const nemcon::object_in& msg);
//void throw_cb(const std_msgs::Int8& msg);

std_msgs::Int8 msg_throw;
ros::Publisher pub_judg;
nemcon::TZ_judg msg_judg;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "TZ_judg");
	ros::NodeHandle nh;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	ros::Rate loop_rate(40);

	//ros::Subscriber sub_throw = nh.subscribe("Throw_on_1", 1000, throw_cb);
	ros::Subscriber sub_obj = nh.subscribe("object_in", 1000, object_cb);
	pub_judg = nh.advertise<nemcon::TZ_judg>("TZ_judg", 1000);

	while(ros::ok())
	{
		current_time = ros::Time::now();
		t += (current_time - last_time).toSec();

		if(leave)
		{
			if(!objR && !objT && !objL)
			{
				if(t >= 0.5)
				{
					ROS_INFO("leave");
					msg_judg.leave = true;
					pub_judg.publish(msg_judg);
					leave = false;
				}
			}
			else
			{
				t = 0.0f;
			}
		}

		if(judg)
		{
			if(!objR && !objT && objL)
			{
				if(t >= 1.5)
				{
					ROS_INFO("TZ1");
					msg_judg.TZ1 = true;
					pub_judg.publish(msg_judg);
					judg = false;
				}
			}
			else
			{
				t = 0.0f;
			}
		}
		last_time = current_time;
		loop_rate.sleep();
		ros::spinOnce();
	}
}

void object_cb(const nemcon::object_in& msg)
{
	objR = msg.objR;
	objT = msg.objT;
	objL = msg.objL;
}
