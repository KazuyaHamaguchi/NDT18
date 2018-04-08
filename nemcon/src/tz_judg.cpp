#include <ros/ros.h>
#include <nemcon/TZ_judg.h>
#include <nemcon/object_in.h>
#include <std_msgs/Int8.h>

ros::Time current_time, last_time;

bool objR = false;
bool objT = false;
bool objL = false;

bool leave = false;
bool leave2 = false;
bool judg = false;

float t = 0.0f;

void object_cb(const nemcon::object_in& msg);
void throw_cb(const std_msgs::Int8& msg);

ros::Publisher pub_judg;
ros::Publisher pub_throw;
std_msgs::Int8 msg_judg;
std_msgs::Int8 msg_throw;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "TZ_judg");
	ros::NodeHandle nh;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	ros::Rate loop_rate(40);

	ros::Subscriber sub_throw = nh.subscribe("Throw_on_1", 1000, throw_cb);
	ros::Subscriber sub_obj = nh.subscribe("object_in", 1000, object_cb);
	pub_judg = nh.advertise<std_msgs::Int8>("TZ_judg", 1000);
	ros::Subscriber sub_throw = nh.subscribe("Throw_on_1", 1000, throw_cb);
	pub_throw = nh.advertise<std_msgs::Int8>("Throw_on", 1000);

	while(ros::ok())
	{
		current_time = ros::Time::now();
		t += (current_time - last_time).toSec();

		if(leave)
		{
			if(!objR && !objT && !objL)
			{
				if(t >= 0.8)
				{
					ROS_INFO("leave");
					msg_judg.data = 1;
					pub_judg.publish(msg_judg);
					leave = false;
				}
			}
			else
			{
				t = 0.0f;
			}
		}

		if(leave2)
		{
			if(!objR && !objT && !objL)
			{
				if(t >= 0.8)
				{
					ROS_INFO("leave2");
					msg_judg.data = 5;
					pub_judg.publish(msg_judg);
					leave2 = false;
				}
			}
			else
			{
				t = 0.0f;
			}
		}

		if(judg)
		{
			if(objR && objT && !objL)
			{
				if(t >= 1.5)
				{
					ROS_INFO("TZ1");
					msg_judg.data = 2;
					pub_judg.publish(msg_judg);
					judg = false;
				}
			}
			else if(!objR && objT && objL)
			{
				if(t >= 1.5)
				{
					ROS_INFO("TZ2");
					msg_judg.data = 3;
					pub_judg.publish(msg_judg);
					judg = false;
				}
			}
			else if(objR && !objT && objL)
			{
				if(t >= 1.5)
				{
					ROS_INFO("TZ3");
					msg_judg.data = 4;
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

void throw_cb(const std_msgs::Int8& msg)
{
	if(msg.data == 50)
	{
		ROS_INFO("msg_leave");
		leave = true;
		leave2 = false;
		judg - false;
	}
	if(msg.data == 51)
	{
		ROS_INFO("msg_leave2");
		leave = false;
		leave2 = true;
		judg = false;
	}
	if(msg.data == 52)
	{
		ROS_INFO("msg_judg");
		leave = false;
		leave2 = false;
		judg = true;
	}
	if(msg.data == 100)
	{
		msg_throw.data = -42;
		pub_throw.publish(msg_throw);
	}
}
