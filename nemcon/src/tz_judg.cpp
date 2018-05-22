#include <ros/ros.h>
#include <nemcon/TZ_judg.h>
#include <nemcon/object_in.h>
#include <std_msgs/Int8.h>

ros::Time current_time, last_time;

bool objR = false;
bool objL = false;

bool leave = false;
bool judg = false;

bool flag = false;

float t = 0.0f;

void object_cb(const nemcon::object_in& msg);
void throw_cb(const std_msgs::Int8& msg);

ros::Publisher pub_judg;
std_msgs::Int8 msg_judg;
ros::Publisher pub_throw;
std_msgs::Int8 msg_throw;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "TZ_judg");
	ros::NodeHandle nh;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	ros::Rate loop_rate(40);

	ros::Subscriber sub_obj = nh.subscribe("object_in", 1000, object_cb);
	ros::Subscriber sub_throw = nh.subscribe("judg_call", 1000, throw_cb);
	pub_judg = nh.advertise<std_msgs::Int8>("TZ_judg", 1000);
	pub_throw = nh.advertise<std_msgs::Int8>("Throw_on", 1000);

	while(ros::ok())
	{
		current_time = ros::Time::now();
		t += (current_time - last_time).toSec();

    if(!flag)
    {
		if(leave)
		{
			if(!objR && !objL)
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

		if(judg)
		{
			if(objR && !objL)
			{
				if(t >= 1.5)
				{
					ROS_INFO("true");
					msg_judg.data = 2;
					pub_judg.publish(msg_judg);
					judg = false;
				}
			}
			else if(!objR && objL)
			{
				if(t >= 1.5)
				{
					ROS_INFO("false");
					msg_judg.data = 3;
					pub_judg.publish(msg_judg);
					judg = false;
				}
			}
			else
			{
				t = 0.0f;
			}
		}
    }
    else;

		last_time = current_time;
		loop_rate.sleep();
		ros::spinOnce();
	}
}

void object_cb(const nemcon::object_in& msg)
{
	objR = msg.objR;
	objL = msg.objL;
}

void throw_cb(const std_msgs::Int8& msg)
{
  ROS_INFO("%d", msg.data);
	if(msg.data == 50)
	{
		ROS_INFO("msg_leave");
    flag = false;
		leave = true;
		judg - false;
	}
	else if(msg.data == 52)
	{
		ROS_INFO("msg_judg");
    flag = false;
		leave = false;
		judg = true;
	}
	else if(msg.data == 100)
	{
		msg_throw.data = -41;
		pub_throw.publish(msg_throw);
	}
  else
  {
    flag = true;
    leave = false;
    judg - false;
    ROS_INFO("msg_judg false");
  }
}
