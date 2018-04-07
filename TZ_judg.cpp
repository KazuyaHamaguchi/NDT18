#include <ros/ros.h>
#include <nemcon/TZ_judg.h>
#include <nemcon/object_in.h>
#include <std_msgs/Int8.h>

bool objR = false;
bool objT = false;
bool objL = false;

bool leave = true;
bool judg = false;

void object_cb(const snemcon::TZ_judg& msg);
void throw_cb(const std_msgs::Int8& msg);

std_msgs::Int8 msg_throw;
ros::Publisher pub_object;
nemcon::object msg_obj;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "TZ_judg");
	ros::NodeHandle nh;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	ros::Rate loop_rate(40);

	ros::Subscriber sub_throw = nh.subscribe("Throw_on_1", 1000, throw_cb);
	pub_object = nh.advertise<nemcon::object>("object", 1);

	while(ros::ok())
	{

		if(leave)
		{
			if(!objR && !objT && !objL)
			{
				current_time = ros::Time::now();
				t += (current_time - last_time).toSec();
				ROS_INFO("leave");
				if(t >= 1.5)
				{
					msg_obj.leave = true;
					pub_object.publish(msg_obj);
				}
				last_time = current_time;
			}
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
}

void object_cb(const snemcon::TZ_judg& msg)
{
	objR = msg.objR;
	objT = msg.objT;
	objL = msg.objL;
}
