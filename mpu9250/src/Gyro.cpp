#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <pigpiod_if2.h>
#include <sensor_msgs/Imu.h>

float offsetGyroX = 0.0f;
float offsetGyroY = 0.0f;
float offsetGyroZ = 0.0f;

float gyroCoefficient = 0.0f;

float pregx = 0.0f, pregy = 0.0f, pregz = 0.0f;
float degreeX = 0.0f, degreeY = 0.0f, degreeZ = 0.0f;
float dt = 0.01;
float rad = 3.1415926535 / 180;

bool RESET = false;
bool first = false;

char data[6];
float sum[3] = {0.0f, 0.0f, 0.0f};

int u2s(unsigned unsigneddata);
void calib();

void Reset_cb(const std_msgs::Bool& msg);

int pi = pigpio_start(0, 0);
unsigned handle = i2c_open(pi, 1, 0x68, 0);


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Gyro");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);

	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
	ros::Subscriber sub_Reset = nh.subscribe("Reset", 1000, Reset_cb);

	sensor_msgs::Imu msg;

	msg.header.frame_id = "imu";

	calib();

	//データを取得する
	while(ros::ok())
	{
		if(RESET)
		{
			calib();
		}

		i2c_read_i2c_block_data(pi, handle, 0x43, data, 6);
		float rawX = gyroCoefficient * u2s(data[0] << 8 | data[1]);
		float rawY = gyroCoefficient * u2s(data[2] << 8 | data[3]);
		float rawZ = gyroCoefficient * u2s(data[4] << 8 | data[5]);

		float rawX_1 = gyroCoefficient * u2s(data[0] << 8 | data[1]) + offsetGyroX;
		float rawY_1 = gyroCoefficient * u2s(data[2] << 8 | data[3]) + offsetGyroY;
		float rawZ_1 = gyroCoefficient * u2s(data[4] << 8 | data[5]) + offsetGyroZ;

		//角度を計算
		degreeX += ((pregx + rawX_1) * dt / 2) * rad;
		degreeY += ((pregy + rawY_1) * dt / 2) * rad;
		degreeZ += ((pregz + rawZ_1) * dt / 2) * rad;

		pregx = rawX_1;
		pregy = rawY_1;
		pregz = rawZ_1;

		float cr2 = cos(0*0.5);
		float cp2 = cos(0*0.5);
		float cy2 = cos(degreeZ*0.5);
		float sr2 = sin(0*0.5);
		float sp2 = sin(0*0.5);
		float sy2 = sin(degreeZ*0.5);

		msg.orientation.w = cr2*cp2*cy2 + sr2*sp2*sy2;
		msg.orientation.x = sr2*cp2*cy2 - cr2*sp2*sy2;
		msg.orientation.y = cr2*sp2*cy2 + sr2*cp2*sy2;
		msg.orientation.z = cr2*cp2*sy2 - sr2*sp2*cy2;


		msg.angular_velocity.x = degreeX;
		msg.angular_velocity.y = degreeY;
		msg.angular_velocity.z = degreeZ;
		imu_pub.publish(msg);

		/*printf("%8.7f\t", rawX);
		printf("%8.7f\t", rawY);
		printf("%8.7f\t", rawZ);
		printf("%8.7f\t", degreeX);
		printf("%8.7f\t", degreeY);
		printf("%8.7f\n", degreeZ);*/

		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}

void calib()
{
	if(!first)
	{
		for (int i = 0; i < 3; i++)
		{
			sum[i] = 0;
		}

		// レジスタをリセットする
		i2c_write_byte_data(pi, handle, 0x6B, 0x80);
		time_sleep(0.1);

		//PWR_MGMT_1をクリア
		i2c_write_byte_data(pi, handle, 0x6B, 0x00);
		time_sleep(0.1);

		//ジャイロセンサの計測レンジを1000dpsに修正する
		i2c_write_byte_data(pi, handle, 0x1B, 0x10);

		//low-passフィルタを設定
		i2c_write_byte_data(pi, handle, 0x1A, 0x02);

		//dpsを算出する係数
		gyroCoefficient = 1000 / float(0x8000);

		//較正値を算出する
		ROS_INFO("Gyro calibration start");

		//実データのサンプルを取る
		for(int i = 0; i < 1000; i++)
		{
			i2c_read_i2c_block_data(pi, handle, 0x43, data, 6);
			sum[0] += gyroCoefficient * u2s(data[0] << 8 | data[1]);
			sum[1] += gyroCoefficient * u2s(data[2] << 8 | data[3]);
			sum[2] += gyroCoefficient * u2s(data[4] << 8 | data[5]);
		}

		//平均値をオフセットにする
		offsetGyroX = -1.0 * sum[0] / 1000;
		offsetGyroY = -1.0 * sum[1] / 1000;
		offsetGyroZ = -1.0 * sum[2] / 1000;

		printf("%6.6f\t", offsetGyroX);
		printf("%6.6f\t", offsetGyroY);
		printf("%6.6f\n", offsetGyroZ);

		ROS_INFO("Gyro calibration complete");

		first = true;
	}
}

void Reset_cb(const std_msgs::Bool& msg)
{
	RESET = msg.data;

	if(msg.data == true)
	{
		first = false;
	}
	else;
}

int u2s(unsigned unsigneddata)
{
	if(unsigneddata & (0x01 << 15))
	{
		unsigneddata = -1 * ((unsigneddata ^ 0xffff) + 1);
	}
	return unsigneddata;
}
