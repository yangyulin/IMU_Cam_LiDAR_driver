#include "ros/ros.h" 
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Imu.h"

#include <cv.h>
#include <highgui.h>
#include "cxcore.hpp"
#include <pthread.h>
#include <sys/time.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "loitorusbcam.h"
#include "loitorimu.h"

#include <sstream>


using namespace std;
using namespace cv;


ros::Publisher pub_imu;

/*
*  用于构造cv::Mat 的左右眼图像
*/
cv::Mat img_left;
cv::Mat img_right;

/*
*  当前左右图像的时间戳
*/
timeval left_stamp,right_stamp;

/*
*  imu viewer
*/
bool visensor_Close_IMU_viewer=false;
bool imu_start_transfer=false;
void* imu_data_stream(void *)
{
	int counter=0;
	imu_start_transfer=false;


	while((!visensor_Close_IMU_viewer)&&!imu_start_transfer)usleep(1000);
	while(!visensor_Close_IMU_viewer)
	{
		if(visensor_imu_have_fresh_data())
           	{
			counter++;
			// 每隔20帧显示一次imu数据
			if(counter>=20) 
			{
				//cout<<"visensor_imudata_pack->a : "<<visensor_imudata_pack.ax<<" , "<<visensor_imudata_pack.ay<<" , "<<visensor_imudata_pack.az<<endl;
				float ax=visensor_imudata_pack.ax;
				float ay=visensor_imudata_pack.ay;
				float az=visensor_imudata_pack.az;
				//cout<<"visensor_imudata_pack->a : "<<sqrt(ax*ax+ay*ay+az*az)<<endl;
				//cout<<"imu_time : "<<visensor_imudata_pack.imu_time<<endl;
				//cout<<"imu_time : "<<visensor_imudata_pack.system_time.tv_usec<<endl;
				counter=0;
			}
			sensor_msgs::Imu imu_msg;
			imu_msg.header.frame_id = "/imu";
			ros::Time imu_time;
			imu_time.sec=visensor_imudata_pack.system_time.tv_sec;
			imu_time.nsec=1000*visensor_imudata_pack.system_time.tv_usec;
			imu_msg.header.stamp = imu_time;
			imu_msg.header.seq=0;

			imu_msg.linear_acceleration.x=visensor_imudata_pack.ax;
			imu_msg.linear_acceleration.y=visensor_imudata_pack.ay;
			imu_msg.linear_acceleration.z=visensor_imudata_pack.az;
			imu_msg.angular_velocity.x=3.1415926f*visensor_imudata_pack.rx/180.0f;
			imu_msg.angular_velocity.y=3.1415926f*visensor_imudata_pack.ry/180.0f;
			imu_msg.angular_velocity.z=3.1415926f*visensor_imudata_pack.rz/180.0f;
			imu_msg.orientation.w=visensor_imudata_pack.qw;
			imu_msg.orientation.x=visensor_imudata_pack.qx;
			imu_msg.orientation.y=visensor_imudata_pack.qy;
			imu_msg.orientation.z=visensor_imudata_pack.qz;

			pub_imu.publish(imu_msg);
		}
		usleep(10);
	}
	pthread_exit(NULL);
}


int main(int argc, char **argv)
{ 
	/************************ Start Cameras ************************/
	if(argv[1])
		visensor_load_settings(argv[1]);
	else 
		visensor_load_settings("src/loitor_stereo_visensor/Loitor_VISensor_Setups.txt");

	// 手动设置相机参数
	//set_current_mode(5);
	//set_auto_EG(0);
	//set_exposure(50);
	//set_gain(200);
	//set_visensor_cam_selection_mode(2);
	//set_resolution(false);
	//set_fps_mode(true);
	// 保存相机参数到原配置文件
	//save_current_settings();
	
	int r = visensor_Start_Cameras();
	if(r<0)
	{
		printf("Opening cameras failed...\r\n");
		return r;
	}
	// 创建用来接收camera数据的图像
	if(!visensor_resolution_status)
	{
		img_left.create(cv::Size(640,480),CV_8U);
		img_right.create(cv::Size(640,480),CV_8U);
		img_left.data=new unsigned char[IMG_WIDTH_VGA*IMG_HEIGHT_VGA];
		img_right.data=new unsigned char[IMG_WIDTH_VGA*IMG_HEIGHT_VGA];
	}
	else
	{
		img_left.create(cv::Size(752,480),CV_8U);
		img_right.create(cv::Size(752,480),CV_8U);
		img_left.data=new unsigned char[IMG_WIDTH_WVGA*IMG_HEIGHT_WVGA];
		img_right.data=new unsigned char[IMG_WIDTH_WVGA*IMG_HEIGHT_WVGA];
	}
	float hardware_fps=visensor_get_hardware_fps();
	/************************** Start IMU **************************/
	int fd=visensor_Start_IMU();
	if(fd<0)
	{
		printf("open_port error...\r\n");
		return 0;
	}
	printf("open_port success...\r\n");
	usleep(100000);
	/************************ ************ ************************/

	//Create imu_data_stream thread
	pthread_t imu_data_thread;
	int temp;
	if(temp = pthread_create(&imu_data_thread, NULL, imu_data_stream, NULL))
	printf("Failed to create thread imu_data_stream\r\n");
	
	ros::init(argc, argv, "loitor_stereo_visensor");

	ros::NodeHandle n;

	// imu publisher
	pub_imu = n.advertise<sensor_msgs::Imu>("imu0", 200);
 
	// publish 到这两个 topic
	image_transport::ImageTransport it(n);
	image_transport::Publisher pub = it.advertise("/cam0/image_raw", 1);
	sensor_msgs::ImagePtr msg;

	image_transport::ImageTransport it1(n);
	image_transport::Publisher pub1 = it1.advertise("/cam1/image_raw", 1);
	sensor_msgs::ImagePtr msg1;

	// 使用camera硬件帧率设置发布频率
	ros::Rate loop_rate((int)hardware_fps);

	int static_ct=0;

	timeval img_time_test,img_time_offset;
	img_time_test.tv_usec=0;
	img_time_test.tv_sec=0;
	img_time_offset.tv_usec=50021;
	img_time_offset.tv_sec=0;

	while (ros::ok())
	{
		imu_start_transfer=true;

		//cout<<"visensor_get_hardware_fps() ==== "<<visensor_get_hardware_fps()<<endl;

		if(visensor_cam_selection==0)
		{

			visensor_imudata paired_imu=visensor_get_stereoImg((char *)img_left.data,(char *)img_right.data,left_stamp,right_stamp);


			// 显示同步数据的时间戳（单位微秒）
			//cout<<"left_time : "<<left_stamp.tv_usec<<endl;
			//cout<<"right_time : "<<right_stamp.tv_usec<<endl;
			//cout<<"paired_imu time ===== "<<paired_imu.system_time.tv_usec<<endl<<endl;
			//cout<<"visensor_get_hardware_fps() ==== "<<1.0f/visensor_get_hardware_fps()<<endl;

			cv_bridge::CvImage t_left=cv_bridge::CvImage(std_msgs::Header(), "mono8", img_left);
			cv_bridge::CvImage t_right=cv_bridge::CvImage(std_msgs::Header(), "mono8", img_right);

			// 加时间戳(right_time=left_time)
			ros::Time msg_time;
			msg_time.sec=left_stamp.tv_sec;
			msg_time.nsec=1000*left_stamp.tv_usec;
			t_left.header.stamp = msg_time;
			
			ros::Time msg1_time;
			msg1_time.sec=left_stamp.tv_sec;
			msg1_time.nsec=1000*left_stamp.tv_usec;
			t_right.header.stamp = msg1_time;
			t_right.header.seq=0;
			t_left.header.seq=0;

			msg = t_left.toImageMsg();
			msg1 = t_right.toImageMsg();

			static_ct++;
			{
				pub.publish(msg);
				pub1.publish(msg1);
				static_ct=0;
			}
			
			// 显示时间戳
			//cout<<"left_time : "<<left_stamp.tv_usec<<endl;
			//cout<<"right_time : "<<right_stamp.tv_usec<<endl<<endl;

		}
		else if(visensor_cam_selection==1)
		{
			visensor_imudata paired_imu=visensor_get_rightImg((char *)img_right.data,right_stamp);

			// 显示同步数据的时间戳（单位微秒）
			cout<<"right_time : "<<right_stamp.tv_usec<<endl;
			cout<<"paired_imu time ===== "<<paired_imu.system_time.tv_usec<<endl<<endl;

			cv_bridge::CvImage t_right=cv_bridge::CvImage(std_msgs::Header(), "mono8", img_right);

			// 加时间戳
			ros::Time msg1_time;
			msg1_time.sec=right_stamp.tv_sec;
			msg1_time.nsec=1000*right_stamp.tv_usec;
			t_right.header.stamp = msg1_time;
			t_right.header.seq=0;
			
			msg1 = t_right.toImageMsg();
			
			pub1.publish(msg1);
		}
		else if(visensor_cam_selection==2)
		{
			visensor_imudata paired_imu=visensor_get_leftImg((char *)img_left.data,left_stamp);

			// 显示同步数据的时间戳（单位微秒）
			cout<<"left_time : "<<left_stamp.tv_usec<<endl;
			cout<<"paired_imu time ===== "<<paired_imu.system_time.tv_usec<<endl<<endl;

			cv_bridge::CvImage t_left=cv_bridge::CvImage(std_msgs::Header(), "mono8", img_left);

			// 加时间戳
			ros::Time msg_time;
			msg_time.sec=left_stamp.tv_sec;
			msg_time.nsec=1000*left_stamp.tv_usec;
			t_left.header.stamp = msg_time;
			t_left.header.seq=0;


			msg = t_left.toImageMsg();

			
			static_ct++;
			if(static_ct>=5)
			{
				pub.publish(msg);
				static_ct=0;
			}
		}

		ros::spinOnce(); 

		loop_rate.sleep(); 
		
	}

	/* shut-down viewers */
	visensor_Close_IMU_viewer=true;
	if(imu_data_thread !=0)
	{
		pthread_join(imu_data_thread,NULL);
	}

	cout<<endl<<"shutting-down Cameras"<<endl;

	/* close cameras */
	visensor_Close_Cameras();
	/* close IMU */
	visensor_Close_IMU();
	
	return 0;
}











