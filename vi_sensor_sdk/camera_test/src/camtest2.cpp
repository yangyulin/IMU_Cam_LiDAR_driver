#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "loitorusbcam.h"
#include "loitorimu.h"

using namespace std;
using namespace cv;
bool close_img_viewer=false;
bool visensor_Close_IMU_viewer=false;

// 当前左右图像的时间戳
timeval left_stamp,right_stamp;

// image viewer
void *opencv_showimg(void*)
{
    Mat img_left;
    Mat img_right;

    if(!visensor_resolution_status)
    {
        img_left.create(Size(640,480),CV_8U);
        img_right.create(Size(640,480),CV_8U);
        img_left.data=new unsigned char[IMG_WIDTH_VGA*IMG_HEIGHT_VGA];
        img_right.data=new unsigned char[IMG_WIDTH_VGA*IMG_HEIGHT_VGA];
    }
    else
    {
        img_left.create(Size(752,480),CV_8U);
        img_right.create(Size(752,480),CV_8U);
        img_left.data=new unsigned char[IMG_WIDTH_WVGA*IMG_HEIGHT_WVGA];
        img_right.data=new unsigned char[IMG_WIDTH_WVGA*IMG_HEIGHT_WVGA];
    }
    while(!close_img_viewer)
    {
        if(visensor_cam_selection==2)
        {
            visensor_imudata paired_imu=visensor_get_leftImg((char *)img_left.data,left_stamp);

            // 显示同步数据的时间戳（单位微秒）
            cout<<"left_time : "<<left_stamp.tv_usec<<endl;
            cout<<"paired_imu time ===== "<<paired_imu.system_time.tv_usec<<endl<<endl;

            imshow("left",img_left);
            waitKey(1);
        }
        //Cam2
        else if(visensor_cam_selection==1)
        {
            visensor_imudata paired_imu=visensor_get_rightImg((char *)img_right.data,right_stamp);

            // 显示同步数据的时间戳（单位微秒）
            cout<<"right_time : "<<right_stamp.tv_usec<<endl;
            cout<<"paired_imu time ===== "<<paired_imu.system_time.tv_usec<<endl<<endl;

            imshow("right",img_right);
            waitKey(1);
        }
        // Cam1 && Cam2
        else if(visensor_cam_selection==0)
        {
            visensor_imudata paired_imu=visensor_get_stereoImg((char *)img_left.data,(char *)img_right.data,left_stamp,right_stamp);

            // 显示同步数据的时间戳（单位微秒）
            cout<<"left_time : "<<left_stamp.tv_usec<<endl;
            cout<<"right_time : "<<right_stamp.tv_usec<<endl;
            cout<<"paired_imu time ===== "<<paired_imu.system_time.tv_usec<<endl<<endl;

            imshow("left",img_left);
            imshow("right",img_right);
            waitKey(1);
        }
    }
    pthread_exit(NULL);
}


void* show_imuData(void *)
{
    while(!visensor_Close_IMU_viewer)
    {
        if(visensor_imu_have_fresh_data())
        {
            static int printcount=0;
            if((printcount++)%20==0)
                printf("IMU Gyr: %8.4f,%8.4f,%8.4f, Acc: %8.4f,%8.4f,%8.4f, Quat(WXYZ): %8.4f,%8.4f,%8.4f,%8.4f\n",
                       visensor_imudata_pack.rx,
                       visensor_imudata_pack.ry,
                       visensor_imudata_pack.rz,
                       visensor_imudata_pack.ax,
                       visensor_imudata_pack.ay,
                       visensor_imudata_pack.az,
                       visensor_imudata_pack.qw,
                       visensor_imudata_pack.qx,
                       visensor_imudata_pack.qy,
                       visensor_imudata_pack.qz);
        }
        usleep(100);
    }
    pthread_exit(NULL);
}

int main(int argc, char* argv[])
{

    /************************ Start Cameras ************************/
    visensor_load_settings("../Loitor_VISensor_Setups.txt");
    /*
    // 手动设置相机参数
    visensor_set_current_mode(5);
    visensor_set_auto_EG(0);
    visensor_set_exposure(50);
    visensor_set_gain(200);
    visensor_set_cam_selection_mode(2);
    visensor_set_resolution(false);
    visensor_set_fps_mode(true);
    // 保存相机参数到原配置文件
    visensor_save_current_settings();
    */

    int r = visensor_Start_Cameras();
    if(r<0)
    {
        printf("Opening cameras failed...\r\n");
        return r;
    }
    /************************** Start IMU **************************/
    int fd=visensor_Start_IMU();
    if(fd<0)
    {
        printf("visensor_open_port error...\r\n");
        return 0;
    }
    printf("visensor_open_port success...\r\n");
    /************************ ************ ************************/

    usleep(100000);

    //Create img_show thread
    pthread_t showimg_thread;
    int temp;
    if(temp = pthread_create(&showimg_thread, NULL, opencv_showimg, NULL))
        printf("Failed to create thread opencv_showimg\r\n");
    //Create show_imuData thread
    pthread_t showimu_thread;
    if(temp = pthread_create(&showimu_thread, NULL, show_imuData, NULL))
        printf("Failed to create thread show_imuData\r\n");

    while(1)
    {
        // Do - Nothing :)
        //cout<<visensor_get_imu_portname()<<endl;
        //cout<<visensor_get_hardware_fps()<<endl;
        usleep(500000);
    }

    /* shut-down viewers */
    close_img_viewer=true;
    visensor_Close_IMU_viewer=true;
    if(showimg_thread !=0)
    {
        pthread_join(showimg_thread,NULL);
    }
    if(showimu_thread !=0)
    {
        pthread_join(showimu_thread,NULL);
    }

    /* close cameras */
    visensor_Close_Cameras();
    /* close IMU */
    visensor_Close_IMU();

    return 0;
}
