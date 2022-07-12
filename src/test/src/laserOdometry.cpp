#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "common.h"
#include "tic_toc.h"
#include <mutex>
#include <queue>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
using namespace std;


mutex mBuf;
int skipFrameNUM = 5;
bool systemInited = false;
int cornerPointsSharpNum = 0; //角点个数
int surfPointsFlatNum = 0;   //面点个数
queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;
queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;
queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;

double timeCornerPointsSharp = 0;
double timeCornerPointsLessSharp = 0;
double timeSurfPointsFlat = 0;
double timeSurfPointsLessFlat = 0;
double timeLaserCloudFullRes = 0;

pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());



void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2)
{   
    mBuf.lock();//对此线程先上锁
    cornerSharpBuf.push(cornerPointsSharp2);
    mBuf.unlock();

}

void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2)
{
    mBuf.lock();//对此线程先上锁
    cornerLessSharpBuf.push(cornerPointsLessSharp2);
    mBuf.unlock();
}

void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2)
{
    mBuf.lock();//对此线程先上锁
    surfFlatBuf.push(surfPointsFlat2);
    mBuf.unlock();
}

void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2)
{
    mBuf.lock();//对此线程先上锁
    surfLessFlatBuf.push(surfPointsLessFlat2);
    mBuf.unlock();
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
    mBuf.lock();//对此线程先上锁
    fullPointsBuf.push(laserCloudFullRes2);
    mBuf.unlock();
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"laserOdometry");
    ros::NodeHandle nh;

    nh.param<int>("mapping_skip_frame",skipFrameNUM,2);//采样频率

    ROS_INFO("Mapping %d HZ \n", 10 / skipFrameNUM);

    // 订阅提取出来的点云 也就是在scanRegistration中发布的一些点云数据
    ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp",100,laserCloudSharpHandler);
    
    ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp",100,laserCloudLessSharpHandler);
    
    ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat",100,laserCloudFlatHandler);
    
    ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, laserCloudLessFlatHandler);

    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100, laserCloudFullResHandler);

    ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last",100);

    ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last",100);

    ros::Publisher pubLaserCloudSurfRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3",100);

    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);

    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Path>("/laser_odom_path",100);

    nav_msgs::Path laserPath;

    int frameCount = 0;
    ros::Rate rate(100);//先设定一个频率，然后通过睡眠度过一个循环中剩下的时间，来达到该设定频率

    while(ros::ok())
    {
        ros::spinOnce();// 触发一次回调，参考https://www.cnblogs.com/liu-fa/p/5925381.html
        // 这里的回调函数就是将各自的消息存到各自的队列中

        // 首先确保订阅的五个消息都有,有一个队列为空都不行
        if(!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() &&
           !surfFlatBuf.empty() && !surfLessFlatBuf.empty() &&
           !fullPointsBuf.empty())
        {
            // 分别求出队列的第一个时间
            timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec();
            timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();
            timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec();
            timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.toSec();
            timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();
            // 因为同一帧的时间戳都是相同的,因此这里比较是否是同一帧
            // 一般不会出发这个事件,因为在发布的时候都是五个消息同步发送的,他们的header都赋的一样的
            if(timeCornerPointsSharp != timeLaserCloudFullRes ||
               timeCornerPointsLessSharp != timeLaserCloudFullRes ||
               timeSurfPointsFlat != timeLaserCloudFullRes ||
               timeSurfPointsLessFlat != timeLaserCloudFullRes)
               {
                    ROS_INFO("unsync messeage!");
                    ROS_BREAK();
               }
            // 分别将五个点云消息取出来，同时转成pcl的点云格式
            mBuf.lock();
            cornerPointsSharp->clear();
            pcl::fromROSMsg(*cornerSharpBuf.front(),*cornerPointsSharp);
            mBuf.unlock();

            cornerPointsLessSharp->clear();
            pcl::fromROSMsg(*cornerLessSharpBuf.front(),*cornerPointsLessSharp);
            mBuf.unlock();

            surfPointsFlat->clear();
            pcl::fromROSMsg(*surfFlatBuf.front(),*surfPointsFlat);
            mBuf.unlock();

            surfPointsLessFlat->clear();
            pcl::fromROSMsg(*surfLessFlatBuf.front(),*surfPointsLessFlat);
            mBuf.unlock();

            laserCloudFullRes->clear();
            pcl::fromROSMsg(*fullPointsBuf.front(),*laserCloudFullRes);
            mBuf.unlock();

            TicToc t_whole;
            // 一个什么也不干的初始化
            if(!systemInited)
            {
                systemInited = true;
                ROS_INFO("Initialzation finished \n");
            }
            else
            {
                // 取出比较突出的特征
                int cornerPointsSharpNum = cornerPointsSharp->points.size(); 
            }


        }
    }
}