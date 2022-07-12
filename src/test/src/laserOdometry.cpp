#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <mutex>
#include <queue>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
using namespace std;


mutex mBuf;
int skipFrameNUM = 5;
queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;
queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;
queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;



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
            
        }
    }
}