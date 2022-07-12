#include <cmath> //使用三角函数 M_PI等
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include "tic_toc.h"
#include "common.h"


int N_SCANS = 0;
float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];
double MINIMUM_RANGE =0.1;
bool systemInited = false;
int systemInitCount = 0;
int systemDelay = 0;
const double scanPeriod = 0.1; //10HZ,100毫秒
bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
ros::Publisher pubRemovePoints;
using namespace std;

// //去除距离小于阈值的点
template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                            pcl::PointCloud<PointT> &cloud_out,float thres)
{
    if(&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }
    size_t j = 0;
    for(size_t i = 0; i < cloud_in.points.size();++i)
    {
        if(cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y 
            + cloud_in.points[i].z * cloud_in.points[i].z < thres)
            continue;
        else
        {
            cloud_out.points[j] = cloud_in.points[i];
            j++;
        }
    }
    if(j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }
    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    if(!systemInited)
    {
        systemInitCount++;
        if(systemInitCount >= systemDelay)
        {
            systemInited = true;
        }
        else
        {
            return;
        }
    }
    TicToc t_whole;
    TicToc t_prepare;
    
    vector<int> scanStartInd(N_SCANS,0);
    vector<int> scanEndInd(N_SCANS,0);

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    //用pcl库实现ros格式到pcl格式的转换
    pcl::fromROSMsg(*laserCloudMsg,laserCloudIn);  //这个函数需要包含头文件pcl_conversions
    
    // Step: 第一步 先去除nan点和距离小于阈值的点
    // 用pcl库去除nan点
    vector<int> indices;
    pcl::removeNaNFromPointCloud(laserCloudIn,laserCloudIn,indices);//这个函数需要包含头文件pcl/filters/voxel_grid.h
    //去除距离小于阈值的点
    removeClosedPointCloud(laserCloudIn,laserCloudIn,MINIMUM_RANGE);
    // 计算起始点和终止点的角度，由于激光雷达是顺时针旋转，所以取负代表逆时针旋转
    int cloudSize = laserCloudIn.points.size();
    float startOri = -atan2(laserCloudIn.points[0].y,laserCloudIn.points[0].x);
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;

    if(endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if(endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }
    
    bool halfPassed = false;
    int count = cloudSize;

    PointType point;//PointType就是一个带有强度信息的点云
    //所以laserCloudScans中就是没一个元素都是单独的一个点云
    // 在这里就是所有第一跟线上的点构成一个点云，第二跟线上......
    vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);

    // Step: 第二步 开始划分每条线上点(计算其俯仰角算其线数)
    for(int i = 0;i < cloudSize; i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
        float angle = atan(point.z / (point.x * point.x + point.y * point.y)) * 180.0 / M_PI;
        int scanID = 0;
        if(N_SCANS == 16)
        {
            scanID = int((angle + 15) / 2 +0.5);
            if(scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 64)
        {   
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies 
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }
        
        else
        {
            printf("wrong scan number\n");
            ROS_BREAK();
        }
        // Step: 第三步 计算每个点的水平角 算出某一线上的点在一圈中的哪个位置 并计算该点相对于起点的时间
        
        float ori = -atan2(point.y,point.x);
        if(!halfPassed)
        {
            if(ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if(ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }
            // 如果超过180度说明过一半了
            if(ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            // 确保-PI * 3 / 2 < ori - endOri < PI / 2
            ori += 2 * M_PI;    // 先补偿2PI
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }
        float relTime = (ori - startOri) / (endOri - startOri);
        point.intensity = scanID + scanPeriod * relTime;   // 整数部分是scan的索引，小数部分是相对起始时刻的时间
        laserCloudScans[scanID].push_back(point);
    }

    cloudSize = count; //前边将没用的点再次筛选出去了 所以count记录了这一帧点云中有效点的个数
    ROS_INFO("point size %d \n", cloudSize);

    // Step: 第四步 将所有线数上的点集中到一个点云中,使用两个数组标记起始结果,这里分别+5和-6是不计算前5个点个后6个点的曲率
    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>()); //定义一个指向装有指针的容器指针
    for(int i = 0;i < N_SCANS; i++)
    {
        scanStartInd[i] = laserCloud->size() + 5;
        *laserCloud += laserCloudScans[i];
        scanEndInd[i] = laserCloud->size() - 6;
    }
    ROS_INFO("prepare time %f \n",t_prepare.toc());//toc()函数就是从开始到结束所经过的时间
    
    // Step: 第五步 计算一帧点云中除了最左边和最右边五个点之外的全部点的曲率 就是用当前点左边5个点和右边五个点之和减去10倍的当前点
    for(int i = 5;i < cloudSize - 5;i++)
    {
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd[i] = i;//用来记录当前点在点云中的索引
        cloudNeighborPicked[i] = 0; // cloudNeighborPicked这个是用来表示点云中第i个点有没有取过（取过就是有没有选他做角点或面点，1代表取了，0代表没取）
        cloudLabel[i] = 0; // cloudLabel这个是用来表示点云中的第i个点的状态，2代表曲率最大的角点，1代表一般大角点 -1代表面点 0代表一般面点
    }
    // Step: 第六步 开始将一帧点云分为6份 每份取2个角点和4个面点
    TicToc t_pts;
    pcl::PointCloud<PointType> cornerPointsSharp;
    pcl::PointCloud<PointType> cornerPointsLessSharp;
    pcl::PointCloud<PointType> surfPointsFlat;
    pcl::PointCloud<PointType> surfPointsLessFlat;

    float t_q_sort = 0;
    for(int i = 0; i < N_SCANS; i++)
    {
        if(scanStartInd[i] - scanEndInd[i] < 6)
        {
            continue;
        } 
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>); // 用来存储除了角点之外的面点
        // 将一帧点云分成6等分
        for(int j = 0; j < 6; j++)
        {
            int sp = scanStartInd[i] + j * (scanEndInd[i] - scanEndInd[i]) / 6;
            int ep = scanStartInd[i] + (j + 1) * (scanEndInd[i] - scanStartInd[i]) / 6 - 1;
            TicToc t_tmp;
            sort(cloudSortInd + sp,cloudSortInd + ep + 1, comp);
            t_q_sort += t_tmp.toc();

            int largestPickedNum = 0;
            // 挑选角点
            for(int k = ep; k >= sp; k--)
            {
                int ind = cloudSortInd[k];
                if(cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1)
                {
                    largestPickedNum++;
                    if(largestPickedNum <= 2)
                    {
                        cloudLabel[ind] = 2;
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else if(largestPickedNum <= 20)
                    {
                        cloudLabel[ind] == 1;
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else{
                        break;
                    }
                    cloudNeighborPicked[ind] = 1;
                    // 为了不过度集中 将选中点周围5个点都置为1,避免后续选到
                    for(int l = 1;l < 6;l++)
                    {
                        // 查看相邻点距离是否差异过大，如果差异过大说明点云在此不连续，是特征边缘，就会是新的特征，因此就不置位了
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }
                        cloudNeighborPicked[ind + i] = 1;
                    }
                    for(int l = -6;l < -1; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }
                        cloudNeighborPicked[ind + i] = 1;
                    }
                }
            }
            // 挑选面点
            int smallestPickedNum = 0;
            for(int k = sp; k <= ep; k++)
            {
                int ind = cloudSortInd[k];
                if(cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1)
                {
                    cloudLabel[ind] = -1;
                    surfPointsFlat.push_back(laserCloud->points[ind]);
                    smallestPickedNum++;
                    if(smallestPickedNum >= 4)
                    {
                        break;
                    }
                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    { 
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }
            // 一般平坦的点最多
            for (int k = sp; k <= ep; k++)
            {
                // 这里可以看到，剩下来的点都是一般平坦，这个也符合实际
                if (cloudLabel[k] <= 0)
                {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }

        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        // 一般平坦点比较多,所以这里做一个体素滤波
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);//设置需要滤波的数据
        downSizeFilter.setLeafSize(0.2,0.2,0.2);//滤波立方体的大小
        downSizeFilter.filter(surfPointsLessFlatScanDS);//滤波之后的结果存到surfPointsLessFlatScanDS容器中
    }
    ROS_INFO("sort q time %f \n", t_q_sort);
    ROS_INFO("seperate points time %f\n",t_pts.toc());

    //Step: 分别将当前点云 四种特征的点云发布出去
    // 发布当前点云
    sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "/camera_init";
    pubLaserCloud.publish(laserCloudOutMsg);

    // 发布角点
    sensor_msgs::PointCloud2 cornerPointsSharpMsg;
    pcl::toROSMsg(cornerPointsSharp,cornerPointsSharpMsg);
    cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsSharpMsg.header.frame_id = "/camera_init";
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);//发布角点，每一帧雷达数据分为6份，每份提取二个角点

    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
    cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsLessSharpMsg.header.frame_id = "/camera_init";
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);//发布曲率较大的角点 提取20个角点

    sensor_msgs::PointCloud2 surfPointsFlat2;
    pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
    surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsFlat2.header.frame_id = "/camera_init";
    pubSurfPointsFlat.publish(surfPointsFlat2);//发布面点，每一帧雷达数据分为6份，每份提取四个面点

    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
    surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsLessFlat2.header.frame_id = "/camera_init";
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);//发布一般面点

    ROS_INFO("scan registration time %f ms *************\n", t_whole.toc());
    // 因为雷达的频率是10hz，也就是100ms执行一次回调函数，如果执行一次超过100ms，则会丢帧
    if(t_whole.toc() > 100)
        ROS_WARN("scan registration process over 100ms");
}



int main(int argc, char **argv)
{
    ros::init(argc,argv,"scanRegistration");
    
    ros::NodeHandle nh;

    nh.param<int>("scan_line",N_SCANS,16);
    nh.param<double>("minimum_range",MINIMUM_RANGE,0.1);

    ROS_INFO("scan line number %d \n",N_SCANS);
    if(N_SCANS !=16 && N_SCANS != 32 && N_SCANS !=64 )
    {
        ROS_INFO("only support velodyne with 16,32 or 64 scan line!");
    }

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points",100,laserCloudHandler);

    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2",100);
    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp",100);
    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp",100);
    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat",100);
    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat",100);
    pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);
    



    ros::spin();
    
    return 0;
    
}