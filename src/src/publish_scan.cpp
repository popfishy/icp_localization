#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

int getBinSize(std::string path)
{
    int size = 0;
    FILE *fp = fopen(path.c_str(), "rb");
    if (fp)
    {
        fseek(fp, 0, SEEK_END);
        size = ftell(fp);
        fclose(fp);
    }
    size = size / (int)sizeof(float) / 4;
    return size;
}

// 读取KITTI VELODYNE点云
Eigen::MatrixXf readBin(std::string path, int size)
{
    Eigen::MatrixXf pc(size, 4);
    std::ifstream velodyne_bin(path, std::ios::binary);
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            float data;
            velodyne_bin.read((char *)&data, sizeof(float));
            pc(i, j) = data;
        }
    }
    velodyne_bin.close();
    return pc;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr readPointCloudKITTI(std::string cur_scan_path)
{
    Eigen::MatrixXf pc = readBin(cur_scan_path, getBinSize(cur_scan_path));
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0; i < pc.rows(); i++)
    {
        pcl::PointXYZI temp;
        temp.x = pc(i, 0);
        temp.y = pc(i, 1);
        temp.z = pc(i, 2);
        cloud->push_back(temp);
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr fastMerge(std::string cur_scan_path)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_scan = readPointCloudKITTI(cur_scan_path);

    for(int i=0; i<cur_scan->size(); i++)
    {
        cur_scan->points[i].intensity = i/10;
    }

    /*Eigen::Matrix<float, 4, 4> trans;
    trans << 1.0 , 0.0 , 0.0 , 0.0,
             0.0 , 0.0 , -1.0 , 0.0,
             0.0 , 1.0 , 0.0 , 0.0,
             0.0 , 0.0 , 0.0 , 1.0;
    pcl::transformPointCloud(*cur_scan, *cur_scan, trans);*/

    pcl::VoxelGrid<pcl::PointXYZI> sor;
    float leaf_size = 0.5f;
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.setInputCloud(cur_scan);
    sor.filter(*cur_scan);

    return cur_scan;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_cur_scan");
    ros::NodeHandle nh;
    ros::Publisher publish_cur_scan = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);

    std::string cur_scan_path = "/home/yjq/dataset/KITTI_odometry/sequences/07/velodyne/000800.bin";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_scan = fastMerge(cur_scan_path);
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cur_scan, msg);

    ros::Rate loop_rate(1);
    while(ros::ok)
    {
        ros::spinOnce();
        publish_cur_scan.publish(msg);
        loop_rate.sleep();
    }
    return 0;
}
