/*
 * @Author: popfishy
 * @Date: 2023-12-29 15:50:55
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-01-27 21:11:18
 * @Description:
 */

#define EIGEN_NO_DEBUG
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/highgui.hpp>
#include <iostream>

static bool cmp(std::string a, std::string b) {
  if (a < b) {
    return true;
  }
  return false;
}

// 获取按顺序排列的点云数据帧路径
static std::vector<std::string> pathList(std::string path, bool sort = false) {
  std::vector<std::string> files;
  if (!boost::filesystem::exists(path)) {
    std::cout << "dont exist" << std::endl;
    return files;
  }
  if (!boost::filesystem::is_directory(path)) {
    std::cout << "not a dir" << std::endl;
    return files;
  }
  struct dirent *ptr;
  DIR *dir;
  dir = opendir(path.c_str());

  while ((ptr = readdir(dir)) != NULL) {
    if (ptr->d_name[0] == '.') continue;
    files.push_back(path + "/" + ptr->d_name);
  }

  if (sort) {
    std::sort(files.begin(), files.end(), cmp);
  }

  return files;
}

// 获取点云数据帧pose数据
std::vector<Eigen::Matrix<float, 4, 4>> readPose_KITTI(std::string path) {
  std::vector<Eigen::Matrix<float, 4, 4>> poses;
  std::ifstream file(path);
  std::string temp_str;
  file >> temp_str;
  while (!file.eof()) {
    Eigen::Matrix<float, 4, 4> pose_temp;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 4; j++) {
        pose_temp(i, j) = std::stof(temp_str);
        file >> temp_str;
      }
    }
    pose_temp.block<1, 4>(3, 0) = Eigen::Matrix<float, 1, 4>{0, 0, 0, 1};
    poses.push_back(pose_temp);
    if (temp_str.size() == 0) {
      break;
    }
  }
  file.close();

  return poses;
}

// 获取calib数据
std::vector<Eigen::Matrix<float, 3, 4>> readCalib(std::string path) {
  std::vector<Eigen::Matrix<float, 3, 4>> calib(5);  // P0 P1 P2 P3 Tr

  std::ifstream file(path);
  std::string temp_str;
  for (int n = 0; n < 5; n++) {
    file >> temp_str;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 4; j++) {
        file >> temp_str;
        calib[n](i, j) = std::stof(temp_str);
      }
    }
  }
  file.close();

  return calib;
}

template <typename T>
float pointDepth(T p) {
  float depth = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
  return depth;
}

// 去除离散点云数据
template <class T>
void depthCrop(T pc, float d) {
  int i = 0;
  while (i < pc->size()) {
    if (pointDepth(pc->points[i]) > d) {
      pc->erase(pc->begin() + i);
    } else {
      i++;
    }
  }
}

void readPointCloud(std::string fileName,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr pc) {
  if (fileName.substr(fileName.length() - 4, 4) == ".bin" ||
      fileName.substr(fileName.length() - 4, 4) == ".BIN") {
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(fileName, *pc) != 0) {
      return;
    }
  } else if (fileName.substr(fileName.length() - 4, 4) == ".pcd" ||
             fileName.substr(fileName.length() - 4, 4) == ".PCD") {
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(fileName, *pc) != 0) {
      return;
    }
  } else if (fileName.substr(fileName.length() - 4, 4) == ".ply" ||
             fileName.substr(fileName.length() - 4, 4) == ".PLY") {
    if (pcl::io::loadPLYFile<pcl::PointXYZI>(fileName, *pc) != 0) {
      return;
    }
  }

  return;
}

// 获取VELODYNE BIN文件中所包含的点数
int getBinSize(std::string path) {
  int size = 0;
  FILE *fp = fopen(path.c_str(), "rb");
  if (fp) {
    fseek(fp, 0, SEEK_END);
    size = ftell(fp);
    fclose(fp);
  }
  size = size / (int)sizeof(float) / 4;
  return size;
}

// 读取KITTI VELODYNE点云(单帧数据)
Eigen::MatrixXf readBin(std::string path, int size) {
  Eigen::MatrixXf pc(size, 4);
  std::ifstream velodyne_bin(path, std::ios::binary);
  for (int i = 0; i < size; i++) {
    for (int j = 0; j < 4; j++) {
      float data;
      velodyne_bin.read((char *)&data, sizeof(float));
      pc(i, j) = data;
    }
  }
  velodyne_bin.close();
  return pc;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr readPointCloudKITTI(std::string fileName) {
  Eigen::MatrixXf pc = readBin(fileName, getBinSize(fileName));
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  for (int i = 0; i < pc.rows(); i++) {
    pcl::PointXYZI temp;
    temp.x = pc(i, 0);
    temp.y = pc(i, 1);
    temp.z = pc(i, 2);
    cloud->push_back(temp);
  }
  return cloud;
}

// TODO 修改为XYZRGB
pcl::PointCloud<pcl::PointXYZRGB>::Ptr readPointCloudKITTIWithRGB(
    std::string lidar_fileName, std::string img_fileName,
    Eigen::Matrix<float, 4, 4> calib_lidar,
    Eigen::Matrix<float, 4, 4> calib_camera) {
  // 读取点云帧数据
  Eigen::MatrixXf pc = readBin(lidar_fileName, getBinSize(lidar_fileName));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  for (int i = 0; i < pc.rows(); i++) {
    pcl::PointXYZRGB temp;
    temp.x = pc(i, 0);
    temp.y = pc(i, 1);
    temp.z = -pc(i, 2);
    cloud->push_back(temp);
  }

  // 读取图像数据
  cv::Mat img = cv::imread(img_fileName);
  int rows = img.rows;
  int cols = img.cols;

  // 进行点云染色
  Eigen::Matrix<float, 4, 4> trans = calib_camera * calib_lidar;
  Eigen::Matrix<float, 4, 1> point_tmp;
  Eigen::Matrix<float, 1, 4> p_result;

  for (int nIndex = 0; nIndex < cloud->points.size(); nIndex++) {
    float point_x = cloud->points[nIndex].x;
    float point_y = cloud->points[nIndex].y;
    float point_z = -cloud->points[nIndex].z;

    point_tmp = Eigen::Matrix<float, 4, 1>{point_x, point_y, point_z, 1};
    p_result = trans * point_tmp;

    float p_w = p_result(0, 2);
    float p_u = (int)((p_result(0, 0)) / p_w);
    float p_v = (int)((p_result(0, 1)) / p_w);
    if ((p_u >= 0) && (p_u <= cols) && (p_v >= 0) && (p_v <= rows) &&
        (p_w >= 0)) {
      cloud->points[nIndex].b =
          img.at<cv::Vec3b>(p_v, p_u)[0];  // not (p_u,p_v)!
      cloud->points[nIndex].g = img.at<cv::Vec3b>(p_v, p_u)[1];
      cloud->points[nIndex].r = img.at<cv::Vec3b>(p_v, p_u)[2];
    }
  }

  return cloud;
}

Eigen::Matrix<float, 4, 4> inversePoseT(Eigen::Matrix<float, 4, 4> pose) {
  pose.block<3, 3>(0, 0) = pose.block<3, 3>(0, 0).transpose();
  pose.block<3, 1>(0, 3) = pose.block<3, 3>(0, 0) * pose.block<3, 1>(0, 3);
  return pose;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr fastMerge(
    std::vector<std::string> lidar_files,
    std::vector<Eigen::Matrix<float, 4, 4>> poses,
    Eigen::Matrix<float, 4, 4> calib_lidar) {
  assert(poses.size() == lidar_files.size());

  // 初始化，以第一帧的位姿为起始位姿
  Eigen::Matrix<float, 4, 4> init_pose_inv = inversePoseT(poses[0]);
  pcl::PointCloud<pcl::PointXYZI>::Ptr global_map =
      readPointCloudKITTI(lidar_files[0]);
  depthCrop(global_map, 50.0);
  pcl::transformPointCloud(*global_map, *global_map, calib_lidar);

  for (int i = 1; i < lidar_files.size(); i++) {
    // 采样
    if (i % 5 != 0) {
      continue;
    }
    std::cout << i << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp =
        readPointCloudKITTI(lidar_files[i]);
    depthCrop(cloud_temp, 50.0);
    for (int j = 0; j < cloud_temp->size(); j++) {
      cloud_temp->points[j].intensity = j / 10;
    }
    Eigen::Matrix<float, 4, 4> trans = init_pose_inv * poses[i] * calib_lidar;
    pcl::transformPointCloud(*cloud_temp, *cloud_temp, trans);
    *global_map += *cloud_temp;
  }

  pcl::VoxelGrid<pcl::PointXYZI> sor;
  float leaf_size = 0.5f;
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.setInputCloud(global_map);
  sor.filter(*global_map);

  return global_map;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr fastMergeWithImg(
    std::vector<std::string> lidar_files, std::vector<std::string> img_files,
    std::vector<Eigen::Matrix<float, 4, 4>> poses,
    Eigen::Matrix<float, 4, 4> calib_lidar,
    Eigen::Matrix<float, 4, 4> calib_camera) {
  assert(poses.size() == lidar_files.size());

  // 初始化，以第一帧的位姿为起始位姿
  Eigen::Matrix<float, 4, 4> init_pose_inv = inversePoseT(poses[0]);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map =
      readPointCloudKITTIWithRGB(lidar_files[0], img_files[0], calib_lidar,
                                 calib_camera);
  // depthCrop(global_map, 50.0);
  pcl::transformPointCloud(*global_map, *global_map, calib_lidar);

  for (int i = 1; i < lidar_files.size(); i++) {
    // 采样
    if (i % 5 != 0) {
      continue;
    }
    std::cout << i << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp =
        readPointCloudKITTIWithRGB(lidar_files[i], img_files[i], calib_lidar,
                                   calib_camera);
    // depthCrop(cloud_temp, 50.0);
    Eigen::Matrix<float, 4, 4> trans = init_pose_inv * poses[i] * calib_lidar;
    pcl::transformPointCloud(*cloud_temp, *cloud_temp, trans);
    *global_map += *cloud_temp;
  }

  // pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  // float leaf_size = 0.5f;
  // sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  // sor.setInputCloud(global_map);
  // sor.filter(*global_map);

  return global_map;
}

int main() {
  bool isRGBcloud = true;

  std::string velodyne_path =
      "/home/yjq/dataset/KITTI_odometry/sequences/07/velodyne";
  std::string pose_path =
      "/home/yjq/dataset/KITTI_odometry/sequences/07/poses.txt";
  std::string calib_path =
      "/home/yjq/dataset/KITTI_odometry/sequences/07/calib.txt";
  std::string img_path =
      "/home/yjq/dataset/KITTI_odometry/sequences/07/image_3";

  std::vector<std::string> lidar_files = pathList(velodyne_path, true);
  std::vector<std::string> img_files = pathList(img_path, true);
  std::vector<Eigen::Matrix<float, 4, 4>> poses = readPose_KITTI(pose_path);
  std::vector<Eigen::Matrix<float, 3, 4>> calib = readCalib(calib_path);
  Eigen::Matrix<float, 4, 4> calib_lidar;
  Eigen::Matrix<float, 4, 4> calib_camera;
  // Tr_velo_to_cam Matrix
  calib_lidar.block<3, 4>(0, 0) = calib[4];
  calib_lidar.block<1, 4>(3, 0) = Eigen::Matrix<float, 1, 4>{0, 0, 0, 1};
  // P3 Matrix
  calib_camera.block<3, 4>(0, 0) = calib[3];
  calib_camera.block<1, 4>(3, 0) = Eigen::Matrix<float, 1, 4>{0, 0, 0, 1};
  
  // MergeWithImg
  if (isRGBcloud) {
    std::cout<<"123"<<std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map = fastMergeWithImg(
        lidar_files, img_files, poses, calib_lidar, calib_camera);
    std::cout<<"123"<<std::endl;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    for (int i = 0; i < global_map->size(); i++) {
      if (global_map->points[i].r == 0 && global_map->points[i].g == 0 &&
          global_map->points[i].b == 0) {
        inliers->indices.push_back(i);
      }
    }
    extract.setInputCloud(global_map);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*global_map);
    std::cout<<"123"<<std::endl;
    pcl::visualization::CloudViewer viewer("cloud view");
    viewer.showCloud(global_map);
    while (!viewer.wasStopped()) {
    }

    pcl::io::savePCDFileASCII("../result/kitti.pcd", *global_map);

  } else {
    // Merge pcd
    pcl::PointCloud<pcl::PointXYZI>::Ptr global_map =
        fastMerge(lidar_files, poses, calib_lidar);

    // Modify point cloud coordinate system
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
    transformation_matrix << 1.0, 0.0, 0.0, 0.0, 
                            0.0, 0.0, 1.0, 0.0, 
                            0.0, -1.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0, 1.0;
    pcl::transformPointCloud(*global_map, *global_map, transformation_matrix);

    pcl::visualization::CloudViewer viewer("cloud view");
    viewer.showCloud(global_map);
    while (!viewer.wasStopped()) {
    }

    pcl::io::savePCDFileASCII("../result/result.pcd", *global_map);
  }
  return 0;
}
