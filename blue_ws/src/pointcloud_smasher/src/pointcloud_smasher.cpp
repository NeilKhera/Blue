#include <cmath> 
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

using namespace pcl;

float CAMERA_ANGLE;
float CAMERA_HEIGHT;
float CAMERA_X_LIMIT;
float ANGLE_THRESHOLD;

ros::Publisher grid_pub;

PointCloud<PointXYZRGB>::Ptr frameTransform(PointCloud<PointXYZRGB>::Ptr cloud) {
  Eigen::Affine3f transform_matrix = Eigen::Affine3f::Identity();
  transform_matrix.translation() << 0.0, 0.0, CAMERA_HEIGHT;
  transform_matrix.rotate(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitZ()));
  transform_matrix.rotate(Eigen::AngleAxisf(-(M_PI / 2) - CAMERA_ANGLE, Eigen::Vector3f::UnitX()));

  PointCloud<PointXYZRGB>::Ptr cloud_corrected (new PointCloud<PointXYZRGB>());
  transformPointCloud(*cloud, *cloud_corrected, transform_matrix);
  return cloud_corrected;
}

void downSample(PointCloud<PointXYZRGB>::Ptr cloud, double x_dim, double y_dim, double z_dim) {
  VoxelGrid<PointXYZRGB> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize(x_dim, y_dim, z_dim);
  vox.filter(*cloud);
}

PointCloud<Normal>::Ptr computeNormals(PointCloud<PointXYZRGB>::Ptr cloud, double radius) {
  PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
  search::KdTree<PointXYZRGB>::Ptr kdtree(new search::KdTree<PointXYZRGB>());

  NormalEstimation<PointXYZRGB, Normal> ne;
  ne.setInputCloud(cloud);
  ne.setViewPoint(CAMERA_HEIGHT, 0.0, 0.0);
  ne.setSearchMethod(kdtree);
  ne.setRadiusSearch(radius);
  ne.compute(*normals);

  return normals;
}

PointCloud<PointXYZI>::Ptr normalAnalysis(PointCloud<Normal>::Ptr normals, PointCloud<PointXYZRGB>::Ptr cloud) {
  PointCloud<PointXYZI>::Ptr output_cloud(new PointCloud<PointXYZI>());
  output_cloud->header.frame_id = cloud->header.frame_id;

  for (int i = 0; i < normals->points.size(); i++) {
    float dot = normals->points[i].normal_z;
    float theta = std::acos(dot);

    PointXYZRGB point = cloud->points[i];
    PointXYZI p;
    p.x = point.x;
    p.y = point.y;
    p.z = point.z;

    if (theta > ANGLE_THRESHOLD && theta < M_PI - ANGLE_THRESHOLD) {
      p.intensity = 100;
    } else if (theta < ANGLE_THRESHOLD) {
      p.intensity = (int) ((theta / ANGLE_THRESHOLD) * 100);
    } else {
      p.intensity = (int) (((M_PI - theta) / ANGLE_THRESHOLD) * 100);
    }
    output_cloud->points.push_back(p);
  }
  return output_cloud;
}

nav_msgs::OccupancyGrid getOccupancy(PointCloud<PointXYZI>::Ptr cloud) {
  int size = 100 * 100;
  std::vector<int8_t> grid;
  grid.resize(size, -1);
  std::vector<int8_t> count;
  count.resize(size, 0);

  for (int i = 0; i < cloud->points.size(); i++) {
    PointXYZI p = cloud->points[i];
    int index_row = (int) std::fmin(100.0, std::fmax(((5.0 - p.x) * 100) / 5.0, 0.0));
    int index_col = (int) std::fmin(100.0, std::fmax(((2.5 - p.y) * 100) / 5.0, 0.0));

    int index = index_row * 100 + index_col;
    if (count[index] == 0) {
      grid[index] = p.intensity;
      count[index] = 1;
    } else {
      grid[index] = ((grid[index] * count[index]) + p.intensity) / (count[index] + 1);
      count[index] = count[index] + 1;
    }
  }

  nav_msgs::OccupancyGrid og;
  og.header.seq = cloud->header.seq;
  og.header.stamp = ros::Time::now();
  og.header.frame_id = cloud->header.frame_id;
  og.info.map_load_time = og.header.stamp;
  og.info.resolution = 0.05;
  og.info.width = 100;
  og.info.height = 100;
  og.info.origin.position.x = 5;
  og.info.origin.position.y = 2.5;
  og.info.origin.position.z = 0;
  og.info.origin.orientation.x = 0;
  og.info.origin.orientation.y = 0;
  og.info.origin.orientation.z = 0;
  og.info.origin.orientation.w = 1;
  og.data = grid;

  return og;
}

void mapCallback(const PointCloud<PointXYZRGB>::ConstPtr &cloud) {
  PointCloud<PointXYZRGB>::Ptr cloud_converted(new PointCloud<PointXYZRGB>(*cloud));
  PointCloud<PointXYZRGB>::Ptr cloud_transformed = frameTransform(cloud_converted);

  downSample(cloud_transformed, 0.05, 0.05, 0.05);
  PointCloud<Normal>::Ptr cloud_normals = computeNormals(cloud_transformed, 0.08);
  PointCloud<PointXYZI>::Ptr cloud_analysed = normalAnalysis(cloud_normals, cloud_transformed);

  nav_msgs::OccupancyGrid og = getOccupancy(cloud_analysed);
  grid_pub.publish(og);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointcloud_smasher");
  ros::NodeHandle nh;

  nh.getParam("/ranger_brinkmanship/CAMERA_ANGLE", CAMERA_ANGLE);
  nh.getParam("/ranger_brinkmanship/CAMERA_HEIGHT", CAMERA_HEIGHT);
  nh.getParam("/ranger_brinkmanship/CAMERA_X_LIMIT", CAMERA_X_LIMIT);
  nh.getParam("/ranger_brinkmanship/ANGLE_THRESHOLD", ANGLE_THRESHOLD);

  ros::Subscriber sub = nh.subscribe<PointCloud<PointXYZRGB>>("/camera/depth_registered/points", 1, mapCallback); 
  grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/blue/gameplan", 1);
  
  ros::spin();
  return 0;
}
