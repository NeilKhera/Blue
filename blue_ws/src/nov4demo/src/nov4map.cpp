#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

using namespace pcl;

ros::Publisher grid_pub;

PointCloud<PointXYZRGB>::Ptr frameTransform(PointCloud<PointXYZRGB>::Ptr cloud) {
  Eigen::Affine3f transform_matrix = Eigen::Affine3f::Identity();
  transform_matrix.translation() << 0.0, 0.0, CAMERA_HEIGHT;
  transform_matrix.rotate(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitZ()));
  transform_matrix.rotate(Eigen::AngleAxisf(-(M_PI / 2) - CAMERA_ANGLE, Eigen::Vector3f::UnitX()));

  PointCloud<PointXYZRGB>::Ptr cloud_corrected(new PointCloud<PointXYZRGB>());
  transformPointCloud(*cloud, *cloud_corrected, transform_matrix);
  return cloud_corrected;
}

void downSample(PointCloud<PointXYZRGB>::Ptr cloud, double x_dim, double y_dim, double z_dim) {
  VoxelGrid<PointXYZRGB> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize(x_dim, y_dim, z_dim);
  vox.setFilterFieldName("x");
  vox.setFilterLimits(0, CAMERA_X_LIMIT);
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
      p.intensity = 1;
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
  int8_t[size] grid = {};
  int8_t[size] count = {};
  std::fill_n(grid, size, -1);

  for (int i = 0; i < cloud->points.size(); i++) {
    PointXYZI p = cloud->points[i];
    index_row = 
    index_col =

    int index = index_row * 100 + index_col;
    if (count[index] == 0) {
      grid[index] = p.intensity;
      count[index] = 1;
    } else {
      grid[index] = grid[index] + p.intensity;
      count[index] = count[index] + 1;
    }
  }

  for (int i = 0; i < size; i++) {
    if (grid[i] != -1) {
      grid[i] = grid[i] / count[i];
    }
  }

  nav_msgs::OccupancyGrid og;
  og.header = cloud->header;
  og.info.map_load_time = cloud->header.stamp;
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
  PointCloud<Normal>::Ptr cloud_normals = computeNormal(cloud_tranformed, 0.08);
  PointCloud<PointXYZI>::Ptr cloud_analysed = normalAnalysis(cloud_normals, cloud_transformed);

  nav_msgs::OccupancyGrid og = getOccupancy(cloud_analysed);
  grid_pub.publish(og);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "nov4demo");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<PointCloud<PointXYZI>>("/camera/depth_registered/points", 1, mapCallback); 
  grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/blue/gameplan", 1);
  
  ros::spin();
  return 0;
}
