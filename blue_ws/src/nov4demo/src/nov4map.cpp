#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "ANS/SLAM.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud2.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

//POSE at a certain time
//TODO remove these, if we can get pose at the same time as map
double odom_x=0;
double odom_y=0;
double odom_theta_z=0;
//parameters for published map
double resolution = 0.5;
int32_t map_width = 51;
int32_t map_height = 51;
int32_t robot_r = map_height-2;
int32_t robot_c = map_width/2;
float obstacle_height = 8;
//TODO set these
double displacement_to_robot_x = 0;
double displacement_to_robot_y = 0;
double theta_to_robot = 0;



ros::Publisher test_pub;

//TODO code this method to get the index at which to place a point, given stuff.
//PRE:points are oriented in correct rotational direction/correct frame
int32_t get_index(float_t ground_x, float_t ground_y)
{
        double rotated_x = ground_x*cos(theta_to_robot) - ground_y*sin(theta_to_robot);
	double rotated_y = ground_x*sin(theta_to_robot) + ground_y*cos(theta_to_robot);
	int32_t c = robot_c + (ground_x - displacement_to_robot_x)/resolution;
        int32_t r = robot_r + (ground_y - displacement_to_robot_y)/resolution;
	return r*map_width+c; //TODO double chech that this is the correct array math
}

//int * map;

/*TODO fix the synchronization, and actually get theta to happen, TODO both in robot pose and origin of map*/
void stereoCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	        odom_x = (msg->pose).pose.position.x;
		odom_y = (msg->pose).pose.position.y;
		geometry_msgs::Quaternion q= (msg->pose).pose.orientation;
		// yaw (z-axis rotation) help from:
		// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
		double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
		double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
		double yaw = atan2(siny_cosp, cosy_cosp);
		odom_theta_z = (msg->pose).pose.position.z;
}


//TODO create a callback that updates the theta and displacement



/**
 *  *  * This if derived from the tutorial that demonstrates simple receipt of messages over the ROS system.
 *   *   */
void mapCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{

	//TODO create two maps: one for number of points in cell, one for actual height data.
	float_t map_heights[map_width*map_height] = { };
	float_t map_counts[map_width*map_height] = { };

	for(int i = 0; i<map_width*map_height; i++)
	{
		map_heights[i]=0;
		map_counts[i]=0;
	}

	float_t max_height_magnitude=0.001;
	
	//code help from https://gist.github.com/bricerebsamen/a74fc627390d4c685012
	//and https://github.com/yujinrobot/kobuki/blob/indigo/kobuki_bumper2pc/src/kobuki_bumper2pc.cpp
	//https://medium.com/@jeehoahn/some-ideas-on-pointcloud2-data-type-1d1ae940ef9b
	int32_t x0,x1,x2,x3,y0,y1,y2,y3,z0,z1,z2,z3,x,y,z,index,counts,used_pts=0,all_pts=0;
	float_t xf, yf, zf;
	for (size_t i = 0; i < msg->height * msg->width; i+=msg->point_step)
	{
		all_pts++;
		//get :the four bytes making up the float for each of the x,y,z
		x0=msg->data[i+msg->fields[0].offset];
                x1=msg->data[i+msg->fields[0].offset+1];
                x2=msg->data[i+msg->fields[0].offset+2];
                x3=msg->data[i+msg->fields[0].offset+3];

                y0=msg->data[i+msg->fields[1].offset];
                y1=msg->data[i+msg->fields[1].offset+1];
                y2=msg->data[i+msg->fields[1].offset+2];
                y3=msg->data[i+msg->fields[1].offset+3];

	        z0=msg->data[i+msg->fields[2].offset];
                z1=msg->data[i+msg->fields[2].offset+1];
                z2=msg->data[i+msg->fields[2].offset+2];
                z3=msg->data[i+msg->fields[2].offset+3];

		//create the bit representation of the float depending on endianness
		if(msg->is_bigendian)
		{
			x=x3 | (x2<<8) | (x1<<16) | (x0<<24);
                        y=y3 | (y2<<8) | (y1<<16) | (y0<<24);
                        z=z3 | (z2<<8) | (z1<<16) | (z0<<24);
		}
		else
		{
		        x=x0 | (x1<<8) | (x2<<16) | (x3<<24);
                        y=z0 | (y1<<8) | (y2<<16) | (y3<<24);
                        z=z0 | (z1<<8) | (z2<<16) | (z3<<24);
		}

		memcpy(&xf, (const void *)&(x), 4);
                memcpy(&yf, (const void *)&(y), 4);
                memcpy(&zf, (const void *)&(z), 4);


 		
		
		//get the index on the map that the point should be added to 
		index = get_index(xf,yf);
		float_t h =0;
		//add the point to the map after averaging with those in cell
		if(index<map_width*map_height)
		{
			counts = map_counts[index];
			map_heights[index]=((map_heights[index]*counts) + zf)/(counts+1);
			map_counts[index]=counts+1;
			used_pts++;
		}
	
		ROS_INFO("X:%x Y:%f Z:%d\n H:%f AP:%d UP:%d",x,yf,zf,msg->fields[2].offset,all_pts,used_pts);

	}
	
	//TODO make these be probabilities
	int8_t map[map_width*map_height]={};
	int8_t normd=0;
 	for(int i =0; i<map_width*map_height;i++)
	{
		map[i]=-1;
		if(map_heights[i]>obstacle_height || map_heights[i]<obstacle_height*(-1))
			map[i]=100;
		else
		{
			normd = 100*(map_heights[i]/obstacle_height);
			if(normd<0)
				normd*=-1;
			if(map_counts[i]==0)
				map[i]=-1;
			else
				map[i]=normd;

		}
	}	
	//ros::Publisher test_pub = n.advertise<ANS::SLAM>("gameplan", 400); //publishing to "gameplan" topic
	nav_msgs::OccupancyGrid mesg; 
	nav_msgs::MapMetaData mmd;
	//A SLAM message is comprised of an Occupancy Grid message and two, 2D Pose messages for goal-coordinates. In terminal, run: rosmsg show SLAM      for greater detail
	mmd.width = map_width; //grid dimension WIDTH
    	mmd.height = map_height; //grid dimension HEIGHT
	mmd.origin.position.x = robot_c; //starting position X-Coordinate
	mmd.origin.position.y = robot_r; //starting position Y-Coordinate
	
	mesg.info=mmd;
	mesg.header=msg->header;
	std::vector<signed char> a(map, map+(map_width*map_height));
	mesg.data = a;
	
	//ROS_INFO("startX: %f", mesg.mapMatrix.info.origin.position.x);
	//ROS_INFO("goalX: %f", mesg.goalCoords.x);  
        //ROS_INFO("stereoX: %f", stereoX);

	//ROS_INFO("dataPoint: %d", mesg.mapMatrix.data[startX+goalY*w]);                                                    
	test_pub.publish(mesg);

}




int main(int argc, char **argv)
{

  ros::init(argc, argv, "testPublisher"); //initiate publisher named testPublisher

  ros::NodeHandle n;

  test_pub = n.advertise<nav_msgs::OccupancyGrid>("gameplan", 101); //publishing to "gameplan" topic

  ros::Subscriber sub = n.subscribe("points2", 1000, mapCallback);
  //ros::Subscriber stereo_sub = n.subscribe ("stereo_odometry", 1000, stereoCallback); //change to stereo_odometry for demo_stereo_outdoor.launch

  /*int count = 1;
  while (ros::ok())
  {

        ANS::SLAM msg; //A SLAM message is comprised of an Occupancy Grid message and two, 2D Pose messages for goal-coordinates. In terminal, run: rosmsg show SLAM      for greater detail
	msg.mapMatrix.info.width = w; //grid dimension WIDTH
	msg.mapMatrix.info.height = h; //grid dimension HEIGHT
	msg.mapMatrix.info.origin.position.x = x; //starting position X-Coordinate
	msg.mapMatrix.info.origin.position.y = y; //starting position Y-Coordinate
	//TODO include z
	msg.goalCoords.x = 19.0; //goal position X-Coordinate
	msg.goalCoords.y = 0.0; //goal position Y-Coordinate
	uint64_t len = w*h;
        int8_t mapy[len];
    	for (int i = 0; i < w*h; i++)
	{
		mapy[i]=1;
	}
	msg.mapMatrix.data = mapy;

    ROS_INFO("Published %d", count);
    ROS_INFO("startX: %f", msg.mapMatrix.info.origin.position.x);
    ROS_INFO("goalY: %f", msg.goalCoords.y);
    ROS_INFO("dataPoint: %d", msg.mapMatrix.data[12]);

    test_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }*/
  ros::spin();


  return 0;
}
