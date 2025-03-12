/* @author: YueLin */

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/PointCloud2.h"

int main(int argc, char* argv[])
{
    /* Inintialize */
    ros::init(argc, argv, "map");
    ros::NodeHandle nh("~");

    /* Hyper-parameters */
    std::string frame, topic;
    nh.getParam("frame", frame);
    nh.getParam("topic", topic);
    const float height = nh.param("height", 1.0);

    /* Point cloud */
    sensor_msgs::PointCloud2 pc;

    /* Subscriber */
    ros::Subscriber subscriber = nh.subscribe<nav_msgs::OccupancyGrid>(
        topic, 1, [&height, &pc, &frame](nav_msgs::OccupancyGrid::ConstPtr msg)
        {
            pcl::PointCloud<pcl::PointXYZ> cloud;
            const int Z = height / msg->info.resolution;
            const int X = msg->info.width, Y = msg->info.height;

            /* OccupancyGrid -> Point cloud */
            for(int y = 0; y < Y; y++)
                for(int x = 0; x < X; x++)
                    if(msg->data[x + y * X])
                        for(int z = 0; z < Z; z++)
                            cloud.push_back(pcl::PointXYZ(
                                x * msg->info.resolution,
                                y * msg->info.resolution,
                                z * msg->info.resolution
                            ));
            cloud.height = 1;
            cloud.is_dense = true;
            cloud.width = cloud.points.size();

            /* Point cloud -> Ros message */
            pcl::toROSMsg(cloud, pc);
            pc.header.frame_id = frame;
        }
    );

    /* Publisher */
    ros::Publisher visualizer = nh.advertise<sensor_msgs::PointCloud2>(
        "/map3D", 1
    );

    /* Publish point cloud */
    ros::Timer publisher = nh.createTimer(
        ros::Duration(nh.param("time", 1)),
        [&pc, &visualizer](const ros::TimerEvent&){
            visualizer.publish(pc);
        }
    );

    /* Main loop */
    return ros::spin(), 0;
}
