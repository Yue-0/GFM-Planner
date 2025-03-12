/* @author: YueLin */

#include <tf/tf.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/UInt64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

#include "utils/bfs.hpp"
#include "utils/popcount.h"

int main(int argc, char* argv[])
{
    /* Initialize */
    ros::init(argc, argv, "map");
    ros::NodeHandle nh("~");

    /* Hyper-parameters */
    float scale;
    float rect = nh.param("expansion", 0.4);
    ros::Rate sleep(nh.param("update_rate", 10));
    const uchar T = std::max(
        nh.param("attenuation", 0.25) * nh.param("update_rate", 10), 1.
    );

    /* Metric */
    std::string png;
    nh.getParam("metric", png);
    cv::Mat q = cv::imread(png, cv::IMREAD_UNCHANGED); cv::flip(q, q, 0);

    /* Constants */
    const int H = q.rows, W = q.cols;

    /* Mats */
    cv::Mat global(H, W, CV_8UC1, 0xFF);
    cv::Mat local(H, W, CV_8UC1, 0xFF);
    cv::Mat attenuator = local * 0x0;
    cv::Mat df, sdf;

    /* Messages */
    sensor_msgs::LaserScan lidar;
    nav_msgs::OccupancyGrid visual;
    nav_msgs::OccupancyGrid costmap;
    std_msgs::Float32MultiArray esdf;
    std_msgs::UInt64MultiArray metric;
    geometry_msgs::PointStamped laser, obstacle;
    laser.header.stamp = obstacle.header.stamp = ros::Time(0);

    /* Initialize messages */
    visual.info.width = W;
    visual.info.height = H;
    costmap.info.width = W;
    costmap.info.height = H;
    esdf.data.resize(W * H);
    metric.data.resize(W * H);
    visual.data.resize(W * H);
    costmap.data.resize(W * H);
    esdf.layout.data_offset = W;
    metric.layout.data_offset = W;

    /* Frames */
    std::string robot;
    tf::StampedTransform transform;
    tf::TransformListener listener;
    nh.getParam("robot_frame", robot);

    /* Publishers */
    ros::Publisher
    gfm = nh.advertise<std_msgs::UInt64MultiArray>("/metric", 1),
    visualizer = nh.advertise<nav_msgs::OccupancyGrid>("/gfm", 1),
    distance = nh.advertise<std_msgs::Float32MultiArray>("/esdf", 1),
    publisher = nh.advertise<nav_msgs::OccupancyGrid>("/costmap", 1);

    bool ready[2] = {false, false};

    /* Subscribe laser scan */
    ros::Subscriber subscriber = nh.subscribe<sensor_msgs::LaserScan>(
        "/laser", 1, [&lidar, &ready, &laser]
        (sensor_msgs::LaserScan::ConstPtr scan)
        {
            lidar = *scan;
            laser.header.frame_id = lidar.header.frame_id;
            if(!ready[0]) ready[0] = true;
        }
    );

    /* Subscribe map */
    ros::Subscriber grid = nh.subscribe<nav_msgs::OccupancyGrid>(
        "/map", 1, [&](nav_msgs::OccupancyGrid::ConstPtr og){
            if(ready[1]) return;

            /* Get map infomation */
            scale = 1 / og->info.resolution;
            rect = std::round(rect * scale);

            /* Initialize global map */
            for(int y = 0; y < H; y++)
                for(int x = 0; x < W; x++)
                    if(og->data[x + y * W])
                        global.at<uchar>(y, x) = 0;
            cv::erode(global, global, cv::getStructuringElement(
                cv::MORPH_RECT, cv::Size(rect / 2, rect / 2)
            ));

            /* Initialize metric */
            for(int y = 0; y < H; y++)
                for(int x = 0; x < W; x++)
                {
                    int z = x + y * W;
                    metric.data[z] = 0;
                    for(int channel = 3; channel >= 0; channel--)
                    {
                        metric.data[z] += q.at<cv::Vec4w>(y, x)[channel];
                        if(channel) metric.data[z] <<= 16;
                    }
                    visual.data[z] = 1.5625 * popcount(metric.data[z]);
                }
            
            /* Initialize messages */
            visual.info.resolution = og->info.resolution;
            visual.header.frame_id = og->header.frame_id;
            costmap.info.resolution = og->info.resolution;
            costmap.header.frame_id = og->header.frame_id;
            obstacle.header.frame_id = og->header.frame_id;
            
            /* Wait transform */
            listener.waitForTransform(
                og->header.frame_id, robot, ros::Time(0), ros::Duration(100)
            );

            ready[1] = true;
        }
    );

    /* Publish metric */
    ros::Timer timer = nh.createTimer(
        ros::Duration(1), [&](const ros::TimerEvent&){
            if(!(ready[0] && ready[1])) return;
            visualizer.publish(visual);
            gfm.publish(metric);
        }
    );

    /* Main loop */
    while(ros::ok())
    {
        /* Lidar callback */
        ros::spinOnce();
        if(!(ready[0] && ready[1])) continue;

        /* Attenuate */
        for(int y = 0; y < H; y++)
            for(int x = 0; x < W; x++)
                if(attenuator.at<uchar>(y, x))
                    --attenuator.at<uchar>(y, x);
        cv::rectangle(attenuator, cv::Rect(0, 0, W, H), T, rect / 4);

        /* Add obstacles */
        float yaw = lidar.angle_min;
        for(float d: lidar.ranges)
        {
            if(d < lidar.range_max && d >= lidar.range_min)
            {
                laser.point.x = d * std::cos(yaw);
                laser.point.y = d * std::sin(yaw);
                listener.transformPoint(
                    costmap.header.frame_id, laser, obstacle
                );
                int x = std::round(obstacle.point.x * scale);
                int y = std::round(obstacle.point.y * scale);
                if(std::min(x, y) >= 0 && x < W && y < H)
                    attenuator.at<uchar>(y, x) = T;
            }
            yaw += lidar.angle_increment;
        }

        /* Build map */
        local *= 0;
        for(int y = 0; y < H; y++)
            for(int x = 0; x < W; x++)
                if(attenuator.at<uchar>(y, x) || !global.at<uchar>(y, x))
                    local.at<uchar>(y, x) = 0xFF;

        /* Build ESDF */
        cv::distanceTransform(
            local, sdf, cv::DIST_L2, cv::DIST_MASK_PRECISE
        );
        cv::distanceTransform(
            0xFF - local, df, cv::DIST_L2, cv::DIST_MASK_PRECISE
        );

        /* Update map */
        for(int y = 0; y < H; y++)
            for(int x = 0; x < W; x++)
            {
                float d = 0x32 * df.at<float>(y, x) / rect;
                local.at<uchar>(y, x) = d > 0x32? 0: 0x64 - d;
            }
        
        /* Get robot's position */
        listener.lookupTransform(
            costmap.header.frame_id, robot, ros::Time(0), transform
        );
        int xs = std::round(scale * transform.getOrigin().x());
        int ys = std::round(scale * transform.getOrigin().y());
        xs = std::min(std::max(xs, 0), W - 1);
        ys = std::min(std::max(ys, 0), H - 1);

        /* Prevent obstacles from expanding into the robot */
        if(local.at<uchar>(ys, xs))
        {
            std::pair<int, int> p = bfs(local, xs, ys);
            int x = p.first, y = p.second;
            if(xs > x) {x += xs; xs = x - xs; x -= xs;}
            if(ys > y) {y += ys; ys = y - ys; y -= ys;}
            cv::rectangle(
                local, cv::Point(xs, ys), cv::Point(x + 1, y + 1), 0, -1
            );
        }

        /* Update costmaps */
        for(int y = 0; y < H; y++)
        {
            int z = y * costmap.info.width;
            for(int x = 0; x < W; x++)
            {
                if((esdf.data[z + x] = df.at<float>(y, x)) < 1e-9)
                    esdf.data[z + x] = -sdf.at<float>(y, x);
                esdf.data[z + x] *= costmap.info.resolution;
                costmap.data[z + x] = local.at<uchar>(y, x);
            }
        }

        /* Publish costmap */
        distance.publish(esdf);
        publisher.publish(costmap);

        /* Sleep */
        sleep.sleep();
    }

    return 0x0;
}