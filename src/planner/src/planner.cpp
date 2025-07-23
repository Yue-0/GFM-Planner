/* @author: YueLin */

#include <tf/tf.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/UInt64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

#include "planner/astar.hpp"
#include "planner/minco.hpp"

int main(int argc, char* argv[])
{
    /* Initialize */
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh("~");
    ros::Time::init();

    /* Constants */
    std::string png;
    nh.getParam("metric", png);
    const int S = nh.param("minco_s", 3);
    cv::Mat mat = cv::imread(png, cv::IMREAD_UNCHANGED);
    const int H = mat.rows, W = mat.cols; mat.release();
    const double REPLAN = nh.param("replan_interval", 0.1);
    const double ERR[3] = {
        nh.param("err_dist", 0.1), 
        nh.param("err_angle", 0.1),
        nh.param("control_accuracy", 0.1)
    };
    const double DT = 1 / nh.param("control_hz", 50.0);
    const double SAFE = nh.param("safe_distance", 0.4) / 2;

    /* Variables */
    SE2 start, end;
    bool plan = false;
    float resolution = 0;
    Eigen::MatrixXd states[2] = {
        Eigen::MatrixXd::Zero(3, S),
        Eigen::MatrixXd::Zero(3, S)
    };

    /* Objects */
    ESDF esdf(H, W);
    MINCO minco(S, 3);
    Trajectory trajectory(S);
    cv::Mat map(H, W, CV_8UC1);
    geometry_msgs::Twist velocity;
    gfm_planner::PerceptionAwareAStar planner(
        1 - nh.param("blind", 0.75), 
        nh.param("alpha", 1), H, W
    );
    gfm_planner::GeometricFeatureMetric gfm(
        H, W, nh.param("epsilon", 1.0)
    );
    gfm_planner::PerceptionAwareOptimizer optimizer(
        &gfm, &minco, &esdf,
        nh.param("safe_distance", 0.4),
        nh.param("max_duration", 1.0),
        1 - nh.param("blind", 0.75),
        
        nh.param("lbfgs_past", 3),
        nh.param("lbfgs_memory", 8),
        nh.param("lbfgs_iterations", 0x40),
        nh.param("lbfgs_kappa", 16),
        nh.param("lbfgs_eps", 1e-6),
        nh.param("lbfgs_step", 1e20),
        nh.param("lbfgs_delta", 1e-6),
        nh.param("lbfgs_epsilon", 1e-5),
        nh.param("lbfgs_wolfe", 9e-1),
        nh.param("lbfgs_armijo", 1e-4),
        
        nh.param("lambda_s", 1.0),
        nh.param("lambda_t", 20.0),
        nh.param("lambda_l", 1.0),
        nh.param("lambda_p", 1e4),
        
        nh.param("max_vel", 1.0),
        nh.param("max_acc", 1.0),
        nh.param("max_omega", 1.0),
        nh.param("max_alpha", 1.0)
    );
    
    /* Frames */
    std::string odom;
    nav_msgs::Path path;
    nh.getParam("odom_frame", odom);
    nh.getParam("map_frame", path.header.frame_id);

    /* Wait tf */
    double times[2] = {0, 0};
    tf::StampedTransform transform;
    tf::TransformListener listener;
    while(!listener.canTransform(odom, path.header.frame_id, ros::Time(0)))
    {
        if(times[0] >= 60) throw "Transfrom error!";
        listener.waitForTransform(
            odom, path.header.frame_id, ros::Time(0), ros::Duration(10)
        );
        times[0] += 10;
    }

    /* Publishers */
    ros::Publisher 
    control = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1),
    visualizer = nh.advertise<nav_msgs::Path>("/trajectory", 1);

    /* Subscribe navigation goal */
    ros::Subscriber goal = nh.subscribe<geometry_msgs::PoseStamped>(
        "/move_base_simple/goal", 1, [&gfm, &end, &planner, &plan]
        (geometry_msgs::PoseStamped::ConstPtr pose){
            if(zero(gfm.resolution)) return;
            end.set(
                pose->pose.position.x,
                pose->pose.position.y,
                tf::getYaw(pose->pose.orientation)
            );
            planner.heuristic(end, &gfm); plan = true;
        }
    );

    /* Subscribe position */
    ros::Subscriber localization = nh.subscribe<geometry_msgs::PoseStamped>(
        "/position", 1, [&start](geometry_msgs::PoseStamped::ConstPtr pose){
            start.set(
                pose->pose.position.x,
                pose->pose.position.y,
                tf::getYaw(pose->pose.orientation)
            );
        }
    );

    /* Subscribe costmap */
    ros::Subscriber scan = nh.subscribe<nav_msgs::OccupancyGrid>(
        "/costmap", 1, [&resolution, &H, &W, &map]
        (nav_msgs::OccupancyGrid::ConstPtr grid)
        {
            resolution = grid->info.resolution;
            for(int y = 0; y < H; y++)
            {
                int z = y * W;
                for(int x = 0; x < W; x++)
                    map.at<uchar>(y, x) = grid->data[z + x];
            }
        }
    );

    /* Subscribe ESDF */
    ros::Subscriber sdf = nh.subscribe<std_msgs::Float32MultiArray>(
        "/esdf", 1, [&resolution, &esdf]
        (std_msgs::Float32MultiArray::ConstPtr array){
            if(zero(resolution)) return;
            esdf.update(resolution, array->data.data());
        }
    );

    /* Subscribe metric */
    ros::Subscriber metric = nh.subscribe<std_msgs::UInt64MultiArray>(
        "/metric", 1, [&resolution, &gfm]
        (std_msgs::UInt64MultiArray::ConstPtr array){
            if(zero(resolution)) return;
            gfm.update(resolution, array->data.data());
        }
    );

    /* Main loop */
    ros::Timer planning = nh.createTimer(
        ros::Duration(REPLAN), [
            &REPLAN, &plan, &resolution, &times, &ERR, &start, &end, &states,
            &map, &gfm, &planner, &optimizer, &trajectory, &path, &visualizer
        ](const ros::TimerEvent&){

            /* Wait for initialization */
            if(!plan || (zero(start.x) && zero(start.y)) ||
               zero(resolution) || zero(gfm.resolution)) return;
            plan = false;
            path.poses.clear();
            trajectory.clear();
            
            /* If no planning required */
            if(std::hypot(start.x - end.x, start.y - end.y) < ERR[0] && std::
               fabs(PI - std::fabs(clip(start.yaw - end.yaw) - PI)) < ERR[1])
            {
                path.header.stamp = ros::Time::now();
                visualizer.publish(path);
                return;
            }

            /* Percepotion-aware trajectory planning */
            ROS_INFO("Start planning.");
            ros::Time time = ros::Time::now();
            std::vector<SE2> points = planner.plan(&gfm, map, start, end);
            times[0] = (ros::Time::now() - time).toSec();
            gfm_planner::sampling(gfm.resolution, map, points);
            if(optimizer.setup(points, states))
            {
                times[1] = ros::Time::now().toSec();
                optimizer.optimize(&trajectory);
                times[1] = ros::Time::now().toSec() - times[1];
            }
            else
                ROS_WARN("Planning failed");
            path.header.stamp = time;
            ROS_INFO("Planning completed.");
            ROS_INFO("Path time: %fs", times[0]);
            ROS_INFO("Optimization time: %fs.", times[1]);

            /* Publish trajectory */
            double duration = trajectory.duration();
            for(double t = 0; t < duration; t += REPLAN * 2.5)
            {
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = path.header.stamp;
                Eigen::VectorXd pos = trajectory.pos(t);
                pose.header.frame_id = path.header.frame_id;
                pose.pose.orientation = tf::createQuaternionMsgFromYaw(pos.z());
                pose.pose.position.x = pos.x();
                pose.pose.position.y = pos.y();
                path.poses.push_back(pose);
            }
            visualizer.publish(path);
        }
    );

    ros::Timer controller = nh.createTimer(
        ros::Duration(DT), [
            &trajectory, &start, &velocity, &path, &control, &states
        ](const ros::TimerEvent&){
            Eigen::Vector3d vel;
            double t = (ros::Time::now() - path.header.stamp).toSec();
            if(t > trajectory.duration() || t < 0)
                vel.setZero();
            else
                vel = trajectory.vel(t);
            states[0].col(1) = vel;
            double sin = std::sin(start.yaw), cos = std::cos(start.yaw);
            velocity.linear.x = cos * vel.x() + sin * vel.y();
            velocity.linear.y = cos * vel.y() - sin * vel.x();
            velocity.angular.z = vel.z();
            control.publish(velocity);
        }
    );

    ros::Timer replanning = nh.createTimer(
        ros::Duration(REPLAN), [
            &plan, &REPLAN, &DT, &SAFE, &ERR, &start, &esdf, &trajectory, &path
        ](const ros::TimerEvent&){
            double duration = trajectory.duration();
            double t = (ros::Time::now() - path.header.stamp).toSec();
            if(t > duration || t < 0) return;
            Eigen::VectorXd pos = trajectory.pos(t);
            if(std::hypot(start.x - pos[0], start.y - pos[1]) > ERR[2])
            {
                plan = true; return;
            }
            for(int i = 1; i <= REPLAN / DT; i++)
            {
                pos = trajectory.pos(t + i * DT);
                if(esdf.get(pos) < SAFE)
                {
                    plan = true; return;
                }
            }
        }
    );

    return ros::spin(), 0;
}
