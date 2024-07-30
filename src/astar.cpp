#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "astar.h"
#include "occ_map_transform.h"
#include "opencv2/opencv.hpp"

namespace py = pybind11;

nav_msgs::Path getAStarPath(const geometry_msgs::Pose2D& start, 
                            const geometry_msgs::Pose2D& goal,
                            const string& frame_id) {
    nav_msgs::Path astar_path;
    double start_time = ros::Time::now().toSec();

    // Preprocessing
    Point startpoint, targetpoint;
    vector<Point> PathList;
    Point2d cv_start = Point2d(start.x, start.y);
    OccGridParam.Map2ImageTransform(cv_start, startpoint);
    Point2d cv_goal = Point2d(goal.x, goal.y);
    OccGridParam.Map2ImageTransform(cv_goal, targetpoint);

    // Planning
    planner.PathPlanning(startpoint, targetpoint, PathList);

    if(!PathList.empty()) {
        astar_path.header.stamp = ros::Time::now();
        astar_path.header.frame_id = frame_id;
        astar_path.poses.clear();
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = astar_path.header.stamp;
        pose_stamped.header.frame_id = astar_path.header.frame_id;
        for(int i=0;i<PathList.size();i++) {
            Point2d dst_point;
            OccGridParam.Image2MapTransform(PathList[i], dst_point);

            pose_stamped.pose.position.x = dst_point.x;
            pose_stamped.pose.position.y = dst_point.y;
            pose_stamped.pose.position.z = 0;
            astar_path.poses.push_back(pose_stamped);
        }
        double end_time = ros::Time::now().toSec();
        ROS_INFO("Find a valid path successfully! Use %f s", end_time - start_time);
    } else {
        ROS_ERROR("Can not find a valid path");
    }
    return astar_path;
}

PYBIND11_MODULE(astar_wrapper, m) {
    py::class_<geometry_msgs::Pose2D>(m, "Pose2D")
        .def(py::init<>())
        .def_readwrite("x", &geometry_msgs::Pose2D::x)
        .def_readwrite("y", &geometry_msgs::Pose2D::y)
        .def_readwrite("theta", &geometry_msgs::Pose2D::theta);
    
    py::class_<nav_msgs::Path>(m, "Path")
        .def(py::init<>())
        .def_readwrite("header", &nav_msgs::Path::header)
        .def_readwrite("poses", &nav_msgs::Path::poses);
    
    m.def("getAStarPath", &getAStarPath, "A function which runs the A* Algorithm",
        py::arg("start"), py::arg("goal"), py::arg("frame_id"));
}
