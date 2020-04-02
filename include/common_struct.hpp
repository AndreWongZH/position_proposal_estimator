#pragma once

#include <Eigen/Eigen>
#include <sophus/se3.hpp>

struct Point2D{
    using List = std::list<Point2D>;
    int id;
    size_t view_id;
    float point_size = 2;
    std::array<float,3> colour; // RGB colour
    Eigen::Vector2d xy;
};

struct Point3D{
    using List = std::list<Point3D>;
    int id;
    size_t view_id;
    float point_size = 2;
    std::array<float,3> colour; // RGB colour
    Eigen::Vector3d xyz;
};


// 2D line in homogeneous coordinates
struct Line2D{
    using List = std::list<Line2D>;

    int id;
    size_t view_id;
    float line_size = 2;
    Point2D begin;
    Point2D end;
};

namespace Eigen{
    using Vector6d = Eigen::Matrix<double,6,1> ;
};

struct Line3D{
    int id;
    size_t view_id;
    float line_size = 1;
    Point3D begin;
    Point3D end;
};
