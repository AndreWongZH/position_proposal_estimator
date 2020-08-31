#pragma once

#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sophus/se3.hpp>
#include <basalt/camera/generic_camera.hpp>
#include <pangolin/display/image_view.h>

class DepthVerification{
public:
    DepthVerification();

    void parseYamlConfig(const std::string yaml_file);
    void readImageFiles(const std::vector<std::string> filenames);
    void start();

private:
    struct Point{ float x = -1, y = -1; };

    std::map<std::string,std::string> camera_type_map = {{"pinhole" , "pinhole"},
                                                         {"ds"      , "ds"},
                                                         {"omni", "ucm"}};

    void OnSelectionCallback(pangolin::ImageViewHandler::OnSelectionEventData, int cam_id);
    void drawImageViewOverlay(pangolin::View& v, size_t cam_id);
    void findWorldPoint();
    void undistort(int cam_id, std::vector<double> distortParams);

    std::vector<basalt::GenericCamera<double>> cams_;
    std::vector<cv::Mat> images;
    std::vector<Point> points;
    Eigen::Matrix<double, 4, 1> worldPoint;
    Sophus::SE3<double> T_cn_cnm1;

    template <class Derived> 
    Eigen::Matrix<typename Derived::Scalar, 4, 1> triangulate(
        const Eigen::MatrixBase<Derived>& f0,
        const Eigen::MatrixBase<Derived>& f1,
        const Sophus::SE3<typename Derived::Scalar>& T_0_1);
};
