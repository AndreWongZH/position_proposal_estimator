#pragma once

#include "yaml-cpp/yaml.h"
#include <sophus/se3.hpp>

#include <string>

#include <cassert>


class CalibrationParser{
public:

    struct Calib{
        std::string camera_model;
        Eigen::VectorXd intrinsics;
        Eigen::Vector2i resolution;
        std::string rostopic;
        std::string distortion_model;
        Eigen::VectorXd distortion_coeffs;
        Sophus::SE3<double> T_w_c;
    };
    CalibrationParser() = delete;
    CalibrationParser(std::string yaml_file);

    const std::vector<Calib> getCalibrations(){
        return calib_vec_;
    }

private:
    YAML::Node config_;
    std::vector<Calib> calib_vec_;
};



////// IMPLEMENTATION ///////////////////////

CalibrationParser::CalibrationParser(std::string yaml_file)
{
    std::cout << "Parsing Param File " << yaml_file << std::endl;
    config_ = YAML::LoadFile(yaml_file);

    // Initialise the Calib Struct
    calib_vec_.clear();

    for (const auto& cam : config_)
    {
        // std::cout << "Parsing " << cam.first << ": ";
        auto& calib = calib_vec_.emplace_back();

        // Parsing Intrinsics
        calib.camera_model = cam.second["camera_model"].as<std::string>();
        // std::cout << calib.camera_model << ", ";

        const size_t X = cam.second["intrinsics"].size();
        calib.intrinsics.resize(X);
        for (size_t i = 0; i < X; i++)
        {
            calib.intrinsics(i) =  cam.second["intrinsics"][i].as<double>();
        }

        // Parsing Distortion
        calib.distortion_model = cam.second["distortion_model"].as<std::string>();
        // std::cout << calib.distortion_model << std::endl;

        if (calib.distortion_model != "none")
        {
            const size_t X = cam.second["distortion_coeffs"].size();
            calib.distortion_coeffs.resize(X);
            for (size_t i = 0; i < X; i++)
            {
                calib.distortion_coeffs(i) =  cam.second["distortion_coeffs"][i].as<double>();
            }
        }

        // Parse Resolution
        calib.resolution[0] = cam.second["resolution"][0].as<int>();
        calib.resolution[1] = cam.second["resolution"][1].as<int>();

        // Parse ROS Topic
        if (cam.second["rostopic"])
        {
            calib.rostopic = cam.second["rostopic"].as<std::string>();
        }

        // Parse Extrinsic Calibration
        std::vector<Eigen::RowVector4d> T;
        for (size_t i = 0; i < 4 ; i++)
        {
            T.emplace_back(cam.second["T_w_c"][i].as<std::vector<double>>().data());
        }
        Eigen::Matrix4d M;
        M <<    T[0], 
                T[1], 
                T[2], 
                T[3];
        calib.T_w_c = Sophus::SE3d(M);

        // std::cout << "T_w_c" << std::endl;
        // std::cout << calib.T_w_c.matrix() << std::endl << std::endl;
    }

    assert(config_.size() == calib_vec_.size());
}
