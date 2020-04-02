#pragma once

#include <vector>
#include <basalt/camera/generic_camera.hpp>

#include <sophus/se3.hpp>

#include "calibration_parser.hpp"

#include <map>

// For Debug
#include <cassert>


/// returns the 3D cross product skew symmetric matrix of a given 3D vector
template<class Derived>
  inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(const Eigen::MatrixBase<Derived> & vec)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1], vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
  }


std::map<std::string,std::string> camera_type_map = {
        {"pinhole", "pinhole"},
        {"ds", "ds"}
    };

class MultiViewGeometry{

public:
    //// Initialisation
    void appendCamera(const std::string camera_type, const Eigen::VectorXd& intrinsic_param, const Sophus::SE3<double>& T_w_c);
    void parseYamlConfig(const std::string yaml_file);

    //// Constants
    // const Eigen::Matrix3d EMatrix(const size_t from_cam, const size_t to_cam);
    const Eigen::Vector2d projectEpipole(const size_t from_cam, const size_t to_cam){return Epipole_vec_[from_cam][to_cam];};

    //// Operations
    // Takes in Normalised Camera Coordinates, Return the Epipolar lines
    Eigen::Matrix3Xd project2EpipolarLineinCamera(const Eigen::Matrix3Xd& points_camera, const size_t from_cam, const size_t to_cam);
    // Pixel Version
    Eigen::Matrix3Xd project2EpipolarLineinPixel(const Eigen::Matrix3Xd& points_pixel, const size_t from_cam, const size_t to_cam);
    Eigen::Vector3d project2EpipolarLineinPixel(const Eigen::Vector2d& point_pixel, const size_t from_cam, const size_t to_cam);


    //// Conversions
    inline void pixel2NormHomogeneous(const Eigen::Matrix2Xd xy_pixel, Eigen::Matrix3Xd xy_camera);
    inline void normHomogeneous2Pixel(const Eigen::Matrix3Xd xy_camera, Eigen::Matrix2Xd xy_pixel);

    size_t N_cam;
    
private:
    std::vector<basalt::GenericCamera<double>> cams_; // possible names: pinhole, ds
    // Eigen::aligned_vector<Sophus::SE3<double>> T_w_c_; // No need for GCC 7.0 and above

    // active rotation in camera. Convert world coordinates to camera coordinates
    std::vector<Sophus::SE3<double>> T_w_c_;
    // initialise with basalt::GenericCamera<double>::fromString()

    std::vector<std::vector<Eigen::Matrix3d>> E_vec_;
    std::vector<std::vector<Eigen::Matrix3d>> F_vec_;
    std::vector<std::vector<Eigen::Vector2d>> Epipole_vec_;

    void reset(){
        cams_.clear();
        T_w_c_.clear();
    }

    Eigen::Matrix3d GetKPinhole(const size_t cam);

};



////// IMPLEMENTATION ///////////////

void MultiViewGeometry::appendCamera(const std::string camera_type, const Eigen::VectorXd& intrinsic_param, const Sophus::SE3<double>& T_w_c)
{
    auto cam = basalt::GenericCamera<double>::fromString(camera_type);

    // Pass in intrinsic calibration
    assert(cam.getN() == intrinsic_param.size());
    cam.setFromInit(intrinsic_param);

    // Adding to class
    cams_.push_back(std::move(cam));
    T_w_c_.push_back(T_w_c);

    std::cout << "Camera " <<  cams_.size() - 1 << " initialised, type = " << cam.getName() << std::endl ;
    std::cout << "Intrinsics: " << cam.getParam().transpose() << std::endl;
    std::cout << "Extrinsic (world to camera): " <<  std::endl << T_w_c.matrix() << std::endl << std::endl;
}

void MultiViewGeometry::parseYamlConfig(const std::string yaml_file)
{
    CalibrationParser parser(yaml_file);
    auto& calib_vec = parser.getCalibrations();

    // Reset cameras and calibrations
    this->reset();

    for(auto& calib : calib_vec){
        appendCamera(camera_type_map[calib.camera_model],calib.intrinsics, calib.T_w_c);
    }

    // Reset E and F matrices, for pinhole only
    E_vec_.clear();
    F_vec_.clear();

    const size_t N_cam = calib_vec.size();
    this->N_cam = N_cam;
    E_vec_.resize(N_cam);
    F_vec_.resize(N_cam);
    Epipole_vec_.resize(N_cam);

    for(size_t i = 0; i < N_cam; i++)
    {
        E_vec_[i].resize(N_cam);
        F_vec_[i].resize(N_cam);
        Epipole_vec_[i].resize(N_cam);
        for(size_t j = 0; j < N_cam; j++)
        {
            // Active rotation from i to j, change coordinates from cam_j to cam_i
            auto T_j_i = T_w_c_[i] * T_w_c_[j].inverse();

            // Get Epipole (Change coordinates from i to j)
            Eigen::Vector4d point_3d;
            point_3d << T_j_i.inverse().translation(), 1; // coordinate in cam_j
            cams_[j].project(point_3d, Epipole_vec_[i][j]);


            // Get E and F
            if (i==j || cams_[i].getName() != "pinhole" || cams_[j].getName() != "pinhole")
                continue;
            

            // E_vec_[i][j] =  skew(T_j_i.translation()) * T_j_i.rotationMatrix(); // project from i to j's epipolar line
                E_vec_[i][j] =  T_j_i.rotationMatrix().transpose() * skew(T_j_i.translation()) ;

            // Obtain K
            F_vec_[i][j] = GetKPinhole(j).inverse().transpose() * E_vec_[i][j] * GetKPinhole(i).inverse();

            std::cout << "T_j_i " << i << j << ": " << std::endl << T_j_i.matrix() << std::endl;
            std::cout << "Epipole " << i << j << ": " << Epipole_vec_[i][j].transpose() << std::endl;
            std::cout << "E " << i << j << std::endl << E_vec_[i][j].matrix() << std::endl;
            std::cout << "F " << i << j << std::endl << F_vec_[i][j].matrix() << std::endl;
            std::cout << "GetKPinhole(i)" << std::endl << GetKPinhole(i) << std::endl;
        }
    }
        
}

Eigen::Matrix3d MultiViewGeometry::GetKPinhole(const size_t cam)
{
    auto param = cams_[cam].getParam();
    Eigen::Matrix3d ret;

    ret <<  param[0] , 0 , param[2],
            0,      param[1], param[3],
            0,0,1;

    return ret;
}


Eigen::Matrix3Xd MultiViewGeometry::project2EpipolarLineinCamera(const Eigen::Matrix3Xd& points_camera, const size_t from_cam, const size_t to_cam)
{
    Eigen::Matrix3Xd epipolar_lines = E_vec_[from_cam][to_cam] * points_camera;
    return epipolar_lines;
}

Eigen::Matrix3Xd MultiViewGeometry::project2EpipolarLineinPixel(const Eigen::Matrix3Xd& points_pixel, const size_t from_cam, const size_t to_cam)
{
    Eigen::Matrix3Xd epipolar_lines = F_vec_[from_cam][to_cam] * points_pixel;
    return epipolar_lines;
}


Eigen::Vector3d MultiViewGeometry::project2EpipolarLineinPixel(const Eigen::Vector2d& points_pixel, const size_t from_cam, const size_t to_cam)
{
    Eigen::Vector3d point_pixel_h;

    point_pixel_h << points_pixel , 1;
    return project2EpipolarLineinPixel(static_cast<Eigen::Matrix3Xd>(point_pixel_h), from_cam, to_cam);
}