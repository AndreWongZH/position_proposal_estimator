#include "depth_verification.hpp"
#include "calibration_parser.hpp"

#include <iostream>
#include <vector>
#include <cassert>

#include <pangolin/pangolin.h>
#include <pangolin/gl/gltext.h>

DepthVerification::DepthVerification() : points(2), 
                                         worldPoint(Eigen::Vector4d::Zero()) {}

// Parse camera model, intrinsic params, T_cn_cnm1, Distortion
// Resolution and RosTopic not parsed
void DepthVerification::parseYamlConfig(const std::string yaml_file){

    std::cout << "Parsing Param File " << yaml_file << std::endl;

    YAML::Node configs = YAML::LoadFile(yaml_file);
    
    int cam_id = 0;
    for(const auto& config : configs)
    {
        // Parse camera model
        std::string cameraModel = camera_type_map[config.second["camera_model"].as<std::string>()];
        auto cam = basalt::GenericCamera<double>::fromString(cameraModel);

        // Parse intrinsic parameters
        int intrinsicSize = config.second["intrinsics"].size();
        assert(cam.getN() == intrinsicSize);
        Eigen::VectorXd intrinsicParam(intrinsicSize);  
        for(int i=0; i<intrinsicSize; i++){
            intrinsicParam((i+4) % intrinsicSize) = config.second["intrinsics"][i].as<double>();
        }            
        std::cout<<     intrinsicParam<<std::endl;                          
        cam.applyInc(intrinsicParam);
        cams_.push_back(cam);

        // Parse distortion and undistort if necessary;
        if (config.second["distortion_model"].as<std::string>() != "none"){
            const size_t X = config.second["distortion_coeffs"].size();
            std::vector<double> distortion_coeffs(X);
            for (size_t i = 0; i < X; i++){
                distortion_coeffs[i] =  config.second["distortion_coeffs"][i].as<double>();
            }
            undistort(cam_id, distortion_coeffs);
        }

        // Parse Exrinisic Calibration
        if(config.second["T_cn_cnm1"]){
            std::vector<Eigen::RowVector4d> T;
            for (size_t i = 0; i < 4 ; i++)
            {
                T.emplace_back(config.second["T_cn_cnm1"][i].as<std::vector<double>>().data());
            }
            Eigen::Matrix4d M;
            M <<    T[0], 
                    T[1], 
                    T[2], 
                    T[3];
            T_cn_cnm1 = Sophus::SE3d(M);
        }

        // std::cout << "Camera " <<  cams_.size() - 1 << " initialised, type = " << cam.getName() << std::endl ;
        std::cout << "Intrinsics: " << cam.getParam().transpose() << std::endl;
        ++cam_id;
    }

    std::cout << "Extrinsic (cam0 to cam1): " <<  std::endl << T_cn_cnm1.matrix() << std::endl << std::endl;

}

void DepthVerification::readImageFiles(const std::vector<std::string> filenames){

    for(auto filename : filenames){
        cv::Mat image = cv::imread(filename, cv::IMREAD_GRAYSCALE);
        if(image.empty()){
            std::cout << "Error loading image: " << filename << std::endl;
            exit(1);
        }
        // std::cout << image.size() << std::endl;
        images.push_back(image);
    }
}

void DepthVerification::start(){

    pangolin::CreateWindowAndBind("Depth Verifier" , 1600, 900);
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);         // for displaying text
    
    // this is big window
    auto& img_view_display =
        pangolin::CreateDisplay()
            .SetBounds(0.2, 1.0, 0.0, 0.4)
            .SetLayout(pangolin::LayoutEqual);

    // this stores ptr to each image
    std::vector<std::shared_ptr<pangolin::ImageView>> img_view;

    // this is image format
    pangolin::GlPixFormat fmt;
    fmt.glformat = GL_LUMINANCE;
    fmt.gltype = GL_UNSIGNED_BYTE;
    fmt.scalable_internal_format = GL_LUMINANCE8;

    for(int i=0; i<2; i++){

        auto view = std::make_shared<pangolin::ImageView>();

        view->SetImage(images[i].ptr<uchar>(0,0), images[i].cols, images[i].rows, images[i].step, fmt);
        view->OnSelectionCallback = std::bind(&DepthVerification::OnSelectionCallback, this, std::placeholders::_1, i);
        view->SetDrawFunction(std::bind(&DepthVerification::drawImageViewOverlay, this, std::placeholders::_1, i));
    
        img_view.emplace_back(view);            
        img_view_display.AddDisplay(*view);
        
    }
    
    // pangolin::GlFont font(pangolin::AnonymousPro_ttf, 30);

    while( !pangolin::ShouldQuit() )
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        img_view_display.Activate();

        double dist = 1/worldPoint(3);

        glColor3f(1.0, 1.0, 1.0);
        std::string str = "unit vector (x,y,z): " + std::to_string(worldPoint(0)) + " " 
                                                  + std::to_string(worldPoint(1)) + " " 
                                                  + std::to_string(worldPoint(2)) + ", distance: " 
                                                  + std::to_string(dist);
        pangolin::GlFont::I().Text(str.c_str()).Draw(1000.0, 200.0);

        str = "Actual coordinates (x,y,z): " + std::to_string(dist*worldPoint(0)) + " " 
                                             + std::to_string(dist*worldPoint(1)) + " " 
                                             + std::to_string(dist*worldPoint(2));
        pangolin::GlFont::I().Text(str.c_str()).Draw(1000.0, 250.0);

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }
}

void DepthVerification::OnSelectionCallback(pangolin::ImageViewHandler::OnSelectionEventData e, int cam_id){
    
    if (!e.dragging){

        e.handler.GetHover(points[cam_id].x, points[cam_id].y);
                
        findWorldPoint();    
    }
}

// This function is run by Pangolin automatically for all viewports, set by SetDrawFunction()
void DepthVerification::drawImageViewOverlay(pangolin::View& v, size_t cam_id)
{
    v.Activate();
    
    auto &pt = points[cam_id];

    if(pt.x >= 0 && pt.y >= 0){
        float color[] = {255, 0, 0};
        glColor3fv(color);
        pangolin::glDrawCircle(pt.x, pt.y, 2);    
    }
}

void DepthVerification::undistort(int cam_id, std::vector<double> distortParams){
    
    cv::Mat outputImg = cv::Mat::zeros(images[cam_id].size(), images[cam_id].type());
    
    Eigen::Matrix3d k(Eigen::Matrix3d::Identity());
    Eigen::Matrix3d kinv(Eigen::Matrix3d::Identity());    

    Eigen::VectorXd param = cams_[cam_id].getParam();
    
    k(0,0) = param[0];
    k(1,1) = param[1];
    k(0,2) = param[2];
    k(1,2) = param[3];

    // std::cout<<k<<std::endl;

    double k1 = distortParams[0];
    double k2 = distortParams[1];
    double r1 = distortParams[2];
    double r2 = distortParams[3];

    for(int y = 0; y < images[cam_id].rows; ++y){
        for(int x = 0; x < images[cam_id].cols; ++x){
            // apply Kinv
            double mx = (x - k(0,2)) / k(0,0);
            double my = (y - k(1,2)) / k(1,1);

            // apply distortion
            double rho_sqr =  mx*mx + my*my;

            double ratio_radial = k1*rho_sqr + k2*rho_sqr*rho_sqr;

            double du_radial_x = mx * ratio_radial;
            double du_radial_y = my * ratio_radial;

            double du_tangent_x = 2.0*r1*mx*my + r2*(rho_sqr + 2.0 * mx*mx);
            double du_tangent_y = r1*(rho_sqr + 2.0*my*my) + 2.0*r2*mx*my;

            double udx = mx + du_radial_x + du_tangent_x;
            double udy = my + du_radial_y + du_tangent_y;

            double xd = k(0,0)*udx + k(0,2);
            double yd = k(1,1)*udy + k(1,2);

            const int u = (xd+0.5); //round off to nearest int
            const int v = (yd+0.5);
            
            if(u>=0 && v>=0 && u<images[cam_id].cols && v<images[cam_id].rows){
                outputImg.at<uchar>(y,x) = images[cam_id].at<uchar>(v,u);
            }
        }
    }
    images[cam_id] = outputImg;
}

void DepthVerification::findWorldPoint(){
    for(auto point:points){
        if(point.x < 0 || point.y < 0) {
            worldPoint = Eigen::Matrix<double, 4, 1>::Zero();
            return;
        }
    }

    Eigen::Vector2d p0, p1;
    Eigen::Vector4d q0, q1;
    Eigen::Vector3d r0, r1;

    p0 = {points[0].x, points[0].y};
    p1 = {points[1].x, points[1].y};

    cams_[0].unproject(p0, q0);
    cams_[1].unproject(p1, q1);

    r0 = q0.head(3), r1 = q1.head(3);

    // param: cam0 unit vector, cam1 unit vector, transform from cam1 to cam0;
    worldPoint = triangulate(r0, r1, T_cn_cnm1.inverse());

    std::cout << "pixel coord:\n";
    std::cout << p0.transpose() << std::endl << p1.transpose() << std::endl;

    std::cout << "cam coord unit vector:\n";
    std::cout << q0.transpose() << std::endl << q1.transpose() << std::endl;

    std::cout << "unit_vector (x,y,z), distance:\n";
    std::cout << worldPoint.head(3).transpose() << " " << 1/worldPoint(3) << std::endl;

    std::cout << "actual coordinates:\n";
    std::cout << worldPoint.head(3).transpose() / worldPoint(3) <<std::endl<<std::endl;
    
}

/// Triangulates the point and returns homogenous representation. First 3
/// components - unit-length direction vector. Last component inverse
/// distance.
template <class Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 1> DepthVerification::triangulate(
    const Eigen::MatrixBase<Derived>& f0,
    const Eigen::MatrixBase<Derived>& f1,
    const Sophus::SE3<typename Derived::Scalar>& T_0_1) {
    
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

    using Scalar = typename Derived::Scalar;
    using Vec4 = Eigen::Matrix<Scalar, 4, 1>;

    Eigen::Matrix<Scalar, 3, 4> P1, P2;
    P1.setIdentity();
    P2 = T_0_1.inverse().matrix3x4();

    Eigen::Matrix<Scalar, 4, 4> A(4, 4);
    A.row(0) = f0[0] * P1.row(2) - f0[2] * P1.row(0);
    A.row(1) = f0[1] * P1.row(2) - f0[2] * P1.row(1);
    A.row(2) = f1[0] * P2.row(2) - f1[2] * P2.row(0);
    A.row(3) = f1[1] * P2.row(2) - f1[2] * P2.row(1);

    Eigen::JacobiSVD<Eigen::Matrix<Scalar, 4, 4>> mySVD(A, Eigen::ComputeFullV);
    Vec4 worldPoint = mySVD.matrixV().col(3);
    worldPoint /= worldPoint.template head<3>().norm();

    // Enforce same direction of bearing vector and initial point
    if (f0.dot(worldPoint.template head<3>()) < 0) worldPoint *= -1;

    return worldPoint;
}