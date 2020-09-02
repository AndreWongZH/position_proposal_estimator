#pragma once

#include <iostream>
#include <string>
#include <cassert>

class Cam{
public:
    Cam(){camera_model = "";}
    Cam(std::string model){ setCameraModel(model); }

    bool unproject(const Eigen::Vector2d p, Eigen::Vector4d &q);
    bool project(const Eigen::Vector4d p, Eigen::Vector2d &q);
    void setCameraModel(std::string model);
    void setParam(Eigen::VectorXd p);
    Eigen::VectorXd getParam() {return param;};

private:
    // map model to number of intrinsic param;
    std::map<std::string, int> numParam = {{"pinhole", 4},
                                           {"omni", 5},
                                           {"ds", 6}};

    std::string camera_model;
    Eigen::VectorXd param;

};

void Cam::setCameraModel(std::string model){
    
    // checks if camera_model is supported;
    for(auto m:numParam){
        if(m.first == model){
            camera_model = model;
            return;
        }
    }

    std::cout << "Error: Camera Model " << model << "is not supported" << std::endl;
    exit(1);
}

// pinhole: fx,fy,cx,cy
// omni:    fx,fy,cx,cy,xi
// ds:      fx,fy,cx,cy,xi,alpha 
void Cam::setParam(Eigen::VectorXd p) {
    assert(p.rows() == numParam[camera_model]); 
    param = p;
}

// @param[in] p point to unproject
// @param[out] q result of unprojection as unit vector
// @return whether  unprojecion is valid
bool Cam::unproject(const Eigen::Vector2d p, Eigen::Vector4d &q){

    const double fx = param[0];
    const double fy = param[1];
    const double cx = param[2];
    const double cy = param[3];

    const double mx = (p(0) - cx)/fx;
    const double my = (p(1) - cy)/fy;
    
    const double r2 = mx * mx + my * my;

    if(camera_model == "pinhole"){

        const double norm_inv = 1/sqrt(1+r2);

        q(0) = mx*norm_inv;
        q(1) = my*norm_inv;
        q(2) = norm_inv;
        q(3) = 0;
        // std::cout << q <<std::endl;

    }else if(camera_model == "omni"){

        const double xi = param(4);

        const double D = abs(1+(1-xi*xi)*r2);
        if (D < 1e-5) return false;

        const double fact = (xi + sqrt(D))/(r2 + 1);
        // if the points go to close to 180° things go crazy!
        if (fact - xi < 0.1) return false;

        const double fact2 = 1/(fact - xi);
        
        q(0) = fact*fact2*mx;
        q(1) = fact*fact2*my;
        q(2) = 1;
        q(3) = 0;
        q.normalize();
        // std::cout << q <<std::endl;

    }else if(camera_model == "ds"){

        const double xi = param(4);
        const double alpha = param(5);

        if (alpha > double(0.5)) {
            if (r2 >= double(1) / (double(2) * alpha - double(1))) 
                return false;
        }

        const double xi2_2 = alpha * alpha;
        const double xi1_2 = xi * xi;

        const double sqrt2 = sqrt(double(1) - (double(2) * alpha - double(1)) * r2);

        const double norm2 = alpha * sqrt2 + double(1) - alpha;

        const double mz = (double(1) - xi2_2 * r2) / norm2;
        const double mz2 = mz * mz;

        const double norm1 = mz2 + r2;
        const double sqrt1 = sqrt(mz2 + (double(1) - xi1_2) * r2);
        const double k = (mz * xi + sqrt1) / norm1;

        q(0) = k * mx;
        q(1) = k * my;
        q(2) = k * mz - xi;
        q(3) = double(0);
        q.normalize();
        // std::cout << q << std::endl;

    }

    return true;
}

bool Cam::project(const Eigen::Vector4d p, Eigen::Vector2d &q){
    const double fx = param[0];
    const double fy = param[1];
    const double cx = param[2];
    const double cy = param[3];
    
    double mx=0, my=0;

    if(camera_model == "pinhole"){
        mx = p(0)/p(2);
        my = p(1)/p(2);
    }else if(camera_model == "omni"){
        const double& xi = param[4];

        const double length = p.head(3).norm();
        const double denom = xi * length + p(2);
        
        mx = p(0)/denom;
        my = p(1)/denom;

    }else if(camera_model == "ds"){
        const double& xi = param[4];
        const double& alpha = param[5];

        const double x = p(0);
        const double y = p(1);
        const double z = p(2);
        const double xx = x * x;
        const double yy = y * y;
        const double zz = z * z;

        const double r2 = xx + yy;

        const double d1_2 = r2 + zz;
        const double d1 = sqrt(d1_2);

        const double w1 = alpha > double(0.5) ? (double(1) - alpha) / alpha
                                            : alpha / (double(1) - alpha);
        const double w2 =
            (w1 + xi) / sqrt(double(2) * w1 * xi + xi * xi + double(1));
        if (z <= -w2 * d1) return false;

        const double k = xi * d1 + z;
        const double kk = k * k;

        const double d2_2 = r2 + kk;
        const double d2 = sqrt(d2_2);

        const double norm = alpha * d2 + (double(1) - alpha) * k;

        mx = x / norm;
        my = y / norm;

    }

    q(0) = fx * mx + cx;
    q(1) = fy * my + cy;
    // std::cout << q <<std::endl;

    return true;
}