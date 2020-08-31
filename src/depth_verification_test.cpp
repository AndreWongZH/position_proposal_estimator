#include <iostream>
#include <string>
#include <fstream>

#include "depth_verification.hpp"

int main(){

    // std::string imgl, imgr;
    // std::ifstream f;

    // f.open("../depth_verification_data/images.txt");
    // if(!f.is_open()){
    //     std::cout << "Error opening image txt file" << std::endl;
    //     exit(1);
    // }
    // f >> imgl >> imgr;
    // f.close();

    // std::vector<std::string> imageFileNames = {"../depth_verification_data/" + imgl, "../depth_verification_data/" + imgr};

    std::vector<std::string> imageFileNames = {
        "../depth_verification_data/fisheye1-1596704736.954.png", 
        "../depth_verification_data/fisheye2-1596704736.954.png"};

    DepthVerification dv;           
    dv.readImageFiles(imageFileNames);
    dv.parseYamlConfig("../depth_verification_data/t265-omni-radtan.yaml");
    dv.start();


    return 0;
}