#include <iostream>
#include <string>
#include <fstream>

#include "depth_verification.hpp"

int main(){

    std::string imgl, imgr;
    std::ifstream f;

    f.open("../depth_verification_data/images.txt");
    
    if(!f.is_open()){
        std::cout << "Error opening image txt file" << std::endl;
        exit(1);
    }

    f >> imgl >> imgr;
    f.close();

    std::vector<std::string> imageFileNames = {"../depth_verification_data/" + imgl, "../depth_verification_data/" + imgr};
    // std::vector<std::string> imageFileNames = {"../depth_verification_data/image1.png", "../depth_verification_data/image2.png"};

    DepthVerification dv;           
    dv.parseYamlConfig("../depth_verification_data/t265-ds-none.yaml");
    dv.readImageFiles(imageFileNames);
    dv.start();


    return 0;
}