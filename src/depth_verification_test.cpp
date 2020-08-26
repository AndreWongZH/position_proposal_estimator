#include <iostream>
#include <string>
#include <fstream>

#include "depth_verification.hpp"

int main(){

    std::string imgl, imgr;

    std::ifstream f;
    f.open("../depth_verification_data/images.txt");

    f >> imgl >> imgr;

    f.close();

    std::vector<std::string> imageFileNames = {"../depth_verification_data/" + imgl, "../depth_verification_data/" + imgr};

    DepthVerification dv;           
    dv.parseYamlConfig("../depth_verification_data/t265-ds-none.yaml");
    dv.readImageFiles(imageFileNames);
    dv.start();


    return 0;
}