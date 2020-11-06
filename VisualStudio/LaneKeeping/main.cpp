#include <iostream>
#include <string>
#include "Wrapper.h"

int main(int argc, const char** argv) {

    //std::string source = "D://Kamerka//Movie//2020_0911_222535_025.MP4";
    std::string source = "D://Kamerka//Movie//Autostrada//2018_0723_063625_072.MP4";
    //std::string source = "D://Kamerka//output.mp4";
    if (argc > 1 && strlen(argv[1]) > 0) source = std::string(argv[1]);
    
    Wrapper::init(source);
    Wrapper::run();
}