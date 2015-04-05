#include <iostream>
#include <fstream>
#include <string>
#include "../RoadConvertor/road_convertor.h"

void printUsage(void) {
    std::cout << "create_pano_topview --config config.txt --basedir /home/pic_demo \
--list list.txt --pixels_per_meter 50 --output /home/output" << std::endl;
}

int main(int argc, char** argv) {
    std::string basedir;
    std::string list;
    std::string output;
    std::string config_name;
    int pixels_per_meter = 50;
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "/?") {
            printUsage();
            return -1;
        }
        else if (std::string(argv[i]) == "--basedir") {
            basedir = argv[i + 1];
            i++;
        }
        else if (std::string(argv[i]) == "--list") {
            list = argv[i + 1];
            i++;
        }
        else if (std::string(argv[i]) == "--output") {
            output = argv[i + 1];
            i++;
        }
        else if (std::string(argv[i]) == "--config") {
            config_name = argv[i + 1];
            i++;
        }
        else if (std::string(argv[i]) == "--pixels_per_meter") {
            pixels_per_meter = atoi(argv[i + 1]);
            i++;
        }
    }

    if (basedir.empty() || output.empty() || list.empty() || config_name.empty()) {
        printUsage();
        return -1;
    }

    if (basedir[basedir.length() - 1] != '/' || basedir[basedir.length() - 1] != '\\') {
        basedir += "/";
    }
    if (output[output.length() - 1] != '/' || output[output.length() - 1] != '\\') {
        output += "/";
    }

    std::cout << "---------------------------------------------------------------" << std::endl;
    std::cout << "----------------------panorama to topview----------------------" << std::endl;
    std::cout << "---------------------------------------------------------------" << std::endl;

    float camera_height = 0.0f;
    float heading = 0.0f;
    float pitch = 0.0f;
    float pitch_back = 0.0f;
    float roll = 0.0f;
    float dst_height_meters = 0.0f;
    float dst_width_meters = 0.0f;
    std::ifstream config_file(config_name.c_str());
    if (config_file.good()) {
        config_file >> camera_height;
        config_file >> heading;
        config_file >> pitch;
        config_file >> pitch_back;
        config_file >> roll;
        config_file >> dst_height_meters;
        config_file >> dst_width_meters;
        config_file.close();

        std::cout << "camera height: " << camera_height << std::endl;
        std::cout << "heading: " << heading << std::endl;
        std::cout << "pitch: " << pitch << std::endl;
        std::cout << "pitch back: " << pitch_back << std::endl;
        std::cout << "roll: " << roll << std::endl;
        std::cout << "pixels per meter: " << pixels_per_meter << std::endl;
        std::cout << "meters of height: " << dst_height_meters << std::endl;
        std::cout << "meters of width: " << dst_width_meters << std::endl;
    }
    else {
        std::cout << "can not find config file!" << std::endl;
        return -1;
    }

    std::ifstream img_list(list.c_str());
    if (img_list.good()) {
        //stcv::RoadConvertor road_convertor(1.45f, 270.0f, -0.6f, 0.0f, 50.0f, 30.0f, 80.0f);
        stcv::RoadConvertor road_convertor(camera_height, heading, pitch, pitch_back, roll,
            pixels_per_meter, dst_height_meters, dst_width_meters);
        std::string pid;
        while (img_list >> pid) {
            std::cout << pid << std::endl;
            std::string impath = basedir + pid + ".jpg";
            cv::Mat road;
            cv::Mat pano = cv::imread(impath.c_str());
            if (pano.empty()) {
                std::cout << pid << " can not find image!" << std::endl;
                continue;
            }
            int64 start = cv::getTickCount();
            int ret = road_convertor.create_road_topview(pano, &road);
            if (ret != stcv::RoadConvertor::ROADCVTOR_OK) {
                std::cout << pid << " convert faild!" << std::endl;
                continue;
            }
            int64 end = cv::getTickCount();
            std::cout << static_cast<double>(end - start) / cv::getTickFrequency() << std::endl;
            std::string outpath = output + pid + ".jpg";
            cv::imwrite(outpath.c_str(), road);
        } // end while
    } // end img_list
    img_list.close();
    return 0;
}
