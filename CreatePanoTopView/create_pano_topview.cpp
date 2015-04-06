// Copyright 2015 Baidu Inc. All Rights Reserved.
// Author:  Hou Wenbo (houwenbo@baidu.com)
//
// 文件功能: 将全景图中的道路生成正视图的, 批量转换工具.
//
#include <iostream>
#include <fstream>
#include <string>
#include "../RoadConvertor/batch_convertor.h"

void printUsage(void) {
    std::cout << "create_pano_topview --config config.txt --basedir /home/pic_demo \
--list list.txt --pixels_per_meter 50 --dst_width_meters 50 --dst_length_meters 160 \
--output /home/output" << std::endl;
}

int main(int argc, char** argv) {
    std::string basedir;
    std::string list;
    std::string output;
    std::string config_name;
    float pixels_per_meter = 50.0f;
    float dst_width_meters = 50.0f;
    float dst_length_meters = 160.0f;
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
            pixels_per_meter = static_cast<float>(atof(argv[i + 1]));
            i++;
        }
        else if (std::string(argv[i]) == "--dst_width_meters") {
            dst_width_meters = static_cast<float>(atof(argv[i + 1]));
            i++;
        }
        else if (std::string(argv[i]) == "--dst_length_meters") {
            dst_length_meters = static_cast<float>(atof(argv[i + 1]));
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

    std::map<std::string, stcv::road_cvtor::RoadCvtParm> road_cvt_parm;
    std::ifstream config_file(config_name.c_str());
    if (config_file.good()) {
        std::string batch;
        while (config_file >> batch) {
            if (batch.empty()) {
                break;
            }

            float camera_height;
            float heading;
            float pitch;
            float pitch_back;

            config_file >> camera_height;
            config_file >> heading;
            config_file >> pitch;
            config_file >> pitch_back;

            stcv::road_cvtor::RoadCvtParm parm;
            parm.camera_height = camera_height;
            parm.heading = heading;
            parm.pitch = pitch;
            parm.pitch_back = pitch_back;
            road_cvt_parm.insert(std::pair<std::string,
                stcv::road_cvtor::RoadCvtParm>(batch, parm));
        }
    }
    else {
        std::cout << "can not find config file!" << std::endl;
        return -1;
    }

    std::ifstream img_list(list.c_str());
    if (img_list.good()) {
        stcv::road_cvtor::BatchConvertor batch_convertor;
        int ret = batch_convertor.init(road_cvt_parm, pixels_per_meter,
            dst_width_meters, dst_length_meters);
        if (ret != stcv::road_cvtor::RoadConvertor::ROADCVTOR_OK) {
            std::cout << "[ERROR] BatchConvertor init error." << std::endl;
            return ret;
        }
        std::string pid;
        while (img_list >> pid) {
            std::cout << pid << std::endl;
            std::string impath = basedir + pid + ".jpg";
            cv::Mat road;
            cv::Mat pano = cv::imread(impath.c_str());
            if (pano.empty()) {
                std::cout << pid << "[WARN] Can not find image, " << impath << std::endl;
                continue;
            }
            int64 start = cv::getTickCount();
            std::string prefix = pid.substr(0, 10);
            ret = batch_convertor.road_cvtor(prefix, pano, &road);
            if (ret != stcv::road_cvtor::RoadConvertor::ROADCVTOR_OK) {
                std::cout << pid << "[WARN] Convert faild! " << pid << std::endl;
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
