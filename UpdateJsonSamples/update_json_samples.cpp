// Copyright 2015 Baidu Inc. All Rights Reserved.
// Author:  Hou Wenbo (houwenbo@baidu.com)
//
// Update the json file by CNN detections.
//
#include <string>
#include <fstream>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "../JsonIO/json_io.h"
#include "../RoadConvertor/batch_convertor.h"

const int PANO_WIDTH = 8192;
const int PANO_HEIGHT = 4096;
const float MIN_OVERLAP = 0.6f;

void printUsage(void) {
    std::cout << "evaluate --file updatefile.txt --jsondir /home/json \
--config config.txt --pixels_per_meter 50 --dst_width_meters 50 \
--dst_length_meters 160" << std::endl;
}

int bound_rect(const std::vector<cv::Point>& road_pts, cv::Rect* p_rect) {
    if (road_pts.size() <= 2) {
        return -1;
    }
    int min_x = INT_MAX;
    int min_y = INT_MAX;
    int max_x = 0;
    int max_y = 0;
    for (int p = 0; p < static_cast<int>(road_pts.size()); ++p) {
        if (road_pts[p].x > max_x) {
            max_x = road_pts[p].x;
        }
        if (road_pts[p].y > max_y) {
            max_y = road_pts[p].y;
        }
        if (road_pts[p].x < min_x) {
            min_x = road_pts[p].x;
        }
        if (road_pts[p].y < min_y) {
            min_y = road_pts[p].y;
        }
    }
    cv::Rect rect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
    *p_rect = rect;
    return 0;
}

int main(int argc, char** argv) {
    std::string updatefile;
    std::string jsondir;
    std::string config_name;
    float pixels_per_meter = 50.0f;
    float dst_width_meters = 50.0f;
    float dst_length_meters = 160.0f;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "/?") {
            printUsage();
            return -1;
        }
        else if (std::string(argv[i]) == "--file") {
            updatefile = argv[i + 1];
            ++i;
        }
        else if (std::string(argv[i]) == "--jsondir") {
            jsondir = argv[i + 1];
            ++i;
        }
        else if (std::string(argv[i]) == "--config") {
            config_name = argv[i + 1];
            ++i;
        }
        else if (std::string(argv[i]) == "--pixels_per_meter") {
            pixels_per_meter = static_cast<float>(atof(argv[i + 1]));
            ++i;
        }
        else if (std::string(argv[i]) == "--dst_width_meters") {
            dst_width_meters = static_cast<float>(atof(argv[i + 1]));
            ++i;
        }
        else if (std::string(argv[i]) == "--dst_length_meters") {
            dst_length_meters = static_cast<float>(atof(argv[i + 1]));
            ++i;
        }
    }
    if (jsondir.empty() || config_name.empty() || updatefile.empty()) {
        printUsage();
        return -1;
    }

    if (jsondir[jsondir.length() - 1] != '/' || jsondir[jsondir.length() - 1] != '\\') {
        jsondir += "/";
    }

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
        config_file.close();
    }
    else {
        std::cout << "[ERROR] Can not find config file!" << std::endl;
        return -1;
    }

    stcv::road_cvtor::BatchConvertor batch_convertor;
    int ret = batch_convertor.init(road_cvt_parm, pixels_per_meter,
        dst_width_meters, dst_length_meters);
    if (ret != stcv::road_cvtor::RoadConvertor::ROADCVTOR_OK) {
        std::cout << "[ERROR] BatchConvertor init error." << std::endl;
        return ret;
    }

    std::ifstream datafile(updatefile.c_str());
    if (datafile.fail()) {
        std::cout << "[!] can not open file: " << updatefile << " !" << std::endl;
        return -1;
    }
    std::string line;
    while (datafile >> line) {
        if (line.empty()) {
            break;
        }
        std::string strbak = line;
        int pos = line.find('_');
        int t_code = atoi(line.substr(0, pos).c_str());
        line = line.substr(pos + 1);
        pos = line.find('_');
        line = line.substr(pos + 1);
        line = line.substr(7);
        pos = line.find('_');
        int o_code = atoi(line.substr(0, pos).c_str());
        line = line.substr(pos + 1);
        pos = line.find('_', 12);
        std::string pid = line.substr(0, pos);
        line = line.substr(pos + 1);
        pos = line.find('_');
        int idx = atoi(line.substr(0, pos).c_str()); 
        line = line.substr(pos + 1);
        pos = line.find('_');
        int x = atoi(line.substr(0, pos).c_str());
        line = line.substr(pos + 1);
        pos = line.find('_');
        int y = atoi(line.substr(0, pos).c_str());
        line = line.substr(pos + 1);
        pos = line.find('_');
        int w = atoi(line.substr(0, pos).c_str());
        line = line.substr(pos + 1);
        int h = w;
        cv::Rect rect(x, y, w, h);
        float size = w / 1.5f;
        x += static_cast<int>(0.5 * (w - size));
        y += static_cast<int>(0.5 * (h - size));
        rect.x = x;
        rect.y = y;
        rect.width = size;
        rect.height = size;
        rect.y += static_cast<int>(0.25 * h);
        rect.height /= 2;
        std::string jsonpath = jsondir + pid;
        std::ifstream json_file(jsonpath.c_str());
        if (!json_file.good()) {
            std::cout << "[WARN] Can not open json file " << jsonpath << std::endl;
            continue;
        }
        std::string json;
        json_file >> json;
        if (json.empty()) {
            std::cout << "[WARN] " << pid << " Json is empty. " << jsonpath << std::endl;
            continue;
        }
        stcv::json_io::JsonIO::JsonStruct mark;
        ret = stcv::json_io::JsonIO::parse_json(json, &mark);
        json_file.close();
        if (ret != 0) {
            std::cout << "[WARN] " << pid << " Parse json failed. " << jsonpath << std::endl;
            continue;
        }
        std::string prefix = pid.substr(0, 10);
        if (o_code == 0) {
            stcv::json_io::JsonIO::SampleMark sample;
            sample.graph = stcv::json_io::JsonIO::GraphicType::POLYGON;
            sample.is_tool_import = true;
            sample.pid = pid;
            sample.type_code = t_code;
            std::vector<cv::Point2i> road_pts;
            road_pts.push_back(rect.tl());
            road_pts.push_back(cv::Point(rect.x + rect.width, rect.y));
            road_pts.push_back(rect.br());
            road_pts.push_back(cv::Point(rect.x, rect.y + rect.height));
            batch_convertor.pts_road_to_pano(prefix, PANO_WIDTH, PANO_HEIGHT, road_pts, &sample.pts);
            mark.sample_marks.push_back(sample);
        }
        else {
            bool isfound = false;
            for (int m = 0; m < static_cast<int>(mark.sample_marks.size()); ++m) {
                std::vector<cv::Point2i> road_pts;
                ret = batch_convertor.pts_pano_to_road(prefix, PANO_WIDTH, PANO_HEIGHT,
                    mark.sample_marks[m].pts, &road_pts);
                if (ret != stcv::road_cvtor::RoadConvertor::ROADCVTOR_OK) {
                    std::cout << "[WARN] " << pid << " RoadConvertor failed." << std::endl;
                    continue;
                }
                cv::Rect gt_rect;
                ret = bound_rect(road_pts, &gt_rect);
                if (ret != 0) {
                    continue;
                }
                cv::Rect in_rect = rect & gt_rect;
                cv::Rect un_rect = rect | gt_rect;
                float overlap_ratio = static_cast<float>(in_rect.area()) / un_rect.area();
                if (overlap_ratio > MIN_OVERLAP) {
                    mark.sample_marks[m].type_code = t_code;
                    isfound = true;
                }
            }
            if (!isfound) {
                stcv::json_io::JsonIO::SampleMark sample;
                sample.graph = stcv::json_io::JsonIO::GraphicType::POLYGON;
                sample.is_tool_import = true;
                sample.pid = pid;
                sample.type_code = t_code;
                std::vector<cv::Point2i> road_pts;
                road_pts.push_back(rect.tl());
                road_pts.push_back(cv::Point(rect.x + rect.width, rect.y));
                road_pts.push_back(rect.br());
                road_pts.push_back(cv::Point(rect.x, rect.y + rect.height));
                batch_convertor.pts_road_to_pano(prefix, PANO_WIDTH, PANO_HEIGHT,
                    road_pts, &sample.pts);
                mark.sample_marks.push_back(sample);
            }
        }
        stcv::json_io::JsonIO::generate_json(mark, &json);
        std::ofstream ofile(jsonpath);
        ofile << json;
        ofile.close();
    }
    datafile.close();
    return 0;
}
