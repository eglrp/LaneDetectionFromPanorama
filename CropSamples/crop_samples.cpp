// Copyright 2015 Baidu Inc. All Rights Reserved.
// Author:  Hou Wenbo (houwenbo@baidu.com)
//
// 文件功能: 样本裁剪工具, 裁剪正样本, 裁剪正样本, 填黑生成负样本.
//
#include <iostream>
#include <fstream>
#include <string>
#include "../JsonIO/json_io.h"
#include "../RoadConvertor/batch_convertor.h"

#if _WIN32
#include <Windows.h>
#else
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#endif
#if _WIN32
bool directoryExists(LPCTSTR szPath)
{
    DWORD dwAttrib = GetFileAttributes(szPath);

    return dwAttrib != INVALID_FILE_ATTRIBUTES &&
        (dwAttrib & FILE_ATTRIBUTE_DIRECTORY);
}

void createDir(LPCTSTR szPath)
{
    CreateDirectory(szPath, NULL);
}

#else // if _WIN32
bool directoryExists(const char *path)
{
    struct stat s;
    int err = stat(path, &s);

    return -1 != err && S_ISDIR(s.st_mode);
}

void createDir(const char *path)
{
    mkdir(path, 0700);
}

#endif // if _WIN32

const int PANO_WIDTH = 8192;
const int PANO_HEIGHT = 4096;

void printUsage(void) {
    std::cout << "crop_samples --imgdir /home/pic_demo --jsondir /home/json \
--list list.txt --config config.txt --pixels_per_meter 50 --dst_width_meters 50 \
--dst_length_meters 160 --outdir /home/output" << std::endl;
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
    std::string imgdir;
    std::string jsondir;
    std::string list_name;
    std::string outdir;
    std::string config_name;
    float pixels_per_meter = 50.0f;
    float dst_width_meters = 50.0f;
    float dst_length_meters = 160.0f;
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "/?") {
            printUsage();
            return -1;
        }
        else if (std::string(argv[i]) == "--imgdir") {
            imgdir = argv[i + 1];
            ++i;
        }
        else if (std::string(argv[i]) == "--jsondir") {
            jsondir = argv[i + 1];
            ++i;
        }
        else if (std::string(argv[i]) == "--outdir") {
            outdir = argv[i + 1];
            ++i;
        }
        else if (std::string(argv[i]) == "--list") {
            list_name = argv[i + 1];
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

    if (imgdir.empty() || jsondir.empty() || outdir.empty() || list_name.empty()
        || config_name.empty()) {
        printUsage();
        return -1;
    }

    if (imgdir[imgdir.length() - 1] != '/' || imgdir[imgdir.length() - 1] != '\\') {
        imgdir += "/";
    }
    if (jsondir[jsondir.length() - 1] != '/' || jsondir[jsondir.length() - 1] != '\\') {
        jsondir += "/";
    }
    if (outdir[outdir.length() - 1] != '/' || outdir[outdir.length() - 1] != '\\') {
        outdir += "/";
    }

    std::cout << "---------------------------------------------------------------" << std::endl;
    std::cout << "-------------------------crop samples--------------------------" << std::endl;
    std::cout << "---------------------------------------------------------------" << std::endl;

    //create negative sample dir
    std::string ns_path = outdir + "negative/";
    if (!directoryExists(ns_path.c_str())) {
        createDir(ns_path.c_str());
    }
    std::string ps_path = outdir + "positive/";
    if (!directoryExists(ps_path.c_str())) {
        createDir(ps_path.c_str());
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

    std::ifstream img_list(list_name.c_str());
    if (!img_list.good()) {
        std::cout << "[ERROR] Can not open image list! " << list_name << std::endl;
        return -1;
    }
    std::string pid;
    while (img_list >> pid) {
        if (pid.empty()) {
            break;
        }
        stcv::road_cvtor::BatchConvertor batch_convertor;
        int ret = batch_convertor.init(road_cvt_parm, pixels_per_meter,
            dst_width_meters, dst_length_meters);
        if (ret != stcv::road_cvtor::RoadConvertor::ROADCVTOR_OK) {
            std::cout << "[ERROR] BatchConvertor init error." << std::endl;
            return ret;
        }
        std::cout << pid << std::endl;
        std::string impath = imgdir + pid + ".jpg";
        cv::Mat road = cv::imread(impath.c_str());
        if (road.empty()) {
            std::cout << pid << "[WARN] Can not find image!" << std::endl;
            continue;
        }
        std::string jsonpath = jsondir + pid;
        std::ifstream json_file(jsonpath.c_str());
        if (!json_file.good()) {
            std::cout << "[WARN] Can not open json file " << jsonpath << std::endl;
            continue;
        }
        std::string json;
        json_file >> json;
        if (!json.empty()) {
            std::cout << "[WARN] Json is empty. " << jsonpath << std::endl;
            continue;
        }
        stcv::json_io::JsonIO::JsonStruct mark;
        ret = stcv::json_io::JsonIO::parse_json(json, &mark);
        json_file.close();
        if (ret != 0) {
            std::cout << "[WARN] Parse json failed. " << jsonpath << std::endl;
            continue;
        }
        std::vector<cv::Rect> sample_rect;//for negative samples
        for (int m = 0; m < static_cast<int>(mark.sample_marks.size()); ++m) {
            if (mark.sample_marks[m].pts.size() <= 3) {
                continue;
            }
            std::vector<cv::Point2i> road_pts;
            ret = batch_convertor.pts_pano_to_road(PANO_HEIGHT, PANO_WIDTH,
                mark.sample_marks[m].pts, &road_pts);
            if (ret != stcv::road_cvtor::RoadConvertor::ROADCVTOR_OK) {
                std::cout << "[WARN] RoadConvertor failed." << std::endl;
                continue;
            }
            cv::Rect rect;
            ret = bound_rect(road_pts, &rect);
            if (ret == 0) {
                sample_rect.push_back(rect);
            }
            // create pos samples
            if (mark.sample_marks[m].pts.size() <= 5 || rect.width <= 30 || rect.height <= 30) {
                continue;
            }
            std::string mark_path = cv::format("%s/%s_%d_%d_%d_%d_%d.jpg",
                ps_path.c_str(), mark.sample_marks[m].pid.c_str(), m, rect.x, rect.y,
                rect.width, rect.height);
            if (rect.width > rect.height) {
                rect.y -= static_cast<int>((rect.width - rect.height) * 0.5);
                rect.height = rect.width;
            }
            else if (rect.width < rect.height) {
                rect.x -= static_cast<int>((rect.height - rect.width) * 0.5);
                rect.width = rect.height;
            }
            int expand = 5;
            rect.x -= expand;
            rect.y -= expand;
            rect.width += (expand * 2);
            rect.height += (expand * 2);
            if (rect.x < 0) {
                rect.x = 0;
            }
            if (rect.y < 0) {
                rect.y = 0;
            }
            int road_width_pixels = road.rows;
            int road_length_pixels = road.cols;
            if (rect.x + rect.width >= road_length_pixels) {
                rect.x = road_length_pixels - 1 - rect.width;
            }
            if (rect.y + rect.height >= road_width_pixels) {
                rect.y = road_width_pixels - 1 - rect.height;
            }
            if (rect.x < 0 || rect.y < 0) {
                continue;
            }
            cv::imwrite(mark_path, road(rect));
        }
    } // end while
    img_list.close();

    return 0;
}
