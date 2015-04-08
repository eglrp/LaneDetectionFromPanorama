// Copyright 2015 Baidu Inc. All Rights Reserved.
// Author:  Hou Wenbo (houwenbo@baidu.com)
//
// CNN Sample cutting tool.
//
#include <iostream>
#include <fstream>
#include <string>
#include "../JsonIO/json_io.h"
#include "../RoadConvertor/batch_convertor.h"
#include "evaluate.h"
#include "batch_file.h"

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
const float OVERLAP = 0.4f;
const int SAMPLE_WIDTH = 96;
const int SAMPLE_HEIGHT = 96;

void printUsage(void) {
    std::cout << "crop_cnn_samples --imgdir /home/pic_demo --jsondir /home/json \
--txtdir /home/txt --list list.txt --config config.txt --pixels_per_meter 50 --dst_width_meters 50 \
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
    std::string txtdir;
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
        else if (std::string(argv[i]) == "--txtdir") {
            txtdir = argv[i + 1];
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
        || config_name.empty() || txtdir.empty()) {
        printUsage();
        return -1;
    }

    if (imgdir[imgdir.length() - 1] != '/' || imgdir[imgdir.length() - 1] != '\\') {
        imgdir += "/";
    }
    if (jsondir[jsondir.length() - 1] != '/' || jsondir[jsondir.length() - 1] != '\\') {
        jsondir += "/";
    }
    if (txtdir[txtdir.length() - 1] != '/' || txtdir[txtdir.length() - 1] != '\\') {
        txtdir += "/";
    }
    if (outdir[outdir.length() - 1] != '/' || outdir[outdir.length() - 1] != '\\') {
        outdir += "/";
    }

    std::cout << "---------------------------------------------------------------" << std::endl;
    std::cout << "-----------------------crop cnn samples------------------------" << std::endl;
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

    stcv::Evaluate evaluate(OVERLAP);
    stcv::BatchFile batch_file(10, 100, SAMPLE_WIDTH, SAMPLE_HEIGHT, 3, ns_path, "test");
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
        std::string txtpath = txtdir + pid + ".txt";
        ret = evaluate.load_detect_result(txtpath.c_str());
        if (ret != 0) {
            std::cout << "[ERROR] parse file " << txtpath << std::endl;
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
        if (json.empty()) {
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
        std::vector<stcv::DetResult> groundtruth_vec;
        for (int m = 0; m < static_cast<int>(mark.sample_marks.size()); ++m) {
            if (mark.sample_marks[m].pts.size() <= 3) {
                continue;
            }
            stcv::DetResult det_result;
            det_result.is_used = true;
            if (mark.sample_marks[m].type_code % 10000 == 9999) {
                det_result.is_used = false;
            }
            if (mark.sample_marks[m].pts.size() <= 5) {
                det_result.is_used = false;
            }
            std::string prefix = pid.substr(0, 10);
            std::vector<cv::Point2i> road_pts;
            ret = batch_convertor.pts_pano_to_road(prefix, PANO_WIDTH, PANO_HEIGHT,
                mark.sample_marks[m].pts, &road_pts);
            if (ret != stcv::road_cvtor::RoadConvertor::ROADCVTOR_OK) {
                std::cout << "[WARN] RoadConvertor failed." << std::endl;
                continue;
            }
            cv::Rect rect;
            ret = bound_rect(road_pts, &rect);
            if (ret != 0) {
                continue;
            }
            // create pos samples
            if (rect.width <= 30 || rect.height <= 30) {
                det_result.is_used = false;
            }
            cv::Point2f center(0.0f, 0.0f);
            for (int mp = 0; mp < static_cast<int>(road_pts.size()); ++mp) {
                center.x += road_pts[mp].x;
                center.y += road_pts[mp].y;
            }
            center.x /= static_cast<float>(road_pts.size());
            center.y /= static_cast<float>(road_pts.size());
            if (rect.width > rect.height) {
                rect.y -= static_cast<int>((rect.width - rect.height) * 0.5);
                rect.height = rect.width;
            }
            else if (rect.width < rect.height) {
                rect.x -= static_cast<int>((rect.height - rect.width) * 0.5);
                rect.width = rect.height;
            }
            if (rect.x < 0) {
                rect.x = 0;
            }
            if (rect.y < 0) {
                rect.y = 0;
            }
            det_result.is_true = true;
            det_result.label = mark.sample_marks[m].type_code;
            det_result.rect = rect;
            if (road_pts[0].x < center.x) {
                det_result.direction = 0;
            }
            else {
                det_result.direction = 1;
            }
            groundtruth_vec.push_back(det_result);
        }
        if (groundtruth_vec.size() == 0) {
            continue;
        }
        evaluate.set_groundtruth(groundtruth_vec);
        std::vector<stcv::DetResult> evaluate_result;
        ret = evaluate.evaluate(&evaluate_result);
        if (ret != 0) {
            std::cout << "[ERROR] evaluate error!" << std::endl;
            continue;
        }
        char prefix[5];
        int det_num = static_cast<int>(evaluate_result.size());
        for (int d = 0; d < det_num; ++d) {
            if (!evaluate_result[d].is_used) {
                continue;
            }
            cv::Rect roi = evaluate_result[d].rect;
            if (roi.x < 0 || roi.x + roi.width > road.cols
                || roi.y < 0 || roi.y + roi.height > road.rows) {
                continue;
            }
            cv::Mat patch = road(roi);
            if (evaluate_result[d].is_true && evaluate_result[d].is_used) {
                strcpy(prefix, "TP");
                // extend twice
                int half_roi_width = static_cast<int>(roi.width * 0.5 + 0.5);
                int half_roi_heigth = static_cast<int>(roi.height * 0.5 + 0.5);
                roi.x = (std::max)(roi.x - half_roi_width, 0);
                roi.y = (std::max)(roi.y - half_roi_heigth, 0);
                roi.width =
                    roi.x + 2 * roi.width < road.cols ? 2 * roi.width : road.cols - 1 - roi.x;
                roi.height =
                    roi.y + 2 * roi.height < road.rows ? 2 * roi.height : road.rows - 1 - roi.y;
                patch = road(roi);

                char buffer[256];
                sprintf(buffer, "%sDET_%s_%d_%s_%d_%04d_%04d_%04d_%04d.jpg",
                    ps_path.c_str(), prefix, evaluate_result[d].label, pid.c_str(), d,
                    roi.x, roi.y, roi.width, roi.height);
                if (evaluate_result[d].direction == 0) {
                    cv::imwrite(buffer, patch);
                }
                else {
                    cv::Mat flip_img;
                    cv::flip(patch, flip_img, 1);
                    cv::imwrite(buffer, flip_img);
                }
            }
            else {
                strcpy(prefix, "FP");
                cv::resize(patch, patch, cv::Size(SAMPLE_WIDTH, SAMPLE_HEIGHT));

                char buffer[256];
                sprintf(buffer, "DET_%s_%d_%s_%d_%04d_%04d_%04d_%04d.jpg",
                    prefix, evaluate_result[d].label, pid.c_str(), d,
                    roi.x, roi.y, roi.width, roi.height);

                batch_file.add_patch(patch, evaluate_result[d].label, evaluate_result[d].rect,
                    buffer);
            }
        }
    } // end while
    batch_file.write_remain_data();
    img_list.close();

    return 0;
}
