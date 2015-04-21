#include <string>
#include <fstream>
#include <vector>
#include <iostream>
#include <map>
#include "opencv2/opencv.hpp"
#include "../JsonIO/json_io.h"
#include "../RoadConvertor/batch_convertor.h"

const int PANO_WIDTH = 8192;
const int PANO_HEIGHT = 4096;
const float MIN_OVERLAP = 0.1f;
const int MIN_X = 2500;
const int MAX_X = 5500;
const int MIN_Y = 0;
const int MAX_Y = 4000;
const float NEG_THR = 0.75f;
const float SEC_THR = 1.0f;
const int DET_MIN_SIZE = 60;

struct  LabelScore
{
    int label;
    float score;
};
struct DetRes {
    cv::Rect rect;
    std::vector<LabelScore> labels_score;
};

struct LableSt
{
    LableSt() {
        total = 0;
        loss = 0;
        find = 0;

        det_total = 0;
        det_true = 0;
        det_false = 0;
    }
    int total;
    int loss;
    int find;

    int det_total;
    int det_true;
    int det_false;
};

void printUsage(void) {
    std::cout << "evaluate --txtdir /home/txt --imgdir /home/img --jsondir /home/json \
--list list.txt --config config.txt --top 3 --pixels_per_meter 50 --dst_width_meters 50 \
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

void cal_overlap(const cv::Rect& rect1, const cv::Rect& rect2, float* p_overlap_ratio) {
    cv::Rect inter_rect = rect1 & rect2;
    cv::Rect union_rect = rect1 | rect2;
    if (union_rect.area() == 0) {
        *p_overlap_ratio = 0;
    }
    *p_overlap_ratio = static_cast<float>(inter_rect.area()) / union_rect.area();
}

int main(int argc, char** argv) {
    std::string txtdir;
    std::string jsondir;
    std::string imgdir;
    std::string list_name;
    std::string config_name;
    float pixels_per_meter = 50.0f;
    float dst_width_meters = 50.0f;
    float dst_length_meters = 160.0f;
    int top_num = 3;
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "/?") {
            printUsage();
            return -1;
        }
        else if (std::string(argv[i]) == "--txtdir") {
            txtdir = argv[i + 1];
            ++i;
        }
        else if (std::string(argv[i]) == "--imgdir") {
            imgdir = argv[i + 1];
            ++i;
        }
        else if (std::string(argv[i]) == "--jsondir") {
            jsondir = argv[i + 1];
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
        else if (std::string(argv[i]) == "--top") {
            top_num = atoi(argv[i + 1]);
            ++i;
        }
    }

    if (txtdir.empty() || imgdir.empty() || jsondir.empty() || list_name.empty()
        || config_name.empty()) {
        printUsage();
        return -1;
    }

    if (txtdir[txtdir.length() - 1] != '/' || txtdir[txtdir.length() - 1] != '\\') {
        txtdir += "/";
    }
    if (jsondir[jsondir.length() - 1] != '/' || jsondir[jsondir.length() - 1] != '\\') {
        jsondir += "/";
    }
    if (imgdir[imgdir.length() - 1] != '/' || imgdir[imgdir.length() - 1] != '\\') {
        imgdir += "/";
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

    std::map<int, LableSt> labelst_map;
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
        std::string txtpath = txtdir + pid + ".txt";
        std::ifstream txtfile(txtpath.c_str());
        if (!txtfile.good()) {
            std::cout << "[WARN] Can not open txt file " << txtpath << std::endl;
            continue;
        }
        std::vector<DetRes> detect_res;
        while (!txtfile.eof()) {
            DetRes det_res;
            txtfile >> det_res.rect.x;
            txtfile >> det_res.rect.y;
            txtfile >> det_res.rect.width;
            txtfile >> det_res.rect.height;
            if (det_res.rect.x == 0 && det_res.rect.y == 0
                && det_res.rect.width == 0 && det_res.rect.height) {
                break;
            }
            LabelScore label_score;
            for (int l = 0; l < 10; ++l) {
                txtfile >> label_score.label;
                txtfile >> label_score.score;
                det_res.labels_score.push_back(label_score);
            }
            detect_res.push_back(det_res);
        }
        //// 按照识别置信度对检测窗口排序
        //float maxconf;
        //for (int i = 0; i < detect_res.size(); i++)
        //{
        //    maxconf = 0.0;
        //    int mid = i;
        //    for (int j = i + 1; j < detect_res.size(); j++)
        //    {
        //        if (detect_res[j].labels_score[0].score > maxconf)
        //        {
        //            maxconf = detect_res[j].labels_score[0].score;
        //            mid = j;
        //        }
        //    }
        //    // 交换 i 和 mid
        //    DetRes temp = detect_res[i];
        //    detect_res[i] = detect_res[mid];
        //    detect_res[mid] = temp;
        //}
        ////去除重复的边界框
        //for (int i = 0; i < detect_res.size(); i++)
        //{
        //    if (detect_res[i].labels_score[0].label == 0) {
        //        continue;
        //    }
        //    cv::Rect det_i = detect_res[i].rect;
        //    for (int j = i + 1; j < detect_res.size(); j++)
        //    {
        //        cv::Rect det_j = detect_res[j].rect;

        //        cv::Rect in_rect = det_i & det_j;
        //        cv::Rect un_rect = det_i | det_j;
        //        if (in_rect.area() == 0) {
        //            continue;
        //        }

        //        float overlap = static_cast<float>(in_rect.area()) / un_rect.area();
        //        if (overlap > 0.3) {
        //            //detect_res[i].labels_score[2].label = detect_res[j].labels_score[0].label;
        //            std::vector<DetRes>::iterator it = detect_res.begin() + j;
        //            detect_res.erase(it);
        //            --j;
        //        }
        //    }
        //}

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

        // recall
        for (int m = 0; m < static_cast<int>(mark.sample_marks.size()); ++m) {
            if (mark.sample_marks[m].pts.size() <= 5) {
                continue;
            }
            if (mark.sample_marks[m].type_code == 69999) {
                continue;
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
            rect.x -= static_cast<int>(rect.width * 0.25);
            rect.y -= static_cast<int>(rect.height * 0.25);
            rect.width = static_cast<int>(rect.width * 1.5);
            rect.height = static_cast<int>(rect.height * 1.5);
            cv::Point2f center(0.0f, 0.0f);
            for (int mp = 0; mp < static_cast<int>(road_pts.size()); ++mp) {
                center.x += road_pts[mp].x;
                center.y += road_pts[mp].y;
            }
            center.x /= static_cast<float>(road_pts.size());
            center.y /= static_cast<float>(road_pts.size());
            //if (road_pts[0].x > center.x) {
            //    continue;
            //}
            if (center.x < MIN_X || center.x > MAX_X) {
                continue;
            }
            if (center.y < MIN_Y || center.y > MAX_Y) {
                continue;
            }
            labelst_map[mark.sample_marks[m].type_code].total++;
            bool isfind = false;
            for (int dt = 0; dt < static_cast<int>(detect_res.size()); ++dt) {
                //int tp = 0;
                //for (int tp = 0; tp < top_num; ++tp) {
                //    if (detect_res[dt].labels_score[tp].label == 0) {
                //        if (detect_res[dt].labels_score[tp].score < 0.8f) {
                //            tp = 1;
                //        }
                //    }
                //    if (detect_res[dt].labels_score[tp].label == mark.sample_marks[m].type_code) {
                //        float ratio;
                //        cal_overlap(rect, detect_res[dt].rect, &ratio);
                //        if (ratio >= MIN_OVERLAP) {
                //            isfind = true;
                //            break;
                //        }
                //    }
                //}
                if (detect_res[dt].labels_score[0].label == 0
                    && detect_res[dt].labels_score[0].score > NEG_THR) {
                    continue;
                }
                if (detect_res[dt].rect.width < DET_MIN_SIZE
                    || detect_res[dt].rect.height < DET_MIN_SIZE) {
                    continue;
                }
                /*if (detect_res[dt].labels_score[0].label != 0
                    || (detect_res[dt].labels_score[0].score < 0.8f && detect_res[dt].labels_score[0].label == 0)) {*/
                    for (int tp = 0; tp < top_num; ++tp) {
                        if (detect_res[dt].labels_score[tp].label == mark.sample_marks[m].type_code) {
                            float ratio;
                            cal_overlap(rect, detect_res[dt].rect, &ratio);
                            if (ratio >= MIN_OVERLAP) {
                                isfind = true;
                                break;
                            }
                        }
                    }
                //}
                if (isfind) {
                    break;
                }
            }
            if (isfind) {
                labelst_map[mark.sample_marks[m].type_code].find++;
            }
            else {
                labelst_map[mark.sample_marks[m].type_code].loss++;
            }
        }

        // precision
        for (int dt = 0; dt < static_cast<int>(detect_res.size()); ++dt) {
            if (detect_res[dt].labels_score[0].label == 0
                && detect_res[dt].labels_score[0].score > NEG_THR
                && detect_res[dt].labels_score[1].score < SEC_THR) {
                continue;
            }
            if (detect_res[dt].rect.width == 0 || detect_res[dt].rect.height == 0) {
                continue;
            }
            if (detect_res[dt].rect.width < DET_MIN_SIZE
                || detect_res[dt].rect.height < DET_MIN_SIZE) {
                continue;
            }
            bool isskip = false;
            bool isfind = false;
            int label = 0;
            if (mark.sample_marks.size() == 0) {
                continue;
            }
            for (int m = 0; m < static_cast<int>(mark.sample_marks.size()); ++m) {
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
                rect.x -= static_cast<int>(rect.width * 0.25);
                rect.y -= static_cast<int>(rect.height * 0.25);
                rect.width = static_cast<int>(rect.width * 1.5);
                rect.height = static_cast<int>(rect.height * 1.5);
                float ratio;
                cal_overlap(rect, detect_res[dt].rect, &ratio);
                if (ratio >= MIN_OVERLAP) {
                    if (road_pts.size() <= 5) {
                        isskip = true;
                        break;
                    }
                    cv::Point2f center(0.0f, 0.0f);
                    for (int mp = 0; mp < static_cast<int>(road_pts.size()); ++mp) {
                        center.x += road_pts[mp].x;
                        center.y += road_pts[mp].y;
                    }
                    center.x /= static_cast<float>(road_pts.size());
                    center.y /= static_cast<float>(road_pts.size());
                    /*if (road_pts[0].x > center.x) {
                        isskip = true;
                        break;
                    }*/
                    label = mark.sample_marks[m].type_code;
                    int sel_top_num = top_num;
                    for (int tp = 0; tp < sel_top_num; ++tp) {
                        if (detect_res[dt].labels_score[tp].label == 0) {
                            ++sel_top_num;
                        }
                        if (detect_res[dt].labels_score[tp].label == label) {
                            isfind = true;
                            break;
                        }
                    }
                    if (isfind || isskip) {
                        break;
                    }
                }
            }
            if (isskip) {
                continue;
            }
            if (isfind) {
                labelst_map[label].det_true++;
            }
            else {
                labelst_map[detect_res[dt].labels_score[0].label].det_false++;
#if 0
                // save error images
                std::vector<int> detlabels;
                int sel_top_num = top_num;
                for (int tp = 0; tp < sel_top_num; ++tp) {
                    if (detect_res[dt].labels_score[tp].label == 0) {
                        ++sel_top_num;
                        continue;
                    }
                    detlabels.push_back(detect_res[dt].labels_score[tp].label);
                }
                std::string imname = cv::format("%d_%d_%d_%d_%s.jpg",
                    detlabels[0], detlabels[1], detlabels[2], label, pid.c_str());
                std::string file_path = imgdir + pid + ".jpg";
                cv::Mat img = cv::imread(file_path);
                cv::imwrite(imname, img(detect_res[dt].rect));
#endif
            }
            labelst_map[label].det_total++;
        }
    }
    int sum_total = 0;
    int sum_find = 0;
    int sum_det = 0;
    int sum_corr = 0;
    std::ofstream result_file("log.txt");
    std::map<int, LableSt>::iterator itor = labelst_map.begin();
    for (; itor != labelst_map.end(); ++itor) {
        std::cout << itor->first << std::endl;
        result_file << itor->first << std::endl;
        std::cout << "recall: " << static_cast<float>(itor->second.find) / itor->second.total
            << std::endl;
        result_file << "recall: " << static_cast<float>(itor->second.find) / itor->second.total
            << std::endl;
        std::cout << "precision: "
            << static_cast<float>(itor->second.det_true) / itor->second.det_total << std::endl;
        result_file << "precision: "
            << static_cast<float>(itor->second.det_true) / itor->second.det_total << std::endl;
        result_file << "total: " << itor->second.total << std::endl;
        result_file << "find: " << itor->second.find << std::endl;
        result_file << "loss: " << itor->second.loss << std::endl;
        result_file << "det num: " << itor->second.det_total << std::endl;
        result_file << "det correct: " << itor->second.det_true << std::endl;
        result_file << "det error: " << itor->second.det_false << std::endl;

        sum_total += itor->second.total;
        sum_find += itor->second.find;
        sum_det += itor->second.det_total;
        sum_corr += itor->second.det_true;
    }
    float recall = static_cast<float>(sum_find) / sum_total;
    float percision = static_cast<float>(sum_corr) / sum_det;
    std::cout << "total recall: " << recall << std::endl;
    std::cout << "total percison: " << percision << std::endl;
    result_file << "total recall: " << recall << std::endl;
    result_file << "total percison: " << percision << std::endl;
    result_file.close();
    return 0;
}
