#include "evaluate.h"

namespace stcv {
Evaluate::Evaluate(float overlap_area_ratio) : _overlap_area_ratio(overlap_area_ratio)
{
}

Evaluate::~Evaluate()
{
}

int Evaluate::parse_txt(const std::string& file_name, std::vector<DetResult>* p_rects) {
    std::ifstream txt_file(file_name.c_str());
    if (!txt_file.good()) {
        std::cout << "can not open file " << file_name << std::endl;
        return -1;
    }

    p_rects->clear();
    while (!txt_file.eof()) {
        int x = 0;
        int y = 0;
        int w = 0;
        int h = 0;
        int type = 0;
        txt_file >> x >> y >> w >> h >> type;
        if (x == 0 && y == 0) {
            break;
        }
        DetResult det_result;
        det_result.rect.x = x;
        det_result.rect.y = y;
        det_result.rect.width = w;
        det_result.rect.height = h;
        det_result.label = type;
        det_result.is_true = false;
        p_rects->push_back(det_result);
    }
    return 0;
}

void Evaluate::is_overlap(const cv::Rect& rect1, const cv::Rect& rect2, bool* p_is_overlap) {
    cv::Rect inter_rect = rect1 & rect2;
    cv::Rect union_rect = rect1 | rect2;
    if (union_rect.area() == 0) {
        *p_is_overlap = false;
        return;
    }
    float ratio = static_cast<float>(inter_rect.area()) / union_rect.area();
    if (ratio >= _overlap_area_ratio) {
        *p_is_overlap = true;
    }
    else {
        *p_is_overlap = false;
    }
}

int Evaluate::load_groundtruth(const std::string& file_name, const float& extend_ratio) {
    std::vector<DetResult> rects;
    int ret = parse_txt(file_name, &rects);
    if (ret != 0) {
        return ret;
    }
    _groundtruth.clear();
    int rect_num = static_cast<int>(rects.size());
    for (int i = 0; i < rect_num; ++i) {
        int ext_w = static_cast<int>(rects[i].rect.width * extend_ratio + 0.5);
        int ext_h = static_cast<int>(rects[i].rect.height * extend_ratio + 0.5);
        int ext_x = static_cast<int>(rects[i].rect.x - (ext_w - rects[i].rect.width) * 0.5 + 0.5);
        int ext_y = static_cast<int>(rects[i].rect.y - (ext_h - rects[i].rect.height) * 0.5 + 0.5);
        DetResult det_result;
        det_result.rect.x = ext_x;
        det_result.rect.y = ext_y;
        det_result.rect.width = ext_w;
        det_result.rect.height = ext_h;
        det_result.is_true = true;
        det_result.label = rects[i].label;
        _groundtruth.push_back(det_result);
    }
    return 0;
}

int Evaluate::load_detect_result(const std::string& file_name) {
    std::vector<DetResult> rects;
    int ret = parse_txt(file_name, &rects);
    if (ret != 0) {
        return ret;
    }
    _detects.clear();
    int rect_num = static_cast<int>(rects.size());
    for (int i = 0; i < rect_num; ++i) {
        DetResult det_result = rects[i];
        det_result.is_true = false;
        _detects.push_back(det_result);
    }
    return 0;
}

int Evaluate::evaluate(std::vector<DetResult>* p_result) {
    p_result->clear();
    int det_num = static_cast<int>(_detects.size());
    int gt_num = static_cast<int>(_groundtruth.size());
    p_result->resize(det_num);
    for (int i = 0; i < det_num; ++i) {
        (*p_result)[i] = _detects[i];
        for (int j = 0; j < gt_num; ++j) {
            bool isoverlap = false;
            is_overlap((*p_result)[i].rect, _groundtruth[j].rect, &isoverlap);
            if (isoverlap) {
                (*p_result)[i].is_true = true;
                (*p_result)[i].is_used = _groundtruth[j].is_used;
                (*p_result)[i].label = _groundtruth[j].label;
                (*p_result)[i].direction = _groundtruth[j].direction;
                break;
            }
            else {
                (*p_result)[i].is_true = false;
                (*p_result)[i].is_used = _groundtruth[j].is_used;
                (*p_result)[i].label = 0;
                (*p_result)[i].direction = _groundtruth[j].direction;
            }
        }
    }
    return 0;
}

int Evaluate::set_groundtruth(const std::vector<DetResult>& groundtruth) {
    _groundtruth = groundtruth;
    return 0;
}
}

