#ifndef STCV_EVALUATE_H
#define STCV_EVALUATE_H
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include "opencv2/opencv.hpp"
namespace stcv {
enum FLIP {
    NONE = 0,
    HORIZONTAL = 1,
    VERTIACL = 2
};
enum Direction {
    LEFT = 1,
    RIGHT = 2,
    up = 4,
    DOWN = 8
};
struct DetResult {
    cv::Rect rect;
    bool is_true;// true positive, false negative
    bool is_used;// used for positive
    int label;
    Direction direction;
    FLIP flip;
};
class Evaluate
{
public:
    Evaluate(float overlap_area_ratio);
    ~Evaluate();

    int set_groundtruth(const std::vector<DetResult>& groundtruth);
    int load_groundtruth(const std::string& file_name, const float& extend_ratio);
    int load_detect_result(const std::string& file_name);
    int evaluate(std::vector<DetResult>* p_result);
private:
    int parse_txt(const std::string& file_name, std::vector<DetResult>* p_rects);
    void is_overlap(const cv::Rect& rect1, const cv::Rect& rect2, bool* p_is_overlap);
    void cal_overlap(const cv::Rect& rect1, const cv::Rect& rect2, float* p_overlap_ratio);
    float _overlap_area_ratio;
    std::vector<DetResult> _groundtruth;
    std::vector<DetResult> _detects;
};
}
#endif
