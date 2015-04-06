#include "batch_convertor.h"

namespace stcv {
namespace road_cvtor {
BatchConvertor::BatchConvertor()
{
}

BatchConvertor::~BatchConvertor()
{
}

int BatchConvertor::init(const std::map<std::string, RoadCvtParm>& parm_map,
    const float& pixels_per_meter, const float& road_width_meters,
    const float& road_length_meters) {
    _parm_map = parm_map;
    _pixels_per_meter = pixels_per_meter;
    _road_width_meters = road_width_meters;
    _road_length_meters = road_length_meters;
    if (_parm_map.empty()) {
        std::cout << "[ERROR] In BatchConvertor init, _parm_map is empty." << std::endl;
        return RoadConvertor::ROADCVTOR_INPUT_PARAM_ERROR;
    }
    return RoadConvertor::ROADCVTOR_OK;
}

int BatchConvertor::road_cvtor(const std::string& prefix, const cv::Mat& src_img,
    cv::Mat* p_dst_img) {
    if (_parm_map.empty()) {
        std::cout << "[ERROR] In BatchConvertor init, _parm_map is empty." << std::endl;
        return RoadConvertor::ROADCVTOR_INPUT_PARAM_ERROR;
    }
    if (prefix != _prefix_name) {
        std::map<std::string, RoadCvtParm>::iterator itor = _parm_map.find(prefix);
        if (itor == _parm_map.end()) {
            std::cout << "[ERROR] can not find the parm in _parm_map," << prefix << std::endl;
            return RoadConvertor::ROADCVTOR_INPUT_PARAM_ERROR;
        }

        float height = _parm_map[prefix].camera_height;
        float heading = _parm_map[prefix].heading;
        float pitch = _parm_map[prefix].pitch;
        float pitch_back = _parm_map[prefix].pitch_back;
        _road_convertor.init(height, heading, pitch, pitch_back, 0, _pixels_per_meter,
            _road_width_meters, _road_length_meters);
        _prefix_name = prefix;
    }
    int ret = _road_convertor.create_road_topview(src_img, p_dst_img);
    return ret;
}

int BatchConvertor::pts_pano_to_road(const int& pano_width, const int& pano_height,
    const std::vector<cv::Point2i>& pano_pts,
    std::vector<cv::Point2i>* p_road_pts) {
    return _road_convertor.pts_pano_to_road(pano_width, pano_height, pano_pts, p_road_pts);
}

int BatchConvertor::pts_road_to_pano(const int& pano_width, const int& pano_height,
    const std::vector<cv::Point2i>&road_pts,
    std::vector<cv::Point2i>* p_pano_pts) {
    return _road_convertor.pts_road_to_pano(pano_width, pano_height, road_pts, p_pano_pts);
}
} // namespace road_cvtor
} // namespace stcv
