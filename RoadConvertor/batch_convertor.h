#ifndef STCV_BATCH_CONVERTOR
#define STCV_BATCH_CONVERTOR
#include <map>
#include <string>
#include <iostream>
#include "road_convertor.h"
namespace stcv {
namespace road_cvtor {
struct RoadCvtParm {
    float camera_height;
    float heading;
    float pitch;
    float pitch_back;
};

class BatchConvertor
{
public:
    BatchConvertor();
    ~BatchConvertor();

    int init(const std::map<std::string, RoadCvtParm>& parm_map, const float& pixels_per_meter,
        const float& road_width_meters, const float& road_length_meters);

    int road_cvtor(const std::string& prefix, const cv::Mat& src_img, cv::Mat* p_dst_img);

    int pts_pano_to_road(const int& pano_width, const int& pano_height,
        const std::vector<cv::Point2i>& pano_pts,
        std::vector<cv::Point2i>* p_road_pts);
    int pts_road_to_pano(const int& pano_width, const int& pano_height,
        const std::vector<cv::Point2i>&road_pts,
        std::vector<cv::Point2i>* p_pano_pts);

private:
    /// \brief  同一批次数据的名字，如20150104TT_1057515，前缀为20150104TT.
    std::string _prefix_name;
    float _road_width_meters;
    float _road_length_meters;
    float _pixels_per_meter;
    /// \brief  道路转换类.
    RoadConvertor _road_convertor;
    /// \brief  每批次数据对应的参数.
    std::map<std::string, RoadCvtParm> _parm_map;

    DISALLOW_COPY_AND_ASSIGN(BatchConvertor);
};
} // namespace road_cvtor
} // namespace stcv
#endif
