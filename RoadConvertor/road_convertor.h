// Copyright 2015 Baidu Inc. All Rights Reserved.
// Author:  Hou Wenbo (houwenbo@baidu.com)
//
// 文件功能: 将全景图像中的道路区域转换为正视图。
//
#ifndef STCV_ROAD_CONVERTOR_H
#define STCV_ROAD_CONVERTOR_H

#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
    TypeName(const TypeName &); \
    TypeName& operator=(const TypeName&)

#include "opencv2/opencv.hpp"
#ifndef CV_2PI
#define CV_2PI (2 * CV_PI)
#endif

#include <vector>

namespace stcv {
namespace road_cvtor {
/// \class  RoadConvertor
///
/// \brief  将全景图中的道路区域转换为正视图.
///
/// \author Houwenbo
/// \date   2015/4/3
class RoadConvertor {
public:
    enum {
        ROADCVTOR_OK = 0,
        ROADCVTOR_INPUT_PARAM_ERROR = -1,
        ROADCVTOR_PARAM_OUT_OF_RANGE = -2
    };

    RoadConvertor();

    /// \fn RoadConvertor::RoadConvertor(float camera_height, float heading, float pitch,
    ///      float pitch_back, float roll, float pixels_per_meter,
    ///       float dst_height_meters, float dst_width_meters);
    ///
    /// \brief  Constructor.
    ///
    /// \author Houwenbo
    /// \date   2015/4/3
    ///
    /// \param  camera_height       相机镜头中心距离地面的高度.
    /// \param  heading             The heading.
    /// \param  pitch               The pitch.
    /// \param  pitch_back          The pitch back.
    /// \param  roll                The roll.
    /// \param  pixels_per_meter    每米对应的像素数.
    /// \param  dst_width_meters    生成的正视图中道路区域的宽度，单位为米.
    /// \param  dst_length_meters   生成的正视图中道路区域的长度，单位为米.
    RoadConvertor(float camera_height, float heading, float pitch, float pitch_back, 
        float roll, float pixels_per_meter, float dst_width_meters, float dst_length_meters);
    ~RoadConvertor();

    float heading() const {
        return _heading;
    }
    float pitch() const {
        return _pitch;
    }
    float pitch_back() const {
        return _pitch_back;
    }
    float roll() const {
        return _roll;
    }
    float camera_height() const {
        return _camera_height;
    }
    float pixels_per_meter() const {
        return _pixels_per_meter;
    }
    float dst_width_meters() const {
        return _dst_width_meters;
    }
    float dst_height_meters() const {
        return _dst_height_meters;
    }
    int dst_image_width() const{
        return _dst_image_width;
    }
    int dst_image_height() const{
        return _dst_image_height;
    }

    /// \fn void RoadConvertor::init(float camera_height, float heading, float pitch,
    ///      float pitch_back, float roll, float pixels_per_meter,
    ///       float dst_height_meters, float dst_width_meters);
    ///
    /// \brief  初始化函数，当没有使用带参数的构造函数时，需要调用初始化函数.
    ///
    /// \author Houwenbo
    /// \date   2015/4/3
    ///
    /// \param  camera_height       相机镜头中心距离地面的高度.
    /// \param  heading             The heading.
    /// \param  pitch               The pitch.
    /// \param  pitch_back          The pitch back.
    /// \param  roll                The roll.
    /// \param  pixels_per_meter    图像多少个像素表示1米.
    /// \param  dst_width_meters    生成的正视图中道路区域的宽度，单位为米.
    /// \param  dst_length_meters   生成的正视图中道路区域的长度度，单位为米.
    void init(float camera_height, float heading, float pitch, float pitch_back,
        float roll, float pixels_per_meter, float dst_width_meters, float dst_length_meters);
    int create_road_topview(const cv::Mat& src_img, cv::Mat* p_dst_img);
    int pts_pano_to_road(const int& pano_width, const int& pano_height,
        const std::vector<cv::Point2i>& pano_pts,
        std::vector<cv::Point2i>* p_road_pts);
    int pts_road_to_pano(const int& pano_width, const int& pano_height,
        const std::vector<cv::Point2i>&road_pts,
        std::vector<cv::Point2i>* p_pano_pts);

    /// \fn int RoadConvertor::road_to_pano(const int& pano_width, const int& pano_height,
    ///      const cv::Point2i& road_pt, cv::Point2f* p_pano_pt);
    ///
    /// \brief  道路正视图的坐标到全景图坐标的转换.
    ///
    /// \author Houwenbo
    /// \date   2015/3/26
    ///
    /// \param  pano_width          Width of the pano.
    /// \param  pano_height         Height of the pano.
    /// \param  road_pt             The road point.
    /// \param [in,out] p_pano_pt   If non-null, the pano point.
    ///
    /// \return 成功0，失败非0.
    int road_to_pano(const int& pano_width, const int& pano_height,
        const cv::Point2i& road_pt, cv::Point2f* p_pano_pt);
    int pano_to_road(const int& pano_width, const int& pano_height,
        const cv::Point2f& pano_pt, cv::Point2f* p_road_pt);

private:
    struct PixelWeight {
        float weight;
        cv::Point2i pt;
    };
    struct PixelRelation {
        cv::Point2f accurate_pt;
        PixelWeight pixel_info[4];
    };

    void set_neighbor_weight(PixelRelation* p_pixel_relation);
    int get_pano_pixel(const cv::Mat& pano, const int& road_x,
        const int& road_y, uchar** p_pixel);
    // 由全景图直接投影的理想道路平面，添加pitch变换
    // 输入的点的坐标原点在图像的中心而不是左上角
    int plane_transfrom(const float& device_height_pixels, const float& pitch,
        const cv::Point2f& virtual_pt, cv::Point2f* p_actual_pt);
    // 实际道路正视图转换为在理想投影的道路坐标
    int plane_inverse_transfrom(const float& device_height_pixels, const float& pitch,
        const cv::Point2f& actual_pt, cv::Point2f* p_virtual_pt);

    float _camera_height;
    float _heading;
    float _pitch;
    float _pitch_back;
    float _roll;
    float _pixels_per_meter;
    float _dst_height_meters;
    float _dst_width_meters;
    int _dst_image_width;
    int _dst_image_height;
    PixelRelation* _p_road_to_pano;

    DISALLOW_COPY_AND_ASSIGN(RoadConvertor);
};
} // namespace road_cvtor
} // namespace stcv
#endif
