#include "road_convertor.h"

namespace stcv {
RoadConvertor::RoadConvertor() {
    _p_road_to_pano = NULL;
}
RoadConvertor::RoadConvertor(float camera_height, float heading, float pitch, float pitch_back,
    float roll, float pixels_per_meter, float dst_width_meters, float dst_length_meters)
    : _camera_height(camera_height), _heading(heading), _pitch(pitch), _pitch_back(pitch_back),
    _roll(roll), _pixels_per_meter(pixels_per_meter), _dst_height_meters(dst_width_meters),
    _dst_width_meters(dst_length_meters), _p_road_to_pano(NULL) {
    _dst_image_width = static_cast<int>(ceil(_dst_width_meters * _pixels_per_meter));
    _dst_image_height = static_cast<int>(ceil(_dst_height_meters * _pixels_per_meter));
    _p_road_to_pano = NULL;
}

RoadConvertor::~RoadConvertor() {
    delete [] _p_road_to_pano;
    _p_road_to_pano = NULL;
}

void RoadConvertor::init(float camera_height, float heading, float pitch, float pitch_back,
    float roll, float pixels_per_meter, float dst_width_meters, float dst_length_meters) {
    if (_camera_height == camera_height
        && _heading == heading
        && _pitch == pitch
        && _pitch_back == pitch_back
        && _roll == roll
        && _pixels_per_meter == pixels_per_meter
        && _dst_height_meters == dst_width_meters
        && _dst_width_meters == dst_length_meters
        && _p_road_to_pano != NULL) {
        return;
    }
    _camera_height = camera_height;
    _heading = heading;
    _pitch = pitch;
    _pitch_back = pitch_back;
    _roll = roll;
    _pixels_per_meter = pixels_per_meter;
    _dst_height_meters = dst_width_meters;
    _dst_width_meters = dst_length_meters;
    _dst_image_width = static_cast<int>(ceil(_dst_width_meters * _pixels_per_meter));
    _dst_image_height = static_cast<int>(ceil(_dst_height_meters * _pixels_per_meter));

    delete [] _p_road_to_pano;
    _p_road_to_pano = NULL;
}

int RoadConvertor::create_road_topview(const cv::Mat& src_img, cv::Mat* p_dst_img) {
    if (src_img.empty()) {
        return ROADCVTOR_INPUT_PARAM_ERROR;
    }

    int dst_width = dst_image_width();
    int dst_height = dst_image_height();
    if (_p_road_to_pano == NULL) {
        _p_road_to_pano = new PixelRelation[dst_width * dst_height];
        // 生成道路到全景图的转换表
        for (int y = 0; y < dst_height; ++y) {
            for (int x = 0; x < dst_width; ++x) {
                cv::Point2f pano_pt;
                int ret = road_to_pano(src_img.cols, src_img.rows, cv::Point2i(x, y), &pano_pt);
                if (ret == ROADCVTOR_OK) {
                    if (pano_pt.x < 0) {
                        pano_pt.x = 0.0f;
                    }
                    else if (pano_pt.x > src_img.cols - 1) {
                        pano_pt.x = static_cast<float>(src_img.cols - 1);
                    }
                    if (pano_pt.y < 0) {
                        pano_pt.y = 0.0f;
                    }
                    else if (pano_pt.y > src_img.rows - 1) {
                        pano_pt.y = static_cast<float>(src_img.rows - 1);
                    }
                    _p_road_to_pano[y*dst_width + x].accurate_pt = pano_pt;
                    set_neighbor_weight(&_p_road_to_pano[y*dst_width + x]);
                }
                else {
                    return ret;
                }
            }
        }
    }

    p_dst_img->create(cv::Size(dst_width, dst_height), src_img.type());
    uchar* p_dst_data = p_dst_img->data;
    uchar* pixel = new uchar[4];
    int channels = static_cast<int>(p_dst_img->step[1]);
    int step = static_cast<int>(p_dst_img->step[0]);
    // 根据转换表生成道路正视图
    for (int y = 0; y < p_dst_img->rows; ++y) {
        for (int x = 0; x < p_dst_img->cols; ++x) {
            int ret = get_pano_pixel(src_img, x, y, &pixel);
            if (ret == ROADCVTOR_OK) {
                for (int c = 0; c < channels; ++c) {
                    int index = static_cast<int>(y * step + x * channels + c);
                    p_dst_data[index] = pixel[c];
                }
            }
        }
    }
    delete[] pixel;
    pixel = NULL;

    return ROADCVTOR_OK;
}

int RoadConvertor::pts_pano_to_road(const int& pano_width, const int& pano_height, 
    const std::vector<cv::Point2i>& pano_pts,
    std::vector<cv::Point2i>* p_road_pts) {
    if (pano_pts.size() > 0) {
        cv::Point2f road_pt;
        p_road_pts->resize(pano_pts.size());
        for (int i = 0; i < static_cast<int>(pano_pts.size()); ++i) {
            int ret = pano_to_road(pano_width, pano_height,
                pano_pts[i], &road_pt);
            if (ret != ROADCVTOR_OK) {
                return ret;
            }
            (*p_road_pts)[i].x = static_cast<int>(road_pt.x);
            (*p_road_pts)[i].y = static_cast<int>(road_pt.y);
        }
    }
    return ROADCVTOR_OK;
}

int RoadConvertor::pts_road_to_pano(const int& pano_width, const int& pano_height,
    const std::vector<cv::Point2i>&road_pts,
    std::vector<cv::Point2i>* p_pano_pts) {
    if (road_pts.size() > 0) {
        cv::Point2f pano_pt;
        p_pano_pts->resize(road_pts.size());
        for (int i = 0; i < static_cast<int>(road_pts.size()); ++i) {
            int ret = road_to_pano(pano_width, pano_height,
                road_pts[i], &pano_pt);
            if (ret != ROADCVTOR_OK) {
                return ret;
            }
            (*p_pano_pts)[i].x = static_cast<int>(pano_pt.x);
            (*p_pano_pts)[i].y = static_cast<int>(pano_pt.y);
        }
    }
    return ROADCVTOR_OK;
}

int RoadConvertor::road_to_pano(const int& pano_width, const int& pano_height, 
    const cv::Point2i& road_pt, cv::Point2f* p_pano_pt) {
    int dst_width = dst_image_width();
    int dst_height = dst_image_height();
    if (road_pt.x < 0 || road_pt.x >= dst_width 
        || road_pt.y < 0 || road_pt.y >= dst_height) {
        return ROADCVTOR_INPUT_PARAM_ERROR;
    }

    float road_x = static_cast<float>(road_pt.x) - 0.5f * dst_width;
    float road_y = static_cast<float>(road_pt.y) - 0.5f * dst_height;
    float x_3d = -road_x / pixels_per_meter();
    float y_3d = road_y / pixels_per_meter();
    float dis = sqrt(x_3d * x_3d + y_3d * y_3d);

    float heading_ = static_cast<float>(heading() * CV_PI / 180.0f);

    // 转换为球坐标
    double theta = (CV_PI - atan(dis / _camera_height));
    double phi = -atan2(y_3d, x_3d);
    phi += (_heading * CV_PI / 180.0f);
    if (phi < 0) {
        phi += CV_2PI;
    }
    else if (phi > CV_2PI) {
        phi -= CV_2PI;
    }

    float pitch_ = static_cast<float>(pitch() * CV_PI / 180.0f);
    if (road_x > 0) {
        pitch_ = static_cast<float>(pitch_back() * CV_PI / 180.0f);
    }
    float sph_x = static_cast<float>(sin(theta) * cos(phi - heading_));
    float sph_y = static_cast<float>(sin(theta) * sin(phi - heading_));
    float sph_z = static_cast<float>(cos(theta));
    cv::Mat sph_3d(3, 1, CV_32FC1, cv::Scalar(0));
    sph_3d.at<float>(0, 0) = sph_x;
    sph_3d.at<float>(1, 0) = sph_y;
    sph_3d.at<float>(2, 0) = sph_z;
    float r_[9] = { 0 };
    r_[0] = cos(pitch_);
    r_[2] = sin(pitch_);
    r_[4] = 1.0f;
    r_[6] = -sin(pitch_);
    r_[8] = cos(pitch_);
    cv::Mat r_pitch(3, 3, CV_32FC1, r_);
    cv::Mat t_sph = r_pitch * sph_3d;
    float x = t_sph.at<float>(0, 0);
    float y = t_sph.at<float>(1, 0);
    float z = t_sph.at<float>(2, 0);
    phi = atan2(y, x) + heading_;
    theta = atan(sqrt(x * x + y * y) / z);
    if (phi < 0) {
        phi += CV_2PI;
    }
    else if (phi > CV_2PI) {
        phi -= CV_2PI;
    }
    if (theta < 0) {
        theta += CV_PI;
    }

    // 转换为全景图像素坐标
    p_pano_pt->x = static_cast<float>(phi / CV_2PI * pano_width);
    p_pano_pt->y = static_cast<float>(theta / CV_PI * pano_height);

    if (p_pano_pt->x >= pano_width) {
        p_pano_pt->x = static_cast<float>(pano_width - 1);
    }
    else if (p_pano_pt->x < 0) {
        p_pano_pt->x = 0;
    }
    if (p_pano_pt->y >= pano_height) {
        p_pano_pt->y = static_cast<float>(pano_height - 1);
    }
    else if (p_pano_pt->y < 0) {
        p_pano_pt->y = 0;
    }

    return ROADCVTOR_OK;
}

int RoadConvertor::pano_to_road(const int& pano_width, const int& pano_height,
    const cv::Point2f& pano_pt, cv::Point2f* p_road_pt) {
    int dst_width = dst_image_width();
    int dst_height = dst_image_height();
    if (pano_pt.x < 0 || pano_pt.x >= pano_width
        || pano_pt.y < 0 || pano_pt.y >= pano_height) {
        return ROADCVTOR_INPUT_PARAM_ERROR;
    }

    // 转换为球坐标
    double theta = (pano_pt.y / pano_height) * CV_PI;
    double phi = (pano_pt.x / pano_width) * CV_2PI;
    phi -= (_heading * CV_PI / 180.0);
    if (phi < -CV_PI) {
        phi += CV_2PI;
    }
    else if (phi > CV_PI) {
        phi -= CV_2PI;
    }

    //if (theta <= CV_PI/2) {
    //    return ROADCVTOR_INPUT_PARAM_ERROR;
    //}

    float pitch_ = static_cast<float>(pitch() * CV_PI / 180.0f);
    if (phi > 0.5 * CV_PI || phi < -0.5 * CV_PI) {
        pitch_ = static_cast<float>(pitch_back() * CV_PI / 180.0f);
    }
    pitch_ = -pitch_;
    float sph_x = static_cast<float>(sin(theta) * cos(phi));
    float sph_y = static_cast<float>(sin(theta) * sin(phi));
    float sph_z = static_cast<float>(cos(theta));
    cv::Mat sph_3d(3, 1, CV_32FC1, cv::Scalar(0));
    sph_3d.at<float>(0, 0) = sph_x;
    sph_3d.at<float>(1, 0) = sph_y;
    sph_3d.at<float>(2, 0) = sph_z;
    float r_[9] = { 0 };
    r_[0] = cos(pitch_);
    r_[2] = sin(pitch_);
    r_[4] = 1.0f;
    r_[6] = -sin(pitch_);
    r_[8] = cos(pitch_);
    cv::Mat r_pitch(3, 3, CV_32FC1, r_);
    cv::Mat t_sph = r_pitch * sph_3d;
    float x = t_sph.at<float>(0, 0);
    float y = t_sph.at<float>(1, 0);
    float z = t_sph.at<float>(2, 0);
    phi = atan2(y, x);
    theta = atan(sqrt(x * x + y * y) / z);
    if (theta < 0) {
        theta += CV_PI;
    }
    if (phi < 0) {
        phi += CV_2PI;
    }
    else if (phi > CV_2PI) {
        phi -= CV_2PI;
    }

    float dis = camera_height() * static_cast<float>(tan(CV_PI - theta));
    int x_s = 1;
    int y_s = 1;
    float tan_phi = 0.0f;
    if (phi < 0.5 * CV_PI) {
        tan_phi = static_cast<float>(tan(phi));
        x_s = 1;
        y_s = -1;
    }
    else if (phi < CV_PI) {
        tan_phi = static_cast<float>(tan(CV_PI - phi));
        x_s = -1;
        y_s = -1;
    }
    else if (phi < 1.5 * CV_PI) {
        tan_phi = static_cast<float>(tan(phi - CV_PI));
        x_s = -1;
        y_s = 1;
    }
    else {
        tan_phi = static_cast<float>(tan(CV_2PI - phi));
        x_s = 1;
        y_s = 1;
    }
    float x_3d = x_s * dis / sqrt((1 + tan_phi * tan_phi));
    float y_3d = y_s * sqrt(dis * dis - x_3d * x_3d);
    float road_x = -x_3d * pixels_per_meter();
    float road_y = y_3d * pixels_per_meter();
    road_x += (0.5f * dst_width);
    road_y += (0.5f * dst_height);
    p_road_pt->x = road_x;
    p_road_pt->y = road_y;

    //if (p_road_pt->x < 0 || p_road_pt->x >= dst_width 
    //    || p_road_pt->y < 0 || p_road_pt->y >= dst_height) {
    //    return ROADCVTOR_PARAM_OUT_OF_RANGE;
    //}
    if (p_road_pt->x < 0) {
        p_road_pt->x = 0;
    }
    else if (p_road_pt->x >= dst_width) {
        p_road_pt->x = static_cast<float>(dst_width - 1);
    }
    if (p_road_pt->y < 0) {
        p_road_pt->y = 0;
    }
    else if (p_road_pt->y >= dst_height) {
        p_road_pt->y = static_cast<float>(dst_height - 1);
    }

    return ROADCVTOR_OK;
}

void RoadConvertor::set_neighbor_weight(PixelRelation* p_pixel_relation) {
    float min_x = floor(p_pixel_relation->accurate_pt.x);
    float min_y = floor(p_pixel_relation->accurate_pt.y);
    float max_x = ceil(p_pixel_relation->accurate_pt.x);
    float max_y = ceil(p_pixel_relation->accurate_pt.y);
    p_pixel_relation->pixel_info[0].pt.x = static_cast<int>(min_x);
    p_pixel_relation->pixel_info[0].pt.y = static_cast<int>(min_y);
    p_pixel_relation->pixel_info[1].pt.x = static_cast<int>(max_x);
    p_pixel_relation->pixel_info[1].pt.y = static_cast<int>(min_y);
    p_pixel_relation->pixel_info[2].pt.x = static_cast<int>(max_x);
    p_pixel_relation->pixel_info[2].pt.y = static_cast<int>(max_y);
    p_pixel_relation->pixel_info[3].pt.x = static_cast<int>(min_x);
    p_pixel_relation->pixel_info[3].pt.y = static_cast<int>(max_y);
    float x = p_pixel_relation->accurate_pt.x;
    float y = p_pixel_relation->accurate_pt.y;
    // 双线性插值
    if (static_cast<int>(min_x) == static_cast<int>(max_x)
        && static_cast<int>(min_y) == static_cast<int>(max_y)) {
        p_pixel_relation->pixel_info[0].weight = 1.0f;
        p_pixel_relation->pixel_info[1].weight = 0.0f;
        p_pixel_relation->pixel_info[2].weight = 0.0f;
        p_pixel_relation->pixel_info[3].weight = 0.0f;
    }
    else if (static_cast<int>(min_x) == static_cast<int>(max_x)) {
        p_pixel_relation->pixel_info[0].weight = (max_y - y);
        p_pixel_relation->pixel_info[1].weight = 0.0f;
        p_pixel_relation->pixel_info[2].weight = 0.0f;
        p_pixel_relation->pixel_info[3].weight = (y - min_y);
    }
    else if (static_cast<int>(min_y) == static_cast<int>(max_y)) {
        p_pixel_relation->pixel_info[0].weight = (max_x - x);
        p_pixel_relation->pixel_info[1].weight = (x - min_x);
        p_pixel_relation->pixel_info[2].weight = 0.0f;
        p_pixel_relation->pixel_info[3].weight = 0.0f;
    }
    else {
        p_pixel_relation->pixel_info[0].weight = (max_x - x) * (max_y - y);
        p_pixel_relation->pixel_info[1].weight = (x - min_x) * (max_y - y);
        p_pixel_relation->pixel_info[2].weight = (x - min_x) * (y - min_y);
        p_pixel_relation->pixel_info[3].weight = (max_x - x) * (y - min_y);
    }
}

int RoadConvertor::get_pano_pixel(const cv::Mat& pano, const int& road_x,
    const int& road_y, uchar** p_pixel) {
    int dst_width = dst_image_width();
    PixelWeight* p_pixel_w = _p_road_to_pano[dst_width * road_y + road_x].pixel_info;
    double sp[4] = { 0 };
    int channels = pano.channels();
    int step = static_cast<int>(pano.step[0]);
    for (int i = 0; i < 4; ++i) {
        if (p_pixel_w[i].pt.y < 0 || p_pixel_w[i].pt.y >= pano.rows
            || p_pixel_w[i].pt.x < 0 || p_pixel_w[i].pt.x >= pano.cols) {
            return ROADCVTOR_PARAM_OUT_OF_RANGE;
        }

        int y = p_pixel_w[i].pt.y;
        int x = p_pixel_w[i].pt.x;
        float w = p_pixel_w[i].weight;
        uchar* p_data = pano.data;
        int pos = y * step + x * channels;
        for (int c = 0; c < channels; ++c) {
            float p = static_cast<float>(p_data[pos + c]);
            p *= w;
            sp[c] += p;
        }
    }
    for (int c = 0; c < channels; ++c) {
        (*p_pixel)[c] = static_cast<uchar>(sp[c] + 0.5);
    }

    return ROADCVTOR_OK;
}

int RoadConvertor::plane_transfrom(const float& device_height_pixels, const float& pitch,
    const cv::Point2f& virtual_pt, cv::Point2f* p_actual_pt) {
    if (abs(virtual_pt.x) < 1 || abs(pitch) < 1 * CV_PI / 180.0f) {
        p_actual_pt->x = virtual_pt.x;
        p_actual_pt->y = virtual_pt.y;
        return ROADCVTOR_OK;
    }
    float x = virtual_pt.x;
    float y = virtual_pt.y;

    if (pitch * x > 0) {
        float alpha = abs(atan(device_height_pixels / x));
        float pitch_radian = static_cast<float>(abs(pitch / 180.0f * CV_PI));
        float h = static_cast<float>(x / (1.0 / tan(abs(pitch_radian)) + 1.0 / tan(alpha)));
        p_actual_pt->x = h / sin(abs(pitch_radian));
        float dis = sqrt(device_height_pixels * device_height_pixels + x * x);
        p_actual_pt->y = (1 - abs(h) / (sin(alpha) * dis)) * y;
    }
    else {
        float alpha1 = abs(atan(device_height_pixels / x));
        float pitch_radian = static_cast<float>(abs(pitch / 180.0f * CV_PI));
        float alpha2 = alpha1 - pitch_radian;
        float h = x * sin(pitch_radian);
        p_actual_pt->x = h * (1.0f / tan(pitch_radian) + 1.0f / tan(alpha2));
        float dis = sqrt(device_height_pixels * device_height_pixels + x * x);
        p_actual_pt->y = (1 + abs(h) / (sin(alpha2) * dis)) * y;
    }

    return ROADCVTOR_OK;
}

int RoadConvertor::plane_inverse_transfrom(const float& device_height_pixels, const float& pitch,
    const cv::Point2f& actual_pt, cv::Point2f* p_virtual_pt) {
    if (abs(actual_pt.x) < 1 || abs(pitch) < 1 * CV_PI / 180.0f) {
        p_virtual_pt->x = actual_pt.x;
        p_virtual_pt->y = actual_pt.y;
        return ROADCVTOR_OK;
    }

    float x = actual_pt.x;
    float y = actual_pt.y;
    float pitch_radian = static_cast<float>(pitch / 180.0f * CV_PI);
    float xcos = cos(pitch_radian) * x;
    float xsin = sin(pitch_radian) * x;

    p_virtual_pt->x = device_height_pixels * xcos / (device_height_pixels - xsin);
    float dis = sqrt(device_height_pixels * device_height_pixels
        + p_virtual_pt->x * p_virtual_pt->x);
    float alpha = atan(device_height_pixels / abs(p_virtual_pt->x));
    float h = sin(pitch_radian) * x;
    p_virtual_pt->y = y / (1 - h / (sin(alpha) * dis));

    return ROADCVTOR_OK;
}
}

