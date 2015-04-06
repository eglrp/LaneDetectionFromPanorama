// Copyright 2015 Baidu Inc. All Rights Reserved.
// Author:  Hou Wenbo (houwenbo@baidu.com)
//
// 文件功能: 样本json解析及生成。
//
#ifndef _STCV_DATAIO_JSON_IO_H_
#define _STCV_DATAIO_JSON_IO_H_
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
    TypeName(const TypeName &); \
    TypeName& operator=(const TypeName&)
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
const int VERSION = 1001;
namespace stcv {
namespace json_io {
/// \class  JsonIO
///
/// \brief  样本数据json io类.
///
/// \author Houwenbo
/// \date   2015/3/17
class JsonIO {
public:
    JsonIO() {

    }
    ~JsonIO() {

    }

    /// \enum   GraphicType
    ///
    /// \brief  Values that represent graphic types.
    enum GraphicType {
        RECTANGLE,
        POLYGON,
        LINE,
        NONE
    };

    /// \struct SampleMark
    ///
    /// \brief  样本的基本信息.
    ///
    /// \author Houwenbo
    /// \date   2015/3/17
    struct SampleMark {
        std::string pid;
        int type_code;
        GraphicType graph;
        std::vector<cv::Point> pts;
        std::string comment;
    };

    /// \struct JsonStruct
    ///
    /// \brief  json内的结构信息，一般为一幅图像内标注的所有样本信息.
    ///
    /// \author Houwenbo
    /// \date   2015/3/17
    struct JsonStruct {
        std::vector<SampleMark> sample_marks;
        int version;
        std::string file_tag;
        std::string memo;
        std::string pid;
        std::string wk_date;
        std::string qa_date;
        int wk_status;
        int qa_status;
    };

    /// \fn static int JsonIO::parse_json(const std::string& json, JsonStruct* p_mark);
    ///
    /// \brief  解析json文件.
    ///
    /// \author Houwenbo
    /// \date   2015/3/17
    ///
    /// \param  json            The JSON.
    /// \param [in,out] p_mark  If non-null, the mark.
    ///
    /// \return 成功0，失败非0.
    static int parse_json(const std::string& json, JsonStruct* p_mark);

    /// \fn static int JsonIO::generate_json(const JsonStruct& mark, std::string* p_json);
    ///
    /// \brief  生成json文件.
    ///
    /// \author Houwenbo
    /// \date   2015/3/17
    ///
    /// \param  mark            The mark.
    /// \param [in,out] p_json  If non-null, the JSON.
    ///
    /// \return 成功0，失败非0.
    static int generate_json(const JsonStruct& mark, std::string* p_json);
private:
};
}
}
#endif
