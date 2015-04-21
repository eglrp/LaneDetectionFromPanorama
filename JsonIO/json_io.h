// Copyright 2015 Baidu Inc. All Rights Reserved.
// Author:  Hou Wenbo (houwenbo@baidu.com)
//
// Parse and generate the json file for samples.
//
#ifndef _STCV_DATAIO_JSON_IO_H_
#define _STCV_DATAIO_JSON_IO_H_
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
    TypeName(const TypeName &); \
    TypeName& operator=(const TypeName&)
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
const int VERSION = 1002;
namespace stcv {
namespace json_io {
/// \class  JsonIO
///
/// \brief  Sample data (json) io class.
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
    /// \brief  Base info of sample.
    ///
    /// \author Houwenbo
    /// \date   2015/3/17
    struct SampleMark {
        SampleMark() {
            type_code = 0;
            is_tool_import = false;
        }
        std::string pid;
        int type_code;
        GraphicType graph;
        std::vector<cv::Point> pts;
        std::string comment;
        bool is_tool_import;
    };

    /// \struct JsonStruct
    ///
    /// \brief  sample data struct, for one panorama image.
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
    /// \brief  parse the json file.
    ///
    /// \author Houwenbo
    /// \date   2015/3/17
    ///
    /// \param  json            The JSON.
    /// \param [in,out] p_mark  If non-null, the mark.
    ///
    /// \return Success 0, Failure < 0.
    static int parse_json(const std::string& json, JsonStruct* p_mark);

    /// \fn static int JsonIO::generate_json(const JsonStruct& mark, std::string* p_json);
    ///
    /// \brief  generate the json file.
    ///
    /// \author Houwenbo
    /// \date   2015/3/17
    ///
    /// \param  mark            The mark.
    /// \param [in,out] p_json  If non-null, the JSON.
    ///
    /// \return Success 0, Failure < 0.
    static int generate_json(const JsonStruct& mark, std::string* p_json);
private:
};
}
}
#endif
