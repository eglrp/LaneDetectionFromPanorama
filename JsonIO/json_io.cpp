#include "json_io.h"
#include "json/json.h"
#include <cstdio>
namespace stcv {
namespace json_io {
int JsonIO::parse_json(const std::string& json, JsonStruct* p_mark) {
    Json::Reader reader;
    Json::Value root;
    std::string strjson = json;
    if (reader.parse(strjson, root, false)) {
        int version = root["VERSION"].asInt();
        p_mark->version = version;
        p_mark->file_tag = root["FILE_TAG"].asString();
        p_mark->memo = root["MEMO"].asString();
        p_mark->pid = root["PANO_ID"].asString();
        p_mark->qa_date = root["QA_DATE"].asString();
        p_mark->qa_status = root["QA_STATUS"].asInt();
        p_mark->wk_date = root["WK_DATE"].asString();
        p_mark->wk_status = root["WK_STATUS"].asInt();
        if (version == 1000) {
            std::string pid = root["PANO_ID"].asString();
            const Json::Value image_rects = root["RECT_VECTOR"];
            p_mark->sample_marks.resize(image_rects.size());
            for (size_t i = 0; i < image_rects.size(); ++i) {
                std::string str_type = image_rects[i]["TYPE"].asString();
                int nitem = image_rects[i]["ITEM"].asInt();

                std::string pos = image_rects[i]["RECT"].asString();

                cv::Point pt1;
                cv::Point pt2;
                sscanf(pos.c_str(), "(%d,%d)(%d,%d)", &pt1.x, &pt1.y, &pt2.x, &pt2.y);

                SampleMark sample;
                sample.pid = pid;
                sample.type_code = nitem;
                sample.graph = RECTANGLE;
                sample.pts.push_back(pt1);
                sample.pts.push_back(pt2);

                p_mark->sample_marks[i] = sample;
            }
        }
        else {
            std::string pid = root["PANO_ID"].asString();
            const Json::Value image_rects = root["RECT_VECTOR"];
            p_mark->sample_marks.resize(image_rects.size());
            for (int i = 0; i < (int)image_rects.size(); i++) {
                std::string str_type = image_rects[i]["TYPE"].asString();
                int nitem = image_rects[i]["ITEM"].asInt();

                std::string shape = image_rects[i]["SHAPE"].asString();
                if (shape == "RECT") {
                    std::string pos = image_rects[i]["RECT"].asString();
                    cv::Point pt1;
                    cv::Point pt2;
                    sscanf(pos.c_str(), "(%d,%d)(%d,%d)", &pt1.x, &pt1.y, &pt2.x, &pt2.y);

                    SampleMark sample;
                    sample.pid = pid;
                    sample.type_code = nitem;
                    sample.graph = RECTANGLE;
                    sample.pts.push_back(pt1);
                    sample.pts.push_back(pt2);

                    if (!image_rects[i]["COMMENT"].isNull()) {
                        sample.comment = image_rects[i]["COMMENT"].asString();
                    }

                    if (!image_rects[i]["TOOL_IMPORT"].isNull()) {
                        sample.is_tool_import = image_rects[i]["TOOL_IMPORT"].asBool();
                    }

                    p_mark->sample_marks[i] = sample;
                }
                else if (shape == "POLYGON")
                {
                    if (!image_rects[i]["POLYGON"].isNull())
                    {
                        int point_num = image_rects[i]["POINT_NUM"].asInt();
                        std::string strpts = image_rects[i]["POLYGON"].asString();

                        SampleMark sample;
                        sample.pid = pid;
                        sample.type_code = nitem;
                        sample.graph = POLYGON;

                        for (int p = 0; p < point_num; p++)
                        {
                            size_t pos = strpts.find_first_of(")");
                            std::string strpt = strpts.substr(0, pos + 1);
                            strpts = strpts.substr(pos + 1);
                            cv::Point pt;
                            sscanf(strpt.c_str(), "(%d,%d)", &pt.x, &pt.y);

                            sample.pts.push_back(pt);
                        }

                        if (!image_rects[i]["COMMENT"].isNull())
                        {
                            sample.comment = image_rects[i]["COMMENT"].asString();
                        }

                        if (!image_rects[i]["TOOL_IMPORT"].isNull()) {
                            sample.is_tool_import = image_rects[i]["TOOL_IMPORT"].asBool();
                        }

                        p_mark->sample_marks[i] = sample;
                    }

                }
            }
        }
    }
    else {
        return -1;
    }

    return 0;
}
int JsonIO::generate_json(const JsonStruct& mark, std::string* p_json) {
    if (mark.sample_marks.size() == 0) {
        return 0;
    }
    Json::FastWriter writer;
    Json::Value root;
    root["VERSION"] = VERSION;
    root["FILE_TAG"] = (cv::format("TS_%d", VERSION)).c_str();
    root["MEMO"] = "";
    root["PANO_ID"] = mark.pid;
    root["QA_DATE"] = mark.qa_date;
    root["QA_STATUS"] = mark.qa_status;
    root["WK_DATE"] = mark.wk_date;
    root["WK_STATUS"] = mark.wk_status;
    Json::Value samples;
    for (int i = 0; i < static_cast<int>(mark.sample_marks.size()); ++i) {
        Json::Value item;
        item["COMMENT"] = mark.sample_marks[i].comment;
        item["ITEM"] = mark.sample_marks[i].type_code;
        item["POINT_NUM"] = static_cast<int>(mark.sample_marks[i].pts.size());
        item["TYPE"] = "";
        item["TOOL_IMPORT"] = mark.sample_marks[i].is_tool_import;
        if (mark.sample_marks[i].graph == RECTANGLE) {
            item["SHAPE"] = "RECT";
            if (mark.sample_marks[i].pts.size() != 2) {
                continue;
            }
            cv::Point pt0 = mark.sample_marks[i].pts[0];
            cv::Point pt1 = mark.sample_marks[i].pts[1];
            item["RECT"] = cv::format("(%d,%d)(%d,%d)", pt0.x, pt0.y, pt1.x, pt1.y).c_str();
        }
        else if (mark.sample_marks[i].graph == POLYGON) {
            item["SHAPE"] = "POLYGON";
            std::string strpts = "";
            if (mark.sample_marks[i].pts.size() <= 2) {
                continue;
            }
            for (int p = 0; p < static_cast<int>(mark.sample_marks[i].pts.size()); ++p) {
                cv::Point pt = mark.sample_marks[i].pts[p];
                strpts += cv::format("(%d,%d)", pt.x, pt.y);
            }
            item["POLYGON"] = strpts;
        }
        samples.append(item);
    }
    root["RECT_VECTOR"] = samples;
    (*p_json) = writer.write(root);
    return 0;
}
}
}
