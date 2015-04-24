#include "json_io.h"
#include "road_convertor.h"
#include <fstream>
#include <string>
#include <map>

struct RoadCvtParm {
    float camera_height;
    float heading;
    float pitch;
    float pitch_back;
};

const int DST_HEIGHT_METERS = 50;
const int DST_WIDTH_METERS = 160;
const int PIXELS_PER_METERS = 50;

int main() {
    std::map<std::string, RoadCvtParm> road_cvt_parm;
    std::ifstream topviewconfig("topview_config.txt");
    if (topviewconfig.good()) {
        std::string batch;
        while (topviewconfig >> batch) {
            if (batch.empty()) {
                break;
            }

            float camera_height;
            float heading;
            float pitch;
            float pitch_back;

            topviewconfig >> camera_height;
            topviewconfig >> heading;
            topviewconfig >> pitch;
            topviewconfig >> pitch_back;

            RoadCvtParm parm;
            parm.camera_height = camera_height;
            parm.heading = heading;
            parm.pitch = pitch;
            parm.pitch_back = pitch_back;
            road_cvt_parm.insert(std::pair<std::string, RoadCvtParm>(batch, parm));
        }
    }

    std::string cur_batch;
    stcv::road_cvtor::RoadConvertor roadconvertor(2.32f, 187, 2, 3, 0, PIXELS_PER_METERS, DST_HEIGHT_METERS, DST_WIDTH_METERS);
    std::ifstream listfile("xiaoteng.txt");
    std::string json_dir = "./json_0423_xiaoteng/";
    if (!listfile.good()) {
        return -1;
    }
    std::string pid;
    while (listfile >> pid) {
        if (pid.empty()) {
            break;
        }
        std::string batch = pid.substr(0, 10);
        if (cur_batch != batch) {
            cur_batch = batch;
            std::map<std::string, RoadCvtParm>::iterator it = road_cvt_parm.find(batch);
            if (it == road_cvt_parm.end())
            {
                roadconvertor.init(2.32, 180, 0, 0, 0, PIXELS_PER_METERS, DST_HEIGHT_METERS, DST_WIDTH_METERS);
            }
            else
            {
                float height = road_cvt_parm[batch].camera_height;
                float heading = road_cvt_parm[batch].heading;
                float pitch = road_cvt_parm[batch].pitch;
                float pitch_back = road_cvt_parm[batch].pitch_back;
                roadconvertor.init(height, heading, pitch, pitch_back, 0, PIXELS_PER_METERS, DST_HEIGHT_METERS, DST_WIDTH_METERS);
            }
        }
        std::ofstream otxt(pid + ".txt");
        std::string jsonname = json_dir + pid;
        std::ifstream jsonfile(jsonname.c_str());
        if (jsonfile.good()) {
            std::string json;
            jsonfile >> json;
            if (!json.empty()) {
                stcv::json_io::JsonIO::JsonStruct mark;
                stcv::json_io::JsonIO::parse_json(json, &mark);
                for (int m = 0; m < mark.sample_marks.size(); ++m) {
                    if (mark.sample_marks[m].type_code / 10000 == 6) {
                        std::vector<cv::Point> roadpts;
                        if (mark.sample_marks[m].pts.size() <= 3) {
                            continue;
                        }
                        int ret = roadconvertor.pts_pano_to_road(8192, 4096, mark.sample_marks[m].pts, &roadpts);
                        if (ret == stcv::road_cvtor::RoadConvertor::ROADCVTOR_OK) {
                            int minx = INT_MAX;
                            int miny = INT_MAX;
                            int maxx = 0;
                            int maxy = 0;
                            for (int p = 0; p < roadpts.size(); ++p) {
                                if (roadpts[p].x < minx) {
                                    minx = roadpts[p].x;
                                }
                                if (roadpts[p].y < miny) {
                                    miny = roadpts[p].y;
                                }
                                if (roadpts[p].x > maxx) {
                                    maxx = roadpts[p].x;
                                }
                                if (roadpts[p].y > maxy) {
                                    maxy = roadpts[p].y;
                                }
                            }
#define CREATE_SQUARE
#ifdef CREATE_SQUARE
                            int x = minx;
                            int y = miny;
                            int width = maxx - minx;
                            int height = maxy - miny;
                            if (width > height) {
                                y -= ((width - height) / 2);
                                height = width;
                            }
                            else if (height > width) {
                                x -= ((height - width) / 2);
                                width = height;
                            }
                            if (x < 0) {
                                x = 0;
                            }
                            if (y < 0) {
                                y = 0;
                            }
                            minx = x;
                            miny = y;
                            maxx = x + width;
                            maxy = y + height;
#endif
                            otxt << minx << '\t' << miny << '\t' << maxx - minx + 1 << '\t' 
                                << maxy - miny + 1 << '\t' << mark.sample_marks[m].type_code << std::endl;
                        }
                    }
                }
            }
            otxt.close();
        }
    }
    listfile.close();
    return 0;
}
