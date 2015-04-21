// Copyright 2015 Baidu Inc. All Rights Reserved.
// Author:  Hou Wenbo (houwenbo@baidu.com)
//
// Test road convertor.
//
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "../RoadConvertor/road_convertor.h"
struct Mark {
    cv::Rect rect;
    int type_code;
};
int main(int argc, char** argv) {
    std::string txtdir = "D:\\1.Project\\ImgProc\\03.CodeBaseline\\PatterenRecognition\\stcv\\stcv0.2\\projects\\VC10\\proj_jaegers\\";
    std::string listname = "D:\\1.Project\\ImgProc\\03.CodeBaseline\\PatterenRecognition\\stcv\\stcv0.2\\example_data\\test_img.txt";
    std::string imgdir = "D:\\1.Project\\ImgProc\\03.CodeBaseline\\PatterenRecognition\\stcv\\stcv0.2\\example_data\\test_img\\";
    std::string outdir = "E:\\outdir\\";

    const float DEVICE_HEIGHT = 2.32f;
    const float CAR_TOWARD_ANGLE = 270.0f;
    const float PIXELS_PER_METERS = 50.0f;
    const float DST_HEIGHT_METERS = 50.0f;
    const float DST_WIDTH_METERS = 160.0f;
    const int PANO_WIDTH = 8192;
    const int PANO_HEIGHT = 4096;

    stcv::road_cvtor::RoadConvertor road_cvtor;
    road_cvtor.init(DEVICE_HEIGHT, 187.0f, 2.0f,
        4.0f, 0.0f, PIXELS_PER_METERS, DST_HEIGHT_METERS, DST_WIDTH_METERS);

    std::ifstream listfile(listname.c_str());
    if (listfile.fail()) {
        return -1;
    }
    std::vector<std::string> pids_vec;
    std::string pid;
    while (listfile >> pid) {
        if (pid.empty()) {
            break;
        }
        pids_vec.push_back(pid);
    }
    listfile.close();

    for (int i = 0; i < static_cast<int>(pids_vec.size()); ++i) {
        std::string pid = pids_vec[i];
        std::cout << pid << std::endl;
        std::string imname = imgdir + pid + ".jpg";
        cv::Mat pano = cv::imread(imname);
        //cv::Mat roadimg;
        //road_cvtor.create_road_topview(pano, &roadimg);
        std::string txtname = cv::format("%s%d.txt", txtdir.c_str(), i);
        std::vector<Mark> marks;
        std::ifstream txtfile(txtname.c_str());
        if (txtfile.fail()) {
            continue;
        }
        while (!txtfile.eof()) {
            Mark mark;
            txtfile >> mark.rect.x;
            txtfile >> mark.rect.y;
            txtfile >> mark.rect.width;
            txtfile >> mark.rect.height;
            txtfile >> mark.type_code;
            if (mark.rect.width != 0 && mark.rect.height != 0) {
                marks.push_back(mark);
            }
        }
        txtfile.close();
        for (int m = 0; m < static_cast<int>(marks.size()); ++m) {
            cv::Rect rect = marks[m].rect;
            int size = static_cast<int>(rect.width / 1.5f);
            rect.x += static_cast<int>(0.5f * (rect.width - size));
            rect.y += static_cast<int>(0.5f * (rect.height - size));
            rect.width = size;
            rect.height = size;
            rect.y += static_cast<int>(0.33f * rect.height);
            rect.height = static_cast<int>(0.33f * rect.height);
            std::vector<cv::Point> ppts;
            std::vector<cv::Point> rpts;
            rpts.push_back(rect.tl());
            rpts.push_back(rect.br());
            rpts.push_back(cv::Point(rect.x, rect.y + rect.height));
            rpts.push_back(cv::Point(rect.x + rect.width, rect.y));
            road_cvtor.pts_road_to_pano(PANO_WIDTH, PANO_HEIGHT, rpts, &ppts);
            int minx = INT_MAX;
            int miny = INT_MAX;
            int maxx = INT_MIN;
            int maxy = INT_MIN;
            for (int p = 0; p < static_cast<int>(ppts.size()); ++p) {
                if (ppts[p].x > maxx) {
                    maxx = ppts[p].x;
                }
                if (ppts[p].y > maxy) {
                    maxy = ppts[p].y;
                }
                if (ppts[p].x < minx) {
                    minx = ppts[p].x;
                }
                if (ppts[p].y < miny) {
                    miny = ppts[p].y;
                }
            }
            cv::Rect drrect(minx, miny, maxx - minx + 1, maxy - miny + 1);
            if (drrect.width > 7000) {
                if (drrect.x >(PANO_WIDTH - drrect.x - drrect.width)) {
                    //use left
                    drrect.width = drrect.x;
                    drrect.x = 0;
                }
                else {
                    int w = PANO_WIDTH - drrect.x - drrect.width - 1;
                    drrect.x = drrect.x + drrect.width;
                    drrect.width = w;
                }
            }

            size = (std::max)(drrect.width, drrect.height);
            drrect.x -= static_cast<int>(0.5f * (size - drrect.width));
            drrect.y -= static_cast<int>(0.5f * (size - drrect.height));
            drrect.width = static_cast<int>(size);
            drrect.height = static_cast<int>(size);
            drrect.x -= static_cast<int>(0.25 * drrect.width);
            drrect.y -= static_cast<int>(0.25 * drrect.height);
            drrect.width = static_cast<int>(1.5 * drrect.width);
            drrect.height = static_cast<int>(1.5 * drrect.height);
            if (drrect.x < 0) {
                drrect.width += drrect.x;
                drrect.x = 0;
            }
            if (drrect.y < 0) {
                drrect.height += drrect.y;
                drrect.y = 0;
            }
            if (drrect.x + drrect.width >= pano.cols) {
                drrect.width = pano.cols - drrect.x - 1;
            }
            if (drrect.y + drrect.height >= pano.rows) {
                drrect.height = pano.rows - drrect.y - 1;
            }
            if (drrect.width > 5000 || drrect.height > 5000) {
                continue;
            }
            
            try {
                cv::Scalar color = cv::Scalar(0, 255, 0);
                /*cv::Mat mat_roi = pano(drrect);
                cv::Mat mat_diff = mat_roi.clone();
                double ave = (color.val[0] + color.val[1] + color.val[2]) / 3;
                cv::Scalar diff = cv::Scalar(color.val[0] - ave, color.val[1] - ave, color.val[2] - ave);
                mat_diff.setTo(0.8 * diff);
                cv::add(mat_roi, mat_diff, mat_roi);*/
                cv::rectangle(pano, drrect, color, 2);
                std::string text = cv::format("%d", marks[m].type_code);
                cv::putText(pano, text, cv::Point(drrect.x, drrect.y - 2), 1, 1.0, color, 2);
            }
            catch (std::exception &ex) {
                std::cout << "Error pid: " << pid << std::endl;
                std::cout << ex.what() << std::endl;
                continue;
            }
        }
        std::string savename = outdir + pid + ".jpg";
        cv::imwrite(savename, pano);
        //std::string roadname = outdir + pid + "_r.jpg";
        //cv::imwrite(roadname, roadimg);
    }
    return 0;
}
