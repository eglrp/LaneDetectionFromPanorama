#include "batch_file.h"

namespace stcv {
BatchFile::BatchFile(const int& file_num, const int& max_patchs_in_memory, const int& patch_w, 
    const int& patch_h, const int& channels, const std::string& file_path,
    const std::string& prefix) : _file_num(file_num),
    _max_patchs_in_memory(max_patchs_in_memory), _patch_width(patch_w), _patch_height(patch_h),
    _patch_channels(channels), _file_path(file_path), _prefix(prefix)
{
    _p_batch_file = new Batch[file_num];
    for (int i = 0; i < file_num; ++i) {
        _p_batch_file[i].patch_num = 0;
        _p_batch_file[i].p_patchs = new PatchFile[_max_patchs_in_memory];
        for (int p = 0; p < _max_patchs_in_memory; ++p) {
            _p_batch_file[i].p_patchs[p].data
                = new uchar[_patch_width*_patch_height*_patch_channels];
        }
    }

    _total_cnt = 0;
}

BatchFile::~BatchFile()
{
    for (int i = 0; i < _file_num; ++i) {
        for (int p = 0; p < _max_patchs_in_memory; ++p) {
            if (_p_batch_file[i].p_patchs[p].data != NULL) {
                delete[] _p_batch_file[i].p_patchs[p].data;
                _p_batch_file[i].p_patchs[p].data = NULL;
            }
        }
        if (_p_batch_file[i].p_patchs != NULL) {
            delete[] _p_batch_file[i].p_patchs;
            _p_batch_file[i].p_patchs = NULL;
        }
    }
    if (_p_batch_file != NULL) {
        delete[] _p_batch_file;
        _p_batch_file = NULL;
    }
}

int BatchFile::write_remain_data() {
    for (int i = 0; i < _file_num; ++i) {
        if (_p_batch_file[i].patch_num >= _max_patchs_in_memory)
        {
            int ret = write_to_file(i);
            if (ret != 0) {
                continue;
            }
        }
    }
    return 0;
}

int BatchFile::add_patch(const PatchFile& patch_file) {
    if (_p_batch_file == NULL) {
        return ERROR_MEM_ALLOCATE_FAIL;
    }
    for (int i = 0; i < _file_num; ++i) {
        if (_p_batch_file[i].p_patchs == NULL) {
            return ERROR_MEM_ALLOCATE_FAIL;
        }
    }

    int index = _total_cnt % _file_num;
    _p_batch_file[index].p_patchs[_p_batch_file[index].patch_num].h = patch_file.h;
    _p_batch_file[index].p_patchs[_p_batch_file[index].patch_num].w = patch_file.w;
    _p_batch_file[index].p_patchs[_p_batch_file[index].patch_num].x = patch_file.x;
    _p_batch_file[index].p_patchs[_p_batch_file[index].patch_num].y = patch_file.y;
    _p_batch_file[index].p_patchs[_p_batch_file[index].patch_num].type = patch_file.type;
    memcpy(_p_batch_file[index].p_patchs[_p_batch_file[index].patch_num].name, patch_file.name,
        NAME_MAX_LENGTH);
    memcpy(_p_batch_file[index].p_patchs[_p_batch_file[index].patch_num].data, patch_file.data,
        _patch_width * _patch_height * _patch_channels);
    _p_batch_file[index].patch_num++;
    if (_p_batch_file[index].patch_num >= _max_patchs_in_memory)
    {
        int ret = write_to_file(index);
        if (ret != 0) {
            return ret;
        }
    }
    ++_total_cnt;
    return 0;
}

int BatchFile::add_patch(const cv::Mat& img, const int& type, const cv::Rect& rect,
    const std::string& name) {
    if (img.rows != _patch_height || img.cols != _patch_width) {
        return -1;
    }
    PatchFile patch_file;
    patch_file.data = new unsigned char[_patch_height*_patch_height*_patch_channels];
    for (int y = 0; y < _patch_height; ++y) {
        const uchar* ptr = img.ptr<uchar>(y);
        for (int x = 0; x < _patch_width; ++x) {
            for (int c = 0; c < _patch_channels; ++c) {
                patch_file.data[y*_patch_width + x + c*_patch_height*_patch_width] =
                    ptr[x*_patch_channels + c];
            }
        }
    }
#if 0
    cv::imwrite("or_img.jpg", img);
    cv::Mat debug_img = cv::Mat(img.rows*3, img.cols, CV_8UC1, patch_file.data);
    cv::imwrite("debug_img.jpg", debug_img);
#endif
    if (static_cast<int>(name.length()) >= NAME_MAX_LENGTH) {
        memcpy(patch_file.name, name.c_str(), NAME_MAX_LENGTH);
        patch_file.name[NAME_MAX_LENGTH - 1] = '\0';
    }
    else {
        memcpy(patch_file.name, name.c_str(), name.length());
        patch_file.name[name.length()] = '\0';
    }
    patch_file.type = type;
    patch_file.x = rect.x;
    patch_file.y = rect.y;
    patch_file.w = rect.width;
    patch_file.h = rect.height;
    int ret = add_patch(patch_file);
    delete[] patch_file.data;
    patch_file.data = NULL;
    return ret;
}

int BatchFile::write_to_file(const int& index) {
    std::string file_name = cv::format("%s%s_%d.bin", _file_path.c_str(), _prefix.c_str(), index);
    std::ofstream out_file(file_name.c_str(), std::ios::out | std::ios::binary | std::ios::app);
    if (out_file.fail()) {
        return ERROR_OPENFILE;
    }
    for (int i = 0; i < _p_batch_file[index].patch_num; ++i) {
        PatchFile patch_file = _p_batch_file[index].p_patchs[i];
#if 0
        cv::Mat debug_img 
            = cv::Mat(_patch_height * _patch_channels, _patch_width, CV_8UC1, patch_file.data);
#endif
        out_file.write(reinterpret_cast<const char*>(patch_file.data),
            _patch_height*_patch_width*_patch_channels*sizeof(uchar));
        out_file.write(reinterpret_cast<const char*>(patch_file.name), sizeof(patch_file.name));
        out_file.write(reinterpret_cast<const char*>(&patch_file.type), sizeof(patch_file.type));
        out_file.write(reinterpret_cast<const char*>(&patch_file.x), sizeof(patch_file.x));
        out_file.write(reinterpret_cast<const char*>(&patch_file.y), sizeof(patch_file.y));
        out_file.write(reinterpret_cast<const char*>(&patch_file.w), sizeof(patch_file.w));
        out_file.write(reinterpret_cast<const char*>(&patch_file.h), sizeof(patch_file.h));
    }
    out_file.close();
    _p_batch_file[index].patch_num = 0;
    return 0;
}

int BatchFile::decode_batch(const std::string& batch_file, std::vector<PatchFile>* p_patchs) {
    std::ifstream read_file(batch_file.c_str(), std::ios::binary);
    if (read_file.fail()) {
        return ERROR_OPENFILE;
    }
    p_patchs->clear();
    while (!read_file.eof()) {
        PatchFile patch_file;
        patch_file.data = new uchar[_patch_width*_patch_height*_patch_channels];
        read_file.read(reinterpret_cast<char*>(patch_file.data),
            _patch_width*_patch_height*_patch_channels*sizeof(uchar));
        read_file.read(reinterpret_cast<char*>(patch_file.name), sizeof(patch_file.name));
        read_file.read(reinterpret_cast<char*>(&patch_file.type), sizeof(patch_file.type));
        read_file.read(reinterpret_cast<char*>(&patch_file.x), sizeof(patch_file.x));
        read_file.read(reinterpret_cast<char*>(&patch_file.y), sizeof(patch_file.y));
        read_file.read(reinterpret_cast<char*>(&patch_file.w), sizeof(patch_file.w));
        read_file.read(reinterpret_cast<char*>(&patch_file.h), sizeof(patch_file.h));
        p_patchs->push_back(patch_file);
#if 1
        cv::Mat debug_img
            = cv::Mat(_patch_height * _patch_channels, _patch_width, CV_8UC1, patch_file.data);
        cv::Mat merge_img = cv::Mat(_patch_height, _patch_width, CV_8UC3);
        for (int y = 0; y < _patch_height; ++y) {
            const uchar* ptr_b = debug_img.ptr<uchar>(y);
            const uchar* ptr_g = debug_img.ptr<uchar>(y + _patch_height);
            const uchar* ptr_r = debug_img.ptr<uchar>(y + _patch_height * 2);
            for (int x = 0; x < _patch_width; ++x) {
                merge_img.data[y*merge_img.step + x*merge_img.channels() + 0] = ptr_b[x];
                merge_img.data[y*merge_img.step + x*merge_img.channels() + 1] = ptr_g[x];
                merge_img.data[y*merge_img.step + x*merge_img.channels() + 2] = ptr_r[x];
            }
        }
        cv::imwrite(patch_file.name, merge_img);
#endif
    }
    return 0;
}
}

