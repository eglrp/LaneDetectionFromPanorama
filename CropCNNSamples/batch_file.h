#ifndef STCV_BATCH_FILE_H
#define STCV_BATCH_FILE_H
#include <string>
#include <iostream>
#include <fstream>
#include "opencv2/opencv.hpp"
namespace stcv {
#ifndef DISALLOW_COPY_AND_ASSIGN
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
TypeName(const TypeName&); \
TypeName& operator=(const TypeName&)
#endif
const int NAME_MAX_LENGTH = 256;
struct PatchFile
{
    PatchFile() {
        data = NULL;
        type = 0;
        x = 0;
        y = 0;
        w = 0;
        h = 0;
    }
    unsigned char* data;
    char name[NAME_MAX_LENGTH];
    int type;
    int x;
    int y;
    int w;
    int h;
};
struct Batch
{
    Batch() {
        patch_num = 0;
        p_patchs = NULL;
    }
    int patch_num;
    PatchFile* p_patchs;
};
class BatchFile
{
public:
    enum RETURN_TYPE {
        OK = 0,
        ERROR_MEM_ALLOCATE_FAIL = -1,
        ERROR_OPENFILE = -2
    };

    /// \fn BatchFile::BatchFile(const int& file_num, const int& max_patchs_in_memory,
    ///      const int& patch_w, const int& patch_h, const int& channels,
    ///       const std::string& file_path, const std::string& prefix);
    ///
    /// \brief  Constructor.
    ///
    /// \author Houwenbo
    /// \date   2015/4/3
    ///
    /// \param  file_num                存储的大文件数量.
    /// \param  max_patchs_in_memory    缓存在内存中的最大文件数量，达到最大值就写入文件.
    /// \param  patch_w                 样本数据中图像的宽.
    /// \param  patch_h                 样本数据中图像的高.
    /// \param  channels                样本数据中图像的通道数.
    /// \param  file_path               大文件的待存储路径.
    /// \param  prefix                  生成大文件的前缀名, prefix_0.
    BatchFile(const int& file_num, const int& max_patchs_in_memory, const int& patch_w,
        const int& patch_h, const int& channels, const std::string& file_path,
        const std::string& prefix);
    ~BatchFile();

    /// \fn int BatchFile::add_patch(const PatchFile& patch_file);
    ///
    /// \brief  添加一个样本文件.
    ///
    /// \author Houwenbo
    /// \date   2015/4/3
    ///
    /// \param  patch_file  The patch file.
    ///
    /// \return 成功0，失败非0.
    int add_patch(const PatchFile& patch_file);
    int add_patch(const cv::Mat& img, const int& type, const cv::Rect& rect,
        const std::string& name);

    /// \fn int BatchFile::write_remain_data();
    ///
    /// \brief  将内存中存在的文件写入到文件中.
    ///
    /// \author Houwenbo
    /// \date   2015/4/3
    ///
    /// \return An int.
    int write_remain_data();

    /// \fn int BatchFile::decode_batch(const std::string& batch_file, std::vector<PatchFile>* p_patchs);
    ///
    /// \brief  解码生成的二进制大文件.
    ///
    /// \author Houwenbo
    /// \date   2015/4/3
    ///
    /// \param  batch_file          The batch file.
    /// \param [in,out] p_patchs    If non-null, the patchs.
    ///
    /// \return 成功0，失败非0.
    int decode_batch(const std::string& batch_file, std::vector<PatchFile>* p_patchs);
private:
    int write_to_file(const int& index);
    long _total_cnt;
    int _file_num;
    int _max_patchs_in_memory;
    int _patch_width;
    int _patch_height;
    int _patch_channels;
    std::string _file_path;
    std::string _prefix;   
    Batch* _p_batch_file;

    DISALLOW_COPY_AND_ASSIGN(BatchFile);
};
}
#endif
