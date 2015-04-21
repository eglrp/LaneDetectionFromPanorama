// Copyright 2015 Baidu Inc. All Rights Reserved.
// Author:  Hou Wenbo (houwenbo@baidu.com)
//
// Test decode the batch binary file.
//
#include <vector>
#include "../CropCNNSamples/batch_file.h"
const int SAMPLE_WIDTH = 48;
const int SAMPLE_HEIGHT = 48;
int main() {
    stcv::BatchFile batch_file(10, 100, SAMPLE_WIDTH, SAMPLE_HEIGHT, 3, "", "test");
    std::vector<stcv::PatchFile> patchs;
    batch_file.decode_batch("test_0.bin", &patchs);
    return 0;
}
