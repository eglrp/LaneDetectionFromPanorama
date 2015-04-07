# LaneDetectionFromPanorama
从全景图像中检测车道及地面的导向标志。

将全景图转换成正视图，然后在正视图上截取样本用于后续adaboost和cnn进行训练检测模型和识别模型。
1. 批量将全景图像转换成正视图
create_pano_topview --config config.txt --basedir /home/pic_demo --list list.txt --pixels_per_meter 50 --dst_width_meters 50 --dst_length_meters 160 --output /home/output
--config 配置文件
--basedir 全景图所在文件夹
--list 待生成的文件列表，图像ID列表
--pixels_per_meter 多少个像素表示1米
--dst_width_meters 生成道路对应的实际宽度 单位米
--dst_length_meters 生成道路对应的实际长度 单位米
--output 输出的道路正视图存储的文件路径

config 配置文件格式
imgid \t 相机高度 \t 车头在全景图中的角度 \t pitch \t pitch后
因为实际相机的安装精度不高，提供的pitch精度也不够准确，完全使用一个pitch值无法得到一个理想的道路正视图。
实际采用车前面的道路采用一个pitch值投影，车后面的道路使用另外的pitch值进行投影。

2. 裁剪adaboost检测模型的训练样本数据
crop_samples --imgdir /home/pic_demo --jsondir /home/json --list list.txt --config config.txt --pixels_per_meter 50 --dst_width_meters 50 --dst_length_meters 160 --outdir /home/output
--imgdir 上面工具生成的道路正视图的图像路径
--jsondir 样本标注是在全景图上进行，用json格式存储，每幅全景图对应一个json文件
注意：正样本是在标注样本区域扩大了1.5倍，生成的图像没有进行尺寸归一化
      负样本是在道路的正视图中，将其中有标志的区域填黑生成的
      
3. 训练adaboost检测模型 （adaboost训练代码未添加到github上，待续）
注意：需要对上述生成的正样本进行尺寸归一化处理，即样本图像统一到相同的尺寸

4. 用adaboost算法检测道路正视图，检测结果用于训练CNN识别模型

5. 裁剪CNN识别模型的样本数据
crop_cnn_samples --imgdir /home/pic_demo --jsondir /home/json --txtdir /home/txt --list list.txt --config config.txt --pixels_per_meter 50 --dst_width_meters 50 --dst_length_meters 160 --outdir /home/output
--txtdir adaboost检测生成的结果
数据格式 x \t y \t w \t h \t 类别，每一个检测框对应一行数据
生成的正样本数据是扩展两倍区域的，用于后续的旋转、平移、缩放等变换，增加正样本数量
由于负样本文件较多，输出的负样本结果是将大量图像合并在一起的5个二进制文件

6. 训练CNN识别模型
