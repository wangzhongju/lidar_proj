#include "utils.h"

Eigen::Matrix4d getTransform(double x, double y, double z, double roll, double pitch, double yaw) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform(0, 3) = x;
    transform(1, 3) = y;
    transform(2, 3) = z;
    // 计算绕X轴的旋转矩阵
    Eigen::Matrix3d R_x;
    R_x << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll);

    // 计算绕Y轴的旋转矩阵
    Eigen::Matrix3d R_y;
    R_y << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);

    // 计算绕Z轴的旋转矩阵
    Eigen::Matrix3d R_z;
    R_z << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;

    // 总的旋转矩阵为绕Z、Y、X轴的旋转矩阵的乘积
    Eigen::Matrix3d R = R_z * R_y * R_x;

    // 将旋转矩阵填入 4x4 变换矩阵
    transform.block<3, 3>(0, 0) = R;

    // 设置平移部分
    transform(0, 3) = x;
    transform(1, 3) = y;
    transform(2, 3) = z;

    return transform;
}
std::vector<boost::filesystem::path> loadBagFiles(const std::string& folderPath) {
    std::vector<boost::filesystem::path> bagFiles;

    // 遍历文件夹中的所有文件
    for (const auto& entry : boost::filesystem::directory_iterator(folderPath)) {
        if (boost::filesystem::exists(entry) && boost::filesystem::is_regular_file(entry)) {
            // 检查文件扩展名是否为 .bag
            if (entry.path().extension() == ".bag") {
                bagFiles.push_back(entry.path());
            }
        }
    }

    return bagFiles;
}
std::vector<std::string> extractBetweenSlashes(const std::string& input) {
    std::vector<std::string> result;
    size_t start = 0;
    size_t end = 0;

    // 遍历整个字符串，查找斜杠之间的内容
    while ((start = input.find('/', start)) != std::string::npos) {
        start++;  // 移动到斜杠后面的位置
        end = input.find('/', start);

        // 如果找到下一个斜杠，提取之间的字符串
        if (end != std::string::npos) {
            result.push_back(input.substr(start, end - start));
            start = end;  // 移动到下一个斜杠的位置
        } else {
            break;  // 如果没有找到下一个斜杠，结束循环
        }
    }

    return result;
}
void printProgress(size_t current, size_t total) {
    int progress = static_cast<int>((current + 1) * 100 / total);                                                                      // 计算进度百分比
    std::cout << "Progress: [" << std::string(progress / 2, '#') << std::string(50 - progress / 2, ' ') << "] " << progress << "%\r";  // \r 用于在同一行更新输出
    std::cout.flush();                                                                                                                 // 刷新输出流
}