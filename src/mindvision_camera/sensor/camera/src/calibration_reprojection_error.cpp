#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <iomanip>

class CalibrationReprojectionAnalyzer {
private:
    cv::Size board_size_;
    float square_size_;
    std::vector<std::vector<cv::Point3f>> object_points_;
    std::vector<std::vector<cv::Point2f>> image_points_;
    std::vector<cv::Mat> images_;
    std::vector<std::string> image_paths_;
    
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    std::vector<cv::Mat> rvecs_, tvecs_;
    
public:
    CalibrationReprojectionAnalyzer(cv::Size board_size, float square_size) 
        : board_size_(board_size), square_size_(square_size) {
        
        // 生成标定板的3D点
        std::vector<cv::Point3f> obj_p;
        for(int i = 0; i < board_size_.height; i++) {
            for(int j = 0; j < board_size_.width; j++) {
                obj_p.push_back(cv::Point3f(j * square_size_, i * square_size_, 0));
            }
        }
        object_points_.push_back(obj_p);
    }
    
    bool loadImages(const std::string& image_directory) {
        std::cout << "正在加载图像目录: " << image_directory << std::endl;
        
        if (!std::filesystem::exists(image_directory)) {
            std::cerr << "错误: 图像目录不存在!" << std::endl;
            return false;
        }
        
        // 遍历目录中的所有图像文件
        for (const auto& entry : std::filesystem::directory_iterator(image_directory)) {
            std::string path = entry.path().string();
            std::string ext = entry.path().extension().string();
            
            // 检查是否为图像文件
            if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp") {
                cv::Mat image = cv::imread(path);
                if (!image.empty()) {
                    images_.push_back(image);
                    image_paths_.push_back(path);
                    std::cout << "加载图像: " << entry.path().filename() << std::endl;
                }
            }
        }
        
        std::cout << "总共加载了 " << images_.size() << " 张图像" << std::endl;
        return !images_.empty();
    }
    
    bool findCorners() {
        std::cout << "正在检测角点..." << std::endl;
        
        image_points_.clear();
        object_points_.clear();
        
        std::vector<cv::Point3f> obj_p;
        for(int i = 0; i < board_size_.height; i++) {
            for(int j = 0; j < board_size_.width; j++) {
                obj_p.push_back(cv::Point3f(j * square_size_, i * square_size_, 0));
            }
        }
        
        int successful_detections = 0;
        
        for (size_t i = 0; i < images_.size(); i++) {
            cv::Mat gray;
            cv::cvtColor(images_[i], gray, cv::COLOR_BGR2GRAY);
            
            std::vector<cv::Point2f> corners;
            bool found = cv::findChessboardCorners(gray, board_size_, corners,
                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
            
            if (found) {
                // 亚像素精度优化
                cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
                
                image_points_.push_back(corners);
                object_points_.push_back(obj_p);
                successful_detections++;
                
                std::cout << "✓ " << image_paths_[i] << " - 角点检测成功" << std::endl;
            } else {
                std::cout << "✗ " << image_paths_[i] << " - 角点检测失败" << std::endl;
            }
        }
        
        std::cout << "成功检测角点的图像: " << successful_detections << "/" << images_.size() << std::endl;
        return successful_detections > 0;
    }
    
    bool calibrateCamera() {
        if (image_points_.empty()) {
            std::cerr << "错误: 没有可用的角点数据!" << std::endl;
            return false;
        }
        
        std::cout << "正在进行相机标定..." << std::endl;
        
        cv::Size image_size = images_[0].size();
        
        double rms = cv::calibrateCamera(object_points_, image_points_, image_size,
            camera_matrix_, dist_coeffs_, rvecs_, tvecs_,
            cv::CALIB_FIX_PRINCIPAL_POINT);
        
        std::cout << "标定完成!" << std::endl;
        std::cout << "RMS重投影误差: " << rms << " 像素" << std::endl;
        std::cout << "\n相机内参矩阵:" << std::endl;
        std::cout << camera_matrix_ << std::endl;
        std::cout << "\n畸变系数:" << std::endl;
        std::cout << dist_coeffs_ << std::endl;
        
        return true;
    }
    
    void analyzeReprojectionError() {
        if (image_points_.empty()) {
            std::cerr << "错误: 没有标定数据!" << std::endl;
            return;
        }
        
        std::cout << "\n=== 重投影误差分析 ===" << std::endl;
        
        std::vector<double> per_image_errors;
        double total_error = 0;
        double max_error = 0;
        double min_error = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < object_points_.size(); i++) {
            std::vector<cv::Point2f> projected_points;
            cv::projectPoints(object_points_[i], rvecs_[i], tvecs_[i],
                camera_matrix_, dist_coeffs_, projected_points);
            
            double err = cv::norm(image_points_[i], projected_points, cv::NORM_L2);
            double mean_err = err / object_points_[i].size();
            
            per_image_errors.push_back(mean_err);
            total_error += err * err;
            
            max_error = std::max(max_error, mean_err);
            min_error = std::min(min_error, mean_err);
            
            std::cout << "图像 " << std::setw(2) << i+1 << ": " 
                     << std::fixed << std::setprecision(3) << mean_err << " 像素" << std::endl;
        }
        
        double mean_error = std::sqrt(total_error / (object_points_.size() * object_points_[0].size()));
        
        std::cout << "\n统计信息:" << std::endl;
        std::cout << "平均重投影误差: " << std::fixed << std::setprecision(3) << mean_error << " 像素" << std::endl;
        std::cout << "最大误差: " << std::fixed << std::setprecision(3) << max_error << " 像素" << std::endl;
        std::cout << "最小误差: " << std::fixed << std::setprecision(3) << min_error << " 像素" << std::endl;
        
        // 误差分布分析
        std::cout << "\n误差分布:" << std::endl;
        int good_count = 0, acceptable_count = 0, poor_count = 0;
        
        for (double err : per_image_errors) {
            if (err < 0.5) good_count++;
            else if (err < 1.0) acceptable_count++;
            else poor_count++;
        }
        
        std::cout << "优秀 (<0.5px): " << good_count << " 张" << std::endl;
        std::cout << "可接受 (0.5-1.0px): " << acceptable_count << " 张" << std::endl;
        std::cout << "较差 (>1.0px): " << poor_count << " 张" << std::endl;
    }
    
    void visualizeReprojectionError() {
        std::cout << "\n正在生成可视化结果..." << std::endl;
        
        for (size_t i = 0; i < std::min(object_points_.size(), size_t(10)); i++) {
            // 投影3D点到图像平面
            std::vector<cv::Point2f> projected_points;
            cv::projectPoints(object_points_[i], rvecs_[i], tvecs_[i],
                camera_matrix_, dist_coeffs_, projected_points);
            
            // 创建可视化图像
            cv::Mat vis_image = images_[i].clone();
            
            // 绘制检测到的角点（绿色）
            for (const auto& point : image_points_[i]) {
                cv::circle(vis_image, point, 5, cv::Scalar(0, 255, 0), 2);
            }
            
            // 绘制重投影点（红色）
            for (const auto& point : projected_points) {
                cv::circle(vis_image, point, 3, cv::Scalar(0, 0, 255), 2);
            }
            
            // 绘制误差线
            for (size_t j = 0; j < image_points_[i].size(); j++) {
                cv::line(vis_image, image_points_[i][j], projected_points[j], 
                        cv::Scalar(255, 0, 255), 1);
            }
            
            // 计算并显示该图像的误差
            double err = cv::norm(image_points_[i], projected_points, cv::NORM_L2);
            double mean_err = err / object_points_[i].size();
            
            std::string error_text = "Error: " + std::to_string(mean_err).substr(0, 5) + "px";
            cv::putText(vis_image, error_text, cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
            
            std::string legend1 = "Green: Detected | Red: Reprojected | Purple: Error";
            cv::putText(vis_image, legend1, cv::Point(10, vis_image.rows - 40), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
            
            std::string window_name = "Reprojection Error - Image " + std::to_string(i+1);
            cv::imshow(window_name, vis_image);
            
            std::cout << "显示图像 " << i+1 << " 的重投影误差可视化 (按任意键继续)" << std::endl;
            cv::waitKey(0);
            cv::destroyWindow(window_name);
        }
    }
    
    void saveResults(const std::string& output_file = "calibration_results.yaml") {
        cv::FileStorage fs(output_file, cv::FileStorage::WRITE);
        if (!fs.isOpened()) {
            std::cerr << "无法保存结果文件!" << std::endl;
            return;
        }
        
        fs << "camera_matrix" << camera_matrix_;
        fs << "distortion_coefficients" << dist_coeffs_;
        fs << "image_width" << images_[0].cols;
        fs << "image_height" << images_[0].rows;
        
        std::cout << "标定结果已保存到: " << output_file << std::endl;
    }
};

int main(int argc, char** argv) {
    std::cout << "=== 相机标定重投影误差分析工具 ===" << std::endl;
    
    // 标定板参数 (可根据实际情况修改)
    cv::Size board_size(10, 7);  // 棋盘格内角点数量 (宽x高)
    float square_size = 120.0;   // 方格边长 (mm)
    
    // 图像目录
    std::string image_dir = "./calibration_images";
    if (argc > 1) {
        image_dir = argv[1];
    }
    
    std::cout << "标定板尺寸: " << board_size.width << "x" << board_size.height << std::endl;
    std::cout << "方格大小: " << square_size << "mm" << std::endl;
    std::cout << "图像目录: " << image_dir << std::endl;
    std::cout << std::endl;
    
    CalibrationReprojectionAnalyzer analyzer(board_size, square_size);
    
    // 加载图像
    if (!analyzer.loadImages(image_dir)) {
        std::cerr << "无法加载图像!" << std::endl;
        return -1;
    }
    
    // 检测角点
    if (!analyzer.findCorners()) {
        std::cerr << "角点检测失败!" << std::endl;
        return -1;
    }
    
    // 相机标定
    if (!analyzer.calibrateCamera()) {
        std::cerr << "相机标定失败!" << std::endl;
        return -1;
    }
    
    // 分析重投影误差
    analyzer.analyzeReprojectionError();
    
    // 可视化重投影误差
    std::cout << "\n是否显示重投影误差可视化? (y/n): ";
    char choice;
    std::cin >> choice;
    if (choice == 'y' || choice == 'Y') {
        analyzer.visualizeReprojectionError();
    }
    
    // 保存结果
    analyzer.saveResults();
    
    std::cout << "\n分析完成!" << std::endl;
    return 0;
}
