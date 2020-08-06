#include <iostream>
#include <iomanip>
#include <ctime>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pangolin/pangolin.h>

// 全局变量，用于保存生存的位姿
std::vector<std::vector<float> > pose_in;
std::vector<std::string> v_rgb; std::vector<std::string> v_depth;

// 给定初始速度和初始状态（运行的时候）
int slam_speed = 1, command_go = 1;

struct RotationMatrix{
    Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
};

struct TranslationVector{
    Eigen::Vector3d trans = Eigen::Vector3d(0, 0, 0);
};

int get_pose(std::string path_to_data, std::vector<std::vector<float> > &pose, int index, std::vector<std::string> &v_rgb, std::vector<std::string> &v_depth){
    // 先定义文件夹路径
    std::string path_to_pose = path_to_data + "/groundtruth.txt";
    std::string path_to_imagePath = path_to_data + "/associate.txt";

    std::ifstream check_pose_file(path_to_pose);
    if(!check_pose_file){
        std::cerr << "Can't find the groundtruth.txt" << std::endl;
        return 1;
    }

    std::ifstream check_image_file(path_to_imagePath);
    if(!check_image_file){
        std::cerr << "Can't find the association.txt" << std::endl;
        return 1;
    }
    std::string rgb_path, depth_path;
    double rgb_time, depth_time;
    for(int i = 0; i < index; i++) {
        double temp_pose;
        float temp[8];
        std::vector<float> pose_temp;
        check_pose_file >> temp_pose >> temp[1] >> temp[2] >> temp[3] >> temp[4] >> temp[5] >> temp[6] >> temp[7];
        pose_temp.push_back(temp[1]);
        pose_temp.push_back(temp[2]);
        pose_temp.push_back(temp[3]);
        pose_temp.push_back(temp[7]);
        pose_temp.push_back(temp[4]);
        pose_temp.push_back(temp[5]);
        pose_temp.push_back(temp[6]);
        pose.push_back(pose_temp);
        pose_temp.clear();
        check_image_file >> rgb_time >> rgb_path >> depth_time >> depth_path;
        v_rgb.push_back(path_to_data + "/" + rgb_path);
        v_depth.push_back(path_to_data + "/" + depth_path);
    }
    return 1;
}

int main()
{
    // 读取文件地址
    std::string data_addr = "/Users/robotics_qi/Data/TUM_dataset/rgbd_dataset_freiburg1_room";
    // 定义暂时读取位姿信息存储的vector， 读取总帧数和读取间隔
    std::vector<std::vector<float>> pose; int index = 100;
    // 获取pose.
    get_pose(data_addr, pose, index, v_rgb, v_depth);
    std::cout << v_rgb.size() << std::endl;

    const float w = 0.2;
    const float h = w * 0.75;
    const float z = w * 0.6;

    // 生成一个GUI界面，自定义大小
    pangolin::CreateWindowAndBind("Main", 3180, 2160);
    // 进行深度测试
    glEnable(GL_DEPTH_TEST);

    //放置一个相机
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(3180, 2160, 420, 420, 900, 640, 0.2, 2000),
            pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY));

    // 创建视口
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, -640.f / 480.0f).SetHandler(&handler);

    // 设计显示面板
    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, 0.3);
    // 第一个参数为按钮名称，第二个为默认状态，第三个为是否有选择框
    pangolin::Var<bool> menu("menu.lines", true, true);
    pangolin::Var<bool> goon("menu.go_on", true, true);
    // 设计文本输出面板
//    pangolin::Var<RotationMatrix> rotation_matrix("menu.r", RotationMatrix);
//    pangolin::Var<TranslationVector> translation_vector("menu.t", TranslationVector());

    // 定义图片面元
    pangolin::View& rgb_image = pangolin::Display("rgb").SetBounds(0, 0.3, 0.3, 0.65, 1024.0f / 768.0f).SetLock(pangolin::LockLeft, pangolin::LockBottom);
    pangolin::View& depth_image = pangolin::Display("depth").SetBounds(0, 0.3, 0.65, 1, 1024.0f/768.0f).SetLock(pangolin::LockLeft, pangolin::LockBottom);

    // 初始化
    pangolin::GlTexture imageTexture(640, 480, GL_RGB, false,0, GL_BGR, GL_UNSIGNED_BYTE);

    while(!pangolin::ShouldQuit()){
        clock_t time_start = clock();
        // 清除颜色缓冲和深度缓冲
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        // 先换成白色颜色
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        // 图像读取，显示

        cv::Mat rgb;
        for(int k = v_rgb.size(); k >= 1; --k) {
//            rgb = cv::imread(v_rgb.at(k - 1));
            rgb = cv::imread("/Users/robotics_qi/Data/TUM_dataset/rgbd_dataset_freiburg1_room/rgb/1305031910.765238.png");
            imageTexture.Upload(rgb.data, GL_BGR, GL_UNSIGNED_BYTE);
            rgb_image.Activate();
            glColor3f(1.0, 1.0, 1.0);
            imageTexture.RenderToViewportFlipY();

        }
        pangolin::FinishFrame();
    }

    return 0;
}