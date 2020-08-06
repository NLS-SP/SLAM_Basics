//
// Created by Robotics_qi on 2020/7/5.
//

#include <iostream>
#include <iomanip>
#include <ctime>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pangolin/pangolin.h>
#include <thread>
#include <unistd.h>

// 全局变量保存生成的位姿
std::vector<std::vector<float> > pose_fin;

// groundTruth读取文件信息
int getPose(std::string path_to_dataset);

int main()
{
    std::string data_path = "/Users/robotics_qi/Data/TUM_dataset/rgbd_dataset_freiburg1_room";
    // 定义读取位姿信息存储的vector, 读取总帧数和读数间隔
    // 读取线程
    std::thread read_thread;
    read_thread = std::thread(getPose, data_path);
    read_thread.detach();

    const float w=0.2;
    const float h=w*0.75;
    const float z=w*0.6;
    //生成一个gui界面，定义大小
    pangolin::CreateWindowAndBind("Main",640,480);
    //进行深度测试，保证某视角下像素只有一种颜色，不混杂
    glEnable(GL_DEPTH_TEST);

    //放置一个相机
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
            pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
    );

    //创建视角窗口
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);

    while( !pangolin::ShouldQuit() )
    {
        //清除颜色缓冲和深度缓冲
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        //背景先弄成白色的吧，我觉得白色比较好看
        glClearColor(1.0f,1.0f,1.0f,1.0f);

        //使用变换矩阵画图
        for(int i=0;i<pose_fin.size();i++)
        {
            //使用位置变换矩阵
            glPushMatrix();
            //变换如该矩阵，注意这个变换矩阵是转置的
            std::vector<GLfloat> Twc ={ pose_fin[i][0],pose_fin[i][1],pose_fin[i][2],0,
                                        pose_fin[i][3],pose_fin[i][4],pose_fin[i][5],0,
                                        pose_fin[i][6],pose_fin[i][7],pose_fin[i][8],0,
                                        pose_fin[i][9],pose_fin[i][10],pose_fin[i][11],1 };

            //变换
            glMultMatrixf(Twc.data());
            //每次变换后绘制相机
            glLineWidth(2);
            glBegin(GL_LINES);
            glColor3f(0.0f,0.0f,1.0f);
            glVertex3f(0,0,0);		glVertex3f(w,h,z);
            glVertex3f(0,0,0);		glVertex3f(w,-h,z);
            glVertex3f(0,0,0);		glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);		glVertex3f(-w,h,z);
            glVertex3f(w,h,z);		glVertex3f(w,-h,z);
            glVertex3f(-w,h,z);		glVertex3f(-w,-h,z);
            glVertex3f(-w,h,z);		glVertex3f(w,h,z);
            glVertex3f(-w,-h,z);		glVertex3f(w,-h,z);

            glEnd();
            glPopMatrix();
        }

        //绘制连接的绿色线
        if(pose_fin.size() != 0) {
            glLineWidth(2);
            glBegin(GL_LINES);
            glColor3f(0.0f, 1.f, 0.f);
            for (int i = 0; i < pose_fin.size() - 1; i++) {
                glVertex3f(pose_fin[i][9], pose_fin[i][10], pose_fin[i][11]);
                glVertex3f(pose_fin[i + 1][9], pose_fin[i + 1][10], pose_fin[i + 1][11]);
            }
            glEnd();
        }

        //交换帧和并推进事件
        pangolin::FinishFrame();
    }

}

int getPose(std::string path_to_dataset){
    std::string path_to_pose = path_to_dataset + "/groundtruth.txt";
    std::cout << "Now, the data is in: " << path_to_dataset << std::endl;

    std::vector<std::vector<float>> pose; int index = 4511; int interval = 160;
    std::ifstream check_path(path_to_pose);
    if(!check_path){
        std::cerr << "I cannot find the txt!" << std::endl;
        return 1;
    }

    // 循环取数值给pose.
    for(int i = 0; i < index; ++i){
        float temp[8];
        std::vector<float> pose_temp;
        while(interval--)
            check_path >> temp[0] >> temp[1] >> temp[2] >> temp[3] >> temp[4] >> temp[5] >> temp[6] >> temp[7];
        std::cout << temp[0] << temp[1] << temp[2] << temp[3] << temp[4] << temp[5] << temp[6] << std::endl;
        Eigen::Quaterniond quaternion(temp[7], temp[4], temp[5], temp[6]);
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix = quaternion.matrix();
        pose_temp.push_back(rotation_matrix(0,0));	pose_temp.push_back(rotation_matrix(1,0));	pose_temp.push_back(rotation_matrix(2,0));
        pose_temp.push_back(rotation_matrix(0,1));	pose_temp.push_back(rotation_matrix(1,1));	pose_temp.push_back(rotation_matrix(2,1));
        pose_temp.push_back(rotation_matrix(0,2));	pose_temp.push_back(rotation_matrix(1,2));	pose_temp.push_back(rotation_matrix(2,2));
        pose_temp.push_back(temp[1]); pose_temp.push_back(temp[2]);
        // 将pose_temp放入全局变量pose中用于构图
        pose_fin.push_back(pose_temp);
        std::cout << "Now the size is: " << pose_fin.size() << std::endl;
        pose_temp.clear();
        usleep(100000);
    }
    pose.clear();
    return 1;
}