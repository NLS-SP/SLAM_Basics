#include <limits>
#include <iostream>
#include <pangolin/pangolin.h>

void setImageData(unsigned char* imageArray, int size){
    for(int i = 0; i < size; ++i)
        imageArray[i] = (unsigned char)(rand() / (RAND_MAX/255.0));
}

int main() {
    // Create OpenGL Window.
    pangolin::CreateWindowAndBind("Main", 640, 480);

    // Enable depth testing.
    glEnable(GL_DEPTH_TEST);


    pangolin::OpenGlRenderState s_cam(
            // 第一个是相机投影模型！
            pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000),
            // 第二个观测点位置、观测目标位置、观测的方向位置
            pangolin::ModelViewLookAt(-1, -1, -1, 0, 0, 0, pangolin::AxisY));

    pangolin::View &d_cam = pangolin::Display("cam").SetBounds(0, 1.0f, 0, 1.0f, -640 / 480.f).SetHandler(
            new pangolin::Handler3D(s_cam));

    pangolin::View &d_image = pangolin::Display("image").SetBounds(2 / 3.0f, 1.0f, 0, 1 / 3.0f, 640.0 / 480).SetLock(
            pangolin::LockLeft, pangolin::LockTop);

    const int width = 64, height = 48;

    unsigned char* imageArray = new unsigned char[3 * width * height];
    pangolin::GlTexture imageTexture(width, height, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);

    while(!pangolin::ShouldQuit()){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);

        glColor3f(1.0, 1.0, 1.0);
        pangolin::glDrawColouredCube();

        // Set some random image data and upload to GPU.
        setImageData(imageArray, 3 * width * height);
        imageTexture.Upload(imageArray, GL_RGB, GL_UNSIGNED_BYTE);

        // display image.
        d_image.Activate();
        glColor3f(1.0, 1.0, 1.0);
        imageTexture.RenderToViewport();

        pangolin::FinishFrame();
    }
    delete[] imageArray;
    return 0;
}