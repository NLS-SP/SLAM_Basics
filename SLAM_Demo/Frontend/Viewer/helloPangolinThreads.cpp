#include <iostream>
#include <pangolin/pangolin.h>
#include <thread>

static const std::string window_name = "helloPangolinThreads";

void setup(){
    // 创建一个窗口，并将其内容与做主线程绑定
    pangolin::CreateWindowAndBind(window_name, 640, 480);

    // 进行深度测试
    glEnable(GL_DEPTH_TEST);

    // unset the current context from the main thread.
    pangolin::GetBoundWindow()->RemoveCurrent();
}

void run() {
    // 获得内容，并绑定到这个线程
    pangolin::BindToContext(window_name);
    // 进行深度测试
    glEnable(GL_DEPTH_TEST);

    // 定义投影和模型视角矩阵
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
            pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY));

    // 创建交互矩阵窗口
    pangolin::Handler3D handler(s_cam);
    pangolin::View &d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f).SetHandler(
            &handler);

    while(!pangolin::ShouldQuit()){
        // 清楚屏幕并进行缓冲
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        d_cam.Activate(s_cam);

        // 渲染立方体
        pangolin::glDrawColouredCube();

        // 进行缓冲
        pangolin::FinishFrame();
    }
    pangolin::GetBoundWindow()->RemoveCurrent();
}

int main()
{
	setup();

	std::thread render_loop;
	render_loop = std::thread(run);
	render_loop.join();

	return 0;
}