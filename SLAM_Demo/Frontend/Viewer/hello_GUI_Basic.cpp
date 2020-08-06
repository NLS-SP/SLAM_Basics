//
// Created by Robotics_qi on 2020/7/6.
//

#include <string>
#include <iostream>
#include <pangolin/pangolin.h>

void sampleMethod(){
    std::cout << "you typed ctrl-r or pused reset" << std::endl;
}

int main()
{
    // 创建名称为main的视窗，大小为640×480
    pangolin::CreateWindowAndBind("Main", 640, 480);
    // 启动深度测试
    glEnable(GL_DEPTH_TEST);
    // 创建观察相机视图ProjectMatrix(int h, int w, int fu, int fv, int cu, int cv, int zNear, int zFar)
    // ModelViewLookAt(double x, double y, double z, double lx, double ly, double lz, AxisDirection Up)
    // 参数为相机所在位置以及相机所看视点位置（一般设置在原点）
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 100),
            pangolin::ModelViewLookAt(-0, 0.5, -3, 0, 0, 0, pangolin::AxisY));

    // 分割视窗
    const int UI_WIDTH = 180;

    // 右侧显示窗口
    pangolin::View& d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -640.f/480.f).SetHandler(new pangolin::Handler3D(s_cam));

    // 左侧创建控制面板
    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

    // 创建控制面板的控制对象, pangolin中
    pangolin::Var<bool> A_Button("ui.a_button", false, false);         // 按钮
    pangolin::Var<bool> A_Checkbox("ui.a_checkbox", false, true);      // 选框
    pangolin::Var<double> Double_Slider("ui.a_slider", 3, 0, 5);    // double 滑条
    pangolin::Var<int> Int_Slider("ui.b_slider", 2, 0, 5);          // int滑条
    pangolin::Var<std::string> A_String("ui.a_string", "Hello Pangolin");

    pangolin::Var<bool> SAVE_IMG("ui.save_img", false, false); // 按钮
    pangolin::Var<bool> SAVE_WIN("io.save_win", false, false);
    pangolin::Var<bool> RECORD_WIN("ui.record_win", false ,false);

    pangolin::Var<std::function<void()> > reset("ui.Reset", sampleMethod);

    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'b', pangolin::SetVarFunctor<double>("ui.a_slider", 3.5));

    // Demonstration of how we can register a keyboard hook to trigger a method
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'r', sampleMethod);
    // Default hooks for exiting (Esc) and fullscreen (tab).
    while( !pangolin::ShouldQuit() )
    {
        // Clear entire screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 各控件的回调函数
        if(pangolin::Pushed(A_Button))
            std::cout << "Push button A." << std::endl;

        if(A_Checkbox)
            Int_Slider = Double_Slider;
        // 保存整个win
        if( pangolin::Pushed(SAVE_WIN) )
            pangolin::SaveWindowOnRender("window");
        // 保存view
        if( pangolin::Pushed(SAVE_IMG) )
            d_cam.SaveOnRender("cube");
        // 录像
        if( pangolin::Pushed(RECORD_WIN) )
            pangolin::DisplayBase().RecordOnRender("ffmpeg:[fps=50,unique_filename]//screencap.mp4");
        d_cam.Activate(s_cam);
        // glColor3f(1.0,0.0,1.0);
        pangolin::glDrawColouredCube();

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }

    return 0;
}

