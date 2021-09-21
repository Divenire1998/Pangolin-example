/*
 * @Author: Divenire
 * @Date: 2021-09-14 15:00:58
 * @LastEditors: Divenire
 * @LastEditTime: 2021-09-20 22:16:14
 * @Description: 窗口控件的基本使用方法，以及视窗分割
 *               保存视窗，录制视窗，按钮，checkBox的使用,字符串，键盘绑定
 */

#include <pangolin/pangolin.h>
#include <string>
#include <iostream>
// ----------------------------------------------------------------- //


// Reset按钮的回调函数
void SampleMethod()
{
    std::cout << "You typed ctrl-r or pushed reset" << std::endl;
    // std::cout << "Window width: " << i << std::endl;
}

// 用于ui动态显示
std::string name = "1";

// ----------------------------------------------------------------------- //
int main(/*int argc, char* argv[]*/)
{  

    // 创建视窗
    pangolin::CreateWindowAndBind("Main",640,480);
    // 启动深度测试
    glEnable(GL_DEPTH_TEST);
    // 创建一个摄像机
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
        pangolin::ModelViewLookAt(2,2,10, 0,0,0, pangolin::AxisY)
    );

    // 分割视窗，单位像素
    const int UI_WIDTH = 180;

    // 右侧用于显示视口
    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -640.0f/480.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    // 左侧用于创建控制面板，面板的名称为ui
    pangolin::CreatePanel("ui")
        .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

// -----------------------自定义空间列表--------------------------------------- //


    // 按钮
    pangolin::Var<bool> A_Button("ui.a_button", false, false);  

    // 按钮
    pangolin::Var<bool> SAVE_IMG("ui.save_img", false, false);  

    // 按钮
    pangolin::Var<bool> SAVE_WIN("ui.save_win", false, false); 

    // 按钮
    pangolin::Var<bool> RECORD_WIN("ui.record_win", false, false); 

    // 选框
    pangolin::Var<bool> A_Checkbox("ui.A_Checkbox", false, true); 

    // double滑条
    pangolin::Var<double> Double_Slider("ui.double_slider", 3, 0, 5); 

    // int滑条
    pangolin::Var<int> Int_Slider("ui.int_slider", 2, 0, 5); 

    // 字符串 
    pangolin::Var<std::string> A_string("ui.string",name);
    // 设置初始值为6 debug来看
    A_string = "6";

    // 回调函数
    pangolin::Var<std::function<void()>> reset("ui.Reset", SampleMethod);

    // 绑定键盘快捷键
    // Demonstration of how we can register a keyboard hook to alter a Var
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'b', pangolin::SetVarFunctor<double>("ui.a_slider", 3.5));

    // Demonstration of how we can register a keyboard hook to trigger a method
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'r', SampleMethod);

    // Default hooks for exiting (Esc) and fullscreen (tab).


    while( !pangolin::ShouldQuit() )
    {
        // Clear entire screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    

        d_cam.Activate(s_cam);
        // glColor3f(1.0,0.0,1.0);
        pangolin::glDrawColouredCube();

        static uint8_t count=10;
        count++;
        name = std::to_string(count);
        // 更新字符串
        A_string = name;


         // 各控件的回调函数
        if(pangolin::Pushed(A_Button))
            std::cout << "Push button A." << std::endl;
        
        if(A_Checkbox)
            // 勾选
            Int_Slider = Double_Slider;
        
        if( pangolin::Pushed(SAVE_WIN) )
            // 保存整个窗口
            pangolin::SaveWindowOnRender("window");

        if( pangolin::Pushed(SAVE_IMG) )
            // 保存显示视窗
            d_cam.SaveOnRender("cube");

        if( pangolin::Pushed(RECORD_WIN) )
            // 对整个窗口进行录像
            // pangolin::DisplayBase().RecordOnRender("ffmpeg:[fps=50,bps=8388608,unique_filename]//screencap.avi");

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }

    return 0;
}
