/*
 * @Author: Divenire
 * @Date: 2021-09-20 14:53:01
 * @LastEditors: Divenire
 * @LastEditTime: 2021-09-20 15:02:33
 * @Description: file content
 */


#include <iostream>
#include <pangolin/pangolin.h>

int main(/*int argc, char* argv[]*/)
{
  // Create OpenGL window in single line
  pangolin::CreateWindowAndBind("Main",640,480);

  // Data logger object
  // 待可视化的数据存储对象
  pangolin::DataLog log;

  // Optionally add named labels
  std::vector<std::string> labels;
  labels.push_back(std::string("sin(t)"));
  labels.push_back(std::string("cos(t)"));
  labels.push_back(std::string("sin(t)+cos(t)"));

  // 设置Labels的顺序对应的数据名称
  log.SetLabels(labels);

  const float tinc = 0.01f;

  // OpenGL 'view' of data. We might have many views of the same data.
  // 数据可视化操作
  /**
   * @description: 
   * @param {
        Plotter(
        DataLog* default_log,需要绘制的数据对象
        float left=0, float right=600, float bottom=-1, float top=1, xy轴的上下界
        float tickx=30, float ticky=0.5, x轴和y轴的刻度大小
        Plotter* linked_plotter_x = 0,
        Plotter* linked_plotter_y = 0
    );

   * @return {*}
   */  
  pangolin::Plotter plotter(&log,
                            0.0f,
                            4.0f*(float)M_PI/tinc,
                            -4.0f,
                            4.0f,
                            (float)M_PI/(4.0f*tinc),
                            0.5f);
                            plotter.SetBounds(0.0, 1.0, 0.0, 1.0);
  plotter.Track("$i");//坐标轴自动滚动

  // Add some sample annotations to the plot（为区域着色）
  // 第一个标志块的方向为垂直方向，数值为50 π，判断方式为小于，颜色为带透明度的蓝色，
  // 第二个Marker将y轴大于3的区域标记为了红色，
  // 第三个Marker由于是等于，因此其只将y=3这一条线标记为了绿色
  plotter.AddMarker(pangolin::Marker::Vertical,   50*M_PI, pangolin::Marker::LessThan, pangolin::Colour::Blue().WithAlpha(0.2f) );
  plotter.AddMarker(pangolin::Marker::Horizontal,   3, pangolin::Marker::GreaterThan, pangolin::Colour::Red().WithAlpha(0.2f) );
  plotter.AddMarker(pangolin::Marker::Horizontal,    3, pangolin::Marker::Equal, pangolin::Colour::Green().WithAlpha(0.2f) );

  //加入构建好的plotter (画笔？)
  pangolin::DisplayBase().AddDisplay(plotter);

  float t = 0;

  // Default hooks for exiting (Esc) and fullscreen (tab).
  while( !pangolin::ShouldQuit() )
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 在循环中更新DataLog中的数据
    log.Log(sin(t),cos(t),sin(t)+cos(t));
    t += tinc;

    // Render graph, Swap frames and Process Events
    pangolin::FinishFrame();
  }

  return 0;
}
