/*
 * @Author: Divenire
 * @Date: 2021-09-13 21:14:40
 * @LastEditors: Divenire
 * @LastEditTime: 2021-09-20 10:15:33
 * @Description: pangolin 与 opengl基本使用，在原点出绘制立方体和坐标轴
 */

#include <pangolin/pangolin.h>

int main( int argc, char** argv )
{
    // 创建名称为“Main”的GUI窗口，尺寸为640,480
    pangolin::CreateWindowAndBind("Hello Pangolin",640,480);
    
    // 启动深度测试，OpenGL只绘制最前面的一层，绘制时检查当前像素前面是否有别的像素，
    // 如果别的像素挡住了它，那它就不会绘制.
    glEnable(GL_DEPTH_TEST);


    /**
     * @description: Define Camera Render Object (for view / scene browsing)
     *               在视窗中放置一个相机，给出相机自身的位置和相机观察的位置，以及相机本身哪个轴朝上。
     * @param {
     * OpenGlMatrix& projection_matrix:定义相机投影模型：
     *                                  ProjectionMatrix(w, h, fu, fv, u0, v0, zNear, zFar)
     *                                  参数依次为观察相机的图像高度、宽度、焦距fu,fy和漂移cx,cy以及最近和最远视距
     * OpenGlMatrix& modelview_matrix:定义观测方位向量：
     *                                  ModelViewLookAt( x,  y,  z,  lx,  ly,  lz, AxisDirection up);                                 
     *                                  观测点位置(相机所在位置)：(x y z)
     *                                  观测目标位置(相机所看的视点位置)：(lx, ly, lz)
     *                                  观测的方位向量：(0.0,-1.0, 0.0)
     * }
     **/

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
        pangolin::ModelViewLookAt(0,0,10, 0,0,0, pangolin::AxisY)
    );

    /**
     * @description: 接下来我们需要创建一个交互式视图（view）用于显示上一步摄像机所“拍摄”到的内容.
     * 
     * @param {}
     * @return {None}
     */    
    pangolin::View& d_cam = pangolin::CreateDisplay()
        // setBounds()函数前四个参数依次表示视图在视窗中的范围（下、上、左、右），
        // 可以采用相对坐标（0~1）以及绝对坐标（使用Attach对象）最后一个参数实际上
        // 表征的是视图的分辨率，当该参数取正值时，pangolin会将由前四个参数设置的视图大小进行裁剪
        //  ，以满足所设置的分辨率；当该值为负值时，pangolin会在将图片拉伸以充满由前四个参数设置的视图范围。
        .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
        // 视图可移动
        .SetHandler(new pangolin::Handler3D(s_cam));


    while( !pangolin::ShouldQuit() )
    {
        // 清空颜色和深度缓存
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        d_cam.Activate(s_cam);

        // 绘制一个坐标轴 x y z分别为红绿蓝

        glLineWidth( 3 );//设置线的宽度为3
        glBegin( GL_LINES); //画线

            glColor3f(1.0,0.0,0.0); // 红色
            glVertex3f(0.0,0.0,0.0); 
            glVertex3f(1.0,0.0,0.0);

            glColor3f(0.0,1.0,0.0); // 绿色
            glVertex3f(0.0,0.0,0.0); 
            glVertex3f(0.0,1.0,0.0);

            glColor3f(0.0,0.0,1.0); // 蓝色
            glVertex3f(0.0,0.0,0.0); 
            glVertex3f(0.0,0.0,1.0);
            
        glEnd(); 

        glLineWidth( 6 );//设置线的宽度为3
        glBegin( GL_LINES); //画线
            //这样画线可以画出渐变色的感觉
            glColor3f(1.0,0.0,0.0); // 红色
            glVertex3f(0.0,0.0,0.0); 
            glColor3f(0.0,1.0,0.0); // 绿色
            glVertex3f(1.0,1.0,1.0);
            
        glEnd(); 
        // 在原点绘制一个立方体
        pangolin::glDrawColouredCube(-0.3,0.3);

        // 在绘制完成后，需要使用FinishFrame命令刷新视窗。
        pangolin::FinishFrame();
    }
    
    return 0;
}
