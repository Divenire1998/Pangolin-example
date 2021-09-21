/*
 * @Author: Divenire
 * @Date: 2021-09-20 14:34:17
 * @LastEditors: Divenire
 * @LastEditTime: 2021-09-21 10:02:11
 * @Description: 多视窗显示
 */
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>
#include "opencv2/imgcodecs/legacy/constants_c.h"

int main(int argc, char** argv){
    // 创建视窗
    pangolin::CreateWindowAndBind("MultiImage", 680, 480);
    // 启动深度测试
    glEnable(GL_DEPTH_TEST);
    // 设置摄像机
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(680, 480, 420, 420, 340, 240, 0.1, 1000),
        pangolin::ModelViewLookAt(-2, 0, -2, 0, 0, 0, pangolin::AxisY)
    );
    // ---------- 创建三个视窗 ---------- //
    // 下上左右
    pangolin::View& d_cam = pangolin::Display("cam")
        .SetBounds(0., 1., 0., 1., -680/480.)
        .SetHandler(new pangolin::Handler3D(s_cam));
    
    //创建视图函数中的第五个参数。我们注意到，在后两个视图的创建中，该参数是正值，而在创建动画视窗视使用的是负值。
    //该参数实际上表征的是视图的分辨率，当该参数取正值时，pangolin会将由前四个参数设置的视图大小进行裁剪
    //，以满足所设置的分辨率；当该值为负值时，pangolin会在将图片拉伸以充满由前四个参数设置的视图范围。

    //因此我们没有使用setHandler()设置视图句柄，而是使用了setLock()函数设置了视图锁定的位置，该函数会在我们缩放整个视窗后，
    //按照设定的lock选项自动锁定对其位置。
    //例如在本例中，我们将左上角的视图设置为left和top锁定，右下角的视图则设置为right和bottom锁定。


   // 创建glTexture容器用于读取图像
    // 图像宽度、图像高度、pangolin的内部图像存储格式，是否开启现行采样，边界大小（像素）、gl图像存储格式以及gl数据存储格式。

    pangolin::View& cv_img_1 = pangolin::Display("image_1")
        .SetBounds(2/3.0f, 1.0f, 0., 1/3.0f, -640/480.)
        .SetLock(pangolin::LockLeft, pangolin::LockTop);
    
    pangolin::View& cv_img_2 = pangolin::Display("image_2")
        .SetBounds(0., 1/3.0f, 1/3.0f, 1.0, -1241/376.)
        .SetLock(pangolin::LockRight, pangolin::LockBottom);
        
 
    pangolin::GlTexture imgTexture1(640, 480, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);
    pangolin::GlTexture imgTexture2(1241, 376, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);

    while(!pangolin::ShouldQuit()){
        // 清空颜色和深度缓存
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 从文件读取图像
        cv::Mat img1 = cv::imread("/home/divenire/Divenire_ws/dataset/tum/rgbd_dataset_freiburg1_desk/rgb/1305031452.791720.png");
        cv::Mat img2 = cv::imread("/home/divenire/Divenire_ws/dataset/KITTI/dataset/sequences/01/image_0/000001.png");
        // 向GPU装载图像
        imgTexture1.Upload(img1.data, GL_BGR, GL_UNSIGNED_BYTE);
        imgTexture2.Upload(img2.data, GL_BGR, GL_UNSIGNED_BYTE);

        // 显示图像
        cv_img_1.Activate();
        glColor3f(1.0f, 1.0f, 1.0f); // 设置默认背景色，对于显示图片来说，不设置也没关系
        imgTexture1.RenderToViewportFlipY(); // 需要反转Y轴，否则输出是倒着的
        
        cv_img_2.Activate();
        glColor3f(1.0f, 1.0f, 1.0f); // 设置默认背景色，对于显示图片来说，不设置也没关系
        imgTexture2.RenderToViewportFlipY();

        // 视窗启动
        d_cam.Activate(s_cam);
        // 添加一个立方体
        pangolin::glDrawColouredCube();


        pangolin::FinishFrame();
    }

    return 0;
}