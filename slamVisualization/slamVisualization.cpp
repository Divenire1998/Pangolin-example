/*
 * @Author: Divenire
 * @Date: 2021-09-20 15:13:10
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2021-09-22 16:32:18
 * @Description: SLAM显示的一个简单库
 */

#include "slamVisualization.h"

using namespace std;


float ViewpointX = 0;
float ViewpointY = -100;
float ViewpointZ = -0.1;

void slamVisualization::initDraw()
{
    // 创建相机姿态显示窗口
    pangolin::CreateWindowAndBind("camera_pose", WIN_WIDTH_, WIN_HEIGHT_);

    // 启用深度测试
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    // 在OpenGL中使用颜色混合
    glEnable(GL_BLEND);
    // 选择混合选项
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

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
    s_cam_ = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(WIN_WIDTH_, WIN_HEIGHT_, 2000, 2000, WIN_WIDTH_/2, WIN_HEIGHT_/2, 0.1, 10000),
        pangolin::ModelViewLookAt(ViewpointX, ViewpointY, ViewpointZ, 0, 0, 0, 0.0,-1.0, 0.0)
    );


    // 控制面板的长宽
    int PANEL_WIDTH = WIN_WIDTH_ / 4;
    int PANEL_HEIGHT = WIN_HEIGHT_ / 4;

    // 根据控制面板的长宽来设置轨迹显示窗口，填充
    d_cam_ = pangolin::CreateDisplay()
        .SetBounds(0., 1., pangolin::Attach::Pix(PANEL_WIDTH), 1., -(float)WIN_WIDTH_/ (float)WIN_HEIGHT_)
        .SetHandler(new pangolin::Handler3D(s_cam_));

    // 控制面板
    pangolin::CreatePanel("ui")
        .SetBounds(pangolin::Attach::Pix(3.0f *PANEL_HEIGHT),
                    1.,
                    0.,
                    pangolin::Attach::Pix(PANEL_WIDTH),
                    (float)WIN_WIDTH_/ (float)WIN_HEIGHT_)
        .SetLock(pangolin::LockLeft, pangolin::LockTop);;
    
    // 创建显示控件
    ui_set_.clear();
     pangolin::Var<bool> follow_cam("ui.follow_cam", true, true);
    ui_set_.push_back(follow_cam); 
    pangolin::Var<bool> show_cam("ui.show_cam", true, true);
    ui_set_.push_back(show_cam); 
    pangolin::Var<bool> show_traj("ui.show_traj", true, true);
    ui_set_.push_back(show_traj); 
    pangolin::Var<bool> show_img("ui.show_img", true, true);
    ui_set_.push_back(show_img); 
    // 显示原点
    pangolin::Var<bool> show_coordinate("ui.origin", true, true);
    ui_set_.push_back(show_coordinate);


    // 创建数据显示空间
    pangolin::CreatePanel("data")
        .SetBounds( pangolin::Attach::Pix(2.0f *PANEL_HEIGHT), 
                    pangolin::Attach::Pix(3.0f *PANEL_HEIGHT),
                    0.,
                    pangolin::Attach::Pix(PANEL_WIDTH), 
                    (float)WIN_WIDTH_/ (float)WIN_HEIGHT_)
        .SetLock(pangolin::LockLeft, pangolin::LockCenter);
                    
    data_set_.clear();
    pangolin::Var<VecXd> curr_pos("data.pos", VecXd());
    data_set_.push_back(curr_pos);
    pangolin::Var<VecXd> curr_att("data.euler_angle", VecXd());
    data_set_.push_back(curr_att);

    // 原图片显示
    d_img_ = pangolin::CreateDisplay()
    .SetBounds(
                pangolin::Attach::Pix(1.0f *PANEL_HEIGHT), 
                pangolin::Attach::Pix(2.0f *PANEL_HEIGHT), 
                0.,
                pangolin::Attach::Pix(PANEL_WIDTH),
                (float)WIN_WIDTH_/ (float)WIN_HEIGHT_
    )
    .SetLock(pangolin::LockLeft, pangolin::LockBottom);

    // 跟踪图片显示
    d_track_ = pangolin::CreateDisplay()
        .SetBounds(0., pangolin::Attach::Pix(1.0f *PANEL_HEIGHT), 
                0., pangolin::Attach::Pix(PANEL_WIDTH), (float)WIN_WIDTH_/ (float)WIN_HEIGHT_)
        .SetLock(pangolin::LockLeft, pangolin::LockBottom);

}

// 绘制相机的外框
void slamVisualization::drawCam(const float scale=10.){

    if(scale < 0){
        cerr << "scale should be positive !\n";
        return;
    }
        
    const float w = 0.2 * scale;
    const float h = w * 0.75;
    const float z = w * 0.8;

    glLineWidth(2 * scale);
    // 绘制相机轮廓线
    glBegin(GL_LINES);
        glColor3f(0.0f, 1.0f, 1.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);
        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);
        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);
        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
    glEnd();
    
    return;
}

// 根据相机的位姿绘制相机外框
void slamVisualization::drawCamWithPose(Eigen::Vector3d &pos, Eigen::Quaterniond &quat){
    if (!camera_visible_)
        return;

    Eigen::Matrix3d R = quat.toRotationMatrix();

    glPushMatrix();
    std::vector<GLdouble> Twc = {R(0, 0), R(1,0), R(2, 0), 0.,
                                R(0, 1), R(1, 1), R(2, 1), 0.,
                                R(0, 2), R(1, 2), R(2, 2), 0.,
                                pos.x(), pos.y(), pos.z(), 1.};
    glMultMatrixd(Twc.data());
    drawCam();
    glPopMatrix();

    // 如果选了follow camara
    if (follow_camera_)
    {
        pangolin::OpenGlMatrix twc1;
        twc1.m[0] = R(0,0);
        twc1.m[1] = R(1,0);
        twc1.m[2] = R(2,0);
        twc1.m[3]  = 0.0;

        twc1.m[4] = R(0,1);
        twc1.m[5] = R(1,1);
        twc1.m[6] = R(2,1);
        twc1.m[7]  = 0.0;

        twc1.m[8] = R(0,2);
        twc1.m[9] = R(1,2);
        twc1.m[10] = R(2,2);
        twc1.m[11]  = 0.0;

        twc1.m[12] = pos.x();
        twc1.m[13] = pos.y();
        twc1.m[14] = pos.z();
        twc1.m[15]  = 1.0;

        // 视角跟随相机
        s_cam_.Follow(twc1);
    }

    
}


// 绘制相机的轨迹
void slamVisualization::drawTraj(vector<Eigen::Vector3d> &traj){
    if(!traj_visible_)
        return;
    glLineWidth(2);
    glBegin(GL_LINES);
    glColor3f(0.f, 1.f, 0.f);
    for(size_t i=0; i<traj.size() - 1; i++){
        glVertex3d(traj[i].x(), traj[i].y(), traj[i].z());
        glVertex3d(traj[i+1].x(), traj[i+1].y(), traj[i+1].z());
    }
    glEnd();
}



void slamVisualization::drawCoordinate(){
    if(!coordinate_visible_)
        return;
    // 绘制位于原点的坐标系
    glLineWidth(30);
    glBegin(GL_LINES);
        glColor3f(1.0f, 0.f, 0.f);
        glVertex3f(0, 0, 0);
        glVertex3f(1, 0, 0);
        glColor3f(0.f, 1.0f, 0.f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 1, 0);
        glColor3f(0.f, 0.f, 1.f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 1);
    glEnd();
}

// 显示当前的图像
void slamVisualization::displayImg( const string leftImagePath, const string rightImagePath){
    
    if(!img_visible_)
        return;
 
    // 像素排列方式重新布局，防止KITTI数据图片显示失败
    glPixelStorei(GL_UNPACK_ALIGNMENT,1);


    // 使用Pangolin读取图片
    pangolin::GlTexture imageTexture_((pangolin::LoadImage(leftImagePath)));

    // 显示图像
    d_img_.Activate();
    glColor3f(1.0f, 1.0f, 1.0f); // 设置默认背景色，对于显示图片来说，不设置也没关系
    imageTexture_.RenderToViewportFlipY(); // 需要反转Y轴，否则输出是倒着的



    // 使用Pangolin读取图片
    pangolin::GlTexture trackTexture_((pangolin::LoadImage(rightImagePath)));

    // 显示图像
    d_track_.Activate();
    glColor3f(1.0f, 1.0f, 1.0f); // 设置默认背景色，对于显示图片来说，不设置也没关系
    trackTexture_.RenderToViewportFlipY(); // 需要反转Y轴，否则输出是倒着的
}

// 激活显示的图像

void slamVisualization::ActivateScamView()
{
    d_cam_.Activate(s_cam_);
}


void slamVisualization::registerUICallback()
{
    follow_camera_ = ui_set_[0] ? true : false;
    camera_visible_ = ui_set_[1] ? true : false;
    traj_visible_ = ui_set_[2] ? true : false;
    img_visible_ = ui_set_[3] ? true : false;
    coordinate_visible_ = ui_set_[4] ? true : false;
    
}

void slamVisualization::displayData(Eigen::Vector3d &pos, Eigen::Quaterniond& quat){
    VecXd tmp_pose, tmp_euler;
    tmp_pose.vec_ = pos;
    tmp_euler.vec_ = quat.matrix().eulerAngles(2, 1, 0);

    tmp_euler.vec_  *= (180 / M_PI);
    data_set_[0] = tmp_pose;
    data_set_[1] = tmp_euler;
}