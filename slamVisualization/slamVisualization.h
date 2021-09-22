#ifndef SLAMVISUALIZESION_H
#define SLAMVISUALIZESION_H

#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;


// 创建一个VectorXd用于pangolin::Var输出数据
// 根据pangolin文档，自定义类型需要重载输入输出流操作符
struct VecXd
{
    Eigen::VectorXd vec_ = Eigen::Vector3d::Zero();
};

// 使用inline休息避免头文件中的非模板、非成员重复包含
inline ostream& operator << (ostream& out, const VecXd& r){
    int N = r.vec_.size();
    out.setf(ios::fixed);
    out << '=' << " [";
    for(int i=0; i< N-1; i++){
        out << setprecision(2) << r.vec_(i) << ", ";
    }
    out << r.vec_(N-1) << "]";
    return out;
}

inline istream& operator >> (istream& in, VecXd& r){
    return in;
}





class slamVisualization
{
private:
    pangolin::OpenGlRenderState s_cam_;
    pangolin::View d_cam_, d_img_, d_track_;
    pangolin::GlTexture imageTexture_, trackTexture_;
    pangolin::DataLog pose_log_;

    // 存储bool类型的ui面板的控件对象
    std::vector<pangolin::Var<bool>> ui_set_;
    // 存储data面板的控件对象
    std::vector<pangolin::Var<VecXd>> data_set_;

    // 是否跟随相机
    bool follow_camera_ =true;
    // 是否显示相机
    bool camera_visible_ = true;
    // 是否显示轨迹
    bool traj_visible_ = true;
    // 是否显示参考坐标系
    bool coordinate_visible_ = true;
    // 是否显示图像
    bool img_visible_ = true;
    // 窗口尺寸
    int WIN_WIDTH_;
    int WIN_HEIGHT_;

    // 图像尺寸
    int PICTURE_WIDTH_;
    int PICTURE_HEIGHT_;

public:
    slamVisualization();
    slamVisualization(int width = 640, int height = 480,int pic_width=1241,int pic_height=376)
                    :WIN_WIDTH_(width), WIN_HEIGHT_(height),PICTURE_WIDTH_(pic_width),PICTURE_HEIGHT_(pic_height)
    {}

    ~slamVisualization(){}
    // ------------------------------------------------

    void initDraw();

    void ActivateScamView();

    void drawCamWithPose(Eigen::Vector3d &pos, Eigen::Quaterniond &quat);

    void drawTraj(vector<Eigen::Vector3d> &traj);

    void drawCam(const float scale );

    void drawCoordinate();

    void displayImg(const string leftImagePath,const string rightImagePath);

    void displayData(Eigen::Vector3d &pos, Eigen::Quaterniond &quat);

    void registerUICallback();


};


#endif