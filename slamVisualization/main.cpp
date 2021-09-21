/*
 * @Author: Divenire
 * @Date: 2021-09-20 15:18:44
 * @LastEditors: Divenire
 * @LastEditTime: 2021-09-21 11:58:04
 * @Description: 使用KITTI数据集作为简单的SLAM演示程序
 */

#include "slamVisualization.h"
#include "opencv2/imgcodecs/legacy/constants_c.h"

using namespace cv;
using namespace std;
using namespace Eigen;

// 载入图像
void LoadImagesKITTI(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);


void LoadGroundTruthKITTI(const string &strPathToGt,std::vector<Eigen::Isometry3d>& vTransform);


string kitti_dataset_path = "/home/divenire/Divenire_ws/dataset/KITTI/dataset/sequences/00";
string kitti_gt_path = "/home/divenire/Divenire_ws/dataset/KITTI/dataset/poses/00.txt";



int main(int argc, char  *argv[])
{

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImagesKITTI(kitti_dataset_path, vstrImageLeft, vstrImageRight, vTimestamps);

    // load ground truth
    vector<Eigen::Isometry3d> vTransform;
    LoadGroundTruthKITTI(kitti_gt_path,vTransform);


    // 获得图像的数量
    const int nImages = vstrImageLeft.size();

    //初始化查看器
    slamVisualization visualizer(1280,800,1241,376);

    // 初始化视窗
    visualizer.initDraw();

    // 轨迹显示
    vector<Eigen::Vector3d> traj;

    // 双目图像显示
    cv::Mat imLeft, imRight;

    // 预先创建一个Window用于显示图像
    cv::namedWindow("Current Frame");

    for(int ni=0; ni<nImages; ni++)
    {

        double tframe = vTimestamps[ni];
        // 清除颜色缓存
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        // 注册ui回调函数
        visualizer.registerUICallback();

        // 开启相机视图
        visualizer.ActivateScamView();
        // 使用白色背景
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        Eigen::Quaterniond quat = Eigen::Quaterniond(vTransform[ni].rotation());
        Eigen::Vector3d pos = vTransform[ni].translation();
        traj.push_back(pos);
        
        // 显示数据
        visualizer.displayData(pos, quat);
        // 绘制轨迹可视化部分
        visualizer.drawCoordinate();
        visualizer.drawCamWithPose(pos, quat);
        visualizer.drawTraj(traj);

        // Pangolin显示图像
        imLeft = cv::imread(vstrImageLeft[ni]);
        imRight = cv::imread(vstrImageRight[ni]);
        visualizer.displayImg(imLeft,imRight);

        // OPENCV显示图像
        cv::imshow("Current Frame",imLeft);
        // 等待一下，防止图像刷新不出来
        cv::waitKey(1);

        // 循环与退出判断
        pangolin::FinishFrame();
        
        if(pangolin::ShouldQuit())
            break;

    }
    
    return 0;
}


// 类似 mono_kitti.cc， 不过是生成了双目的图像路径
void LoadImagesKITTI(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}



void LoadGroundTruthKITTI(const string &strPathToGt,std::vector<Eigen::Isometry3d>& vTransform)
{
    std::ifstream fin(kitti_gt_path);
    if(!fin){
        cout << "cannot find trajectory file at " << strPathToGt << endl;
        return ;
    }

      while (!fin.eof()) {
        double d,h,l;
        Eigen::Matrix3d R;
        fin >> R(0,0)>>R(0,1)>>R(0,2)>>d
            >> R(1,0)>>R(1,1)>>R(1,2)>>h
            >> R(2,0)>>R(2,1)>>R(2,2)>>l;

        Isometry3d Twr=Eigen::Isometry3d::Identity();
        Twr.rotate(R);
        Twr.pretranslate(Vector3d(d, h, l));
        vTransform.push_back(Twr);
        // cout<<"欧式变换 \n"<<Twr.matrix()<<endl<<"欧式变换的旋转矩阵 \n"<<Twr.rotation()<<endl<<"欧式变换的平移部分 \n"<<Twr.translation().transpose()<<endl;
    }
    cout << "read total " << vTransform.size() << " pose entries" << endl;
   

}