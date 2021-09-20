/*
 * @Author: Divenire
 * @Date: 2021-09-20 15:18:44
 * @LastEditors: Divenire
 * @LastEditTime: 2021-09-20 20:11:59
 * @Description: 使用KITTI数据集作为简单的SLAM演示程序
 */

#include "slamVisualization.h"
#include "opencv2/imgcodecs/legacy/constants_c.h"

using namespace cv;
using namespace std;

queue<string> imgFileNames;
queue<ulong> imgTimeStamps;

const char * dataset_gt = "/home/divenire/Divenire_ws/dataset/EuRoC/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv";
const char * dataset_img_time = "/home/divenire/Divenire_ws/dataset/EuRoC/MH_01_easy/mav0/cam0/data.csv";
const char * dataset_img = "/home/divenire/Divenire_ws/dataset/EuRoC/MH_01_easy/mav0/cam0/data/";



// 类似 mono_kitti.cc， 不过是生成了双目的图像路径
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
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








int main(int argc, char  *argv[])
{

    
    FILE *fp_gt, *fp_img;
    // 请自行修改数据集路径
    fp_gt = fopen(dataset_gt, "r");
    fp_img = fopen(dataset_img_time, "r");
    if(fp_gt == nullptr || fp_img == nullptr)
    {
        cout << "failed to open file !\n";
        return -1;
    }
    // =================== 读取图片路径 ====================//
    // 跳过第一行
    char fl_buf[1024];
    fgets(fl_buf, sizeof(fl_buf), fp_img);
  
    while(!feof(fp_img)){
        char filename[23];
        ulong timestamp;
        fscanf(fp_img, "%lu,%s", &timestamp, filename);
        
        imgTimeStamps.push(timestamp);
        imgFileNames.push(string(filename));
    }
    // ===================读取groundtruth =================================== //
    // 跳过第一行
    fgets(fl_buf, sizeof(fl_buf), fp_gt);

    //初始化查看器
    slamVisualization visualizer(1504, 960);
    // 初始化视窗
    visualizer.initDraw();

    vector<Eigen::Vector3d> traj;


while (!feof(fp_gt))
    {
        // 常规操作
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        visualizer.ActivateScamView();

        // 注册ui回调函数
        visualizer.registerUICallback();

        // 从数据集中读取数据
        // 创建数据寄存器    
        ulong time_stamp(0);
        double px(0.), py(0.), pz(0.);
        double qw(0.), qx(0.), qy(0.), qz(0.);
        double vx(0.), vy(0.), vz(0.);
        double bwx(0.), bwy(0.), bwz(0.), bax(0.), bay(0.), baz(0.);
        fscanf(fp_gt, "%lu,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
            &time_stamp, &px, &py, &pz,
            &qw, &qx, &qy, &qz,
            &vx, &vy, &vz,
            &bwx, &bwy, &bwz,
            &bax, &bay, &baz);

        Eigen::Quaterniond quat(qw, qx, qy, qz);
        Eigen::Vector3d pos(px, py, pz);
        traj.push_back(pos);

        // 显示当前的姿态和位置
        visualizer.displayData(pos, quat);

        // 绘制轨迹可视化部分
        visualizer.drawCoordinate();
        visualizer.drawCamWithPose(pos, quat);
        visualizer.drawTraj(traj);

        // 弹出当前时刻之前的图像
        double imu_time, img_time;
        imu_time = (double)time_stamp / 1e9; 
        img_time = (double)imgTimeStamps.front() / 1e9;
        if(imu_time > img_time){
            // cout << imgFileNames.front() << endl;
            imgTimeStamps.pop();
            imgFileNames.pop();
        }


        // 显示图像
        string img_file = dataset_img + imgFileNames.front();
        cv::Mat img = cv::imread(img_file, CV_LOAD_IMAGE_COLOR);
        visualizer.displayImg(img);
        
        // 循环与退出判断
        pangolin::FinishFrame();
        
        if(pangolin::ShouldQuit())
            break;

    }
    
    return 0;
}
