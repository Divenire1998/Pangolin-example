/*
 * @Author: Divenire
 * @Date: 2021-09-14 15:00:53
 * @LastEditors: Divenire
 * @LastEditTime: 2021-09-20 11:04:10
 * @Description: 多线程Pangolin
 */


#include <pangolin/pangolin.h>
#include <thread>

static const std::string window_name = "HelloPangolinThreads";

void setup() {
    // create a window and bind its context to the main thread
    pangolin::CreateWindowAndBind(window_name, 640, 480);

    // enable depth
    glEnable(GL_DEPTH_TEST);

    // unset the current context from the main thread
    pangolin::GetBoundWindow()->RemoveCurrent();
}

void run() {
    // fetch the context and bind it to this thread
    // 重新将视窗绑定到当前线程
    pangolin::BindToContext(window_name);

    // we manually need to restore the properties of the context
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
        pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
    );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);

    while( !pangolin::ShouldQuit() )
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        // Render OpenGL Cube
        pangolin::glDrawColouredCube();

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }

    // unset the current context from the main thread
    pangolin::GetBoundWindow()->RemoveCurrent();
}

int main( int /*argc*/, char** /*argv*/ )
{
    // create window and context in the main thread
    // setup()函数创建了一个视窗用于后续的显示，但这个视窗实在主线程中创建的，
    // 因此在主线程调用玩后，需要使用GetBoundWindow()->RemoveCurrent()将其解绑。
    setup();

    // use the context in a separate rendering thread
    std::thread render_loop;
    render_loop = std::thread(run);
    
    // 等待Pangolin线程运行结束
    // render_loop.join();


    std::cout<<"system is running"<<std::endl;
    while (1)
    {
        /* code */
        std::cout<<"system is running"<<std::endl;
    }
    

    return 0;
}
