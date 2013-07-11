#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <memory>
#include <chrono>
#include <thread>
#include <mutex>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/contrib/contrib.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


#define ESC_KEY 27
#define g_KEY 103

using namespace cv;
using namespace std;


struct StereoPair {
    Mat view[2];
};
StereoPair stereo_pair;
StereoBM sbm;
Mat Q;
cv::Mat m_map[2][2];
std::mutex data_mutex;
std::mutex viewer_mutex;
std::shared_ptr<pcl::visualization::PCLVisualizer> viewer (
        new pcl::visualization::PCLVisualizer ("3D Viewer"));

void create_point_cloud() {
    cout << "In create_point_cloud" << endl;
    Mat disp, aux[2];
    data_mutex.lock();
    cvtColor(stereo_pair.view[0], aux[0], CV_BGR2GRAY);
    cvtColor(stereo_pair.view[1], aux[1], CV_BGR2GRAY);
    sbm(aux[0], aux[1], disp);
    Mat img_rgb = stereo_pair.view[0].clone();
    data_mutex.unlock();


    Mat disp8;
    double minVal; double maxVal;
    minMaxLoc( disp, &minVal, &maxVal );
    //cout << "Min disp: " << minVal << " Max value: " <<  maxVal << endl;
    disp.convertTo(disp8, CV_8U, 255/(maxVal - minVal));
    imshow("disparity", disp8);

    Mat xyz;
    reprojectImageTo3D(disp, xyz, Q, true);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (
            new pcl::PointCloud<pcl::PointXYZRGB>);
    int i,j;

    cout << "Start Mat->PCL" << endl;
    for (i = 0; i < xyz.rows; i++)
    {
        for (j = 0; j < xyz.cols; j++)
        {

            // TODO remove points that have no interest
            if (xyz.at<cv::Point3f>(i, j).z > 1000) continue;
            pcl::PointXYZRGB point;
            point.x = xyz.at<cv::Point3f>(i, j).x;
            point.y = xyz.at<cv::Point3f>(i, j).y;
            point.z = xyz.at<cv::Point3f>(i, j).z;
            // OpenCV is BGR
            point.r = img_rgb.at<cv::Point3i>(i, j).z;
            point.g = img_rgb.at<cv::Point3i>(i, j).y;
            point.b = img_rgb.at<cv::Point3i>(i, j).x;
            point_cloud_ptr->points.push_back (point);
        }
    }
    point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;
    cout << "End Mat->PCL" << endl;

    // Show point cloud in visualizer
    cout << "Show point cloud" << endl;
    viewer_mutex.lock();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
    viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, rgb, "reconstruction");
    viewer_mutex.unlock();

    //cout << "Storing point cloud file" << endl;
    pcl::io::savePCDFileASCII ("test_pcd_ascii.pcd", *point_cloud_ptr);
    //pcl::io::savePCDFileBinary("test_pcd.pcd", *point_cloud_ptr);

    //FileStorage fs("disp.yml", CV_STORAGE_WRITE);
    //fs << "disp" << disp;
    //FileStorage fs("disp8.yml", CV_STORAGE_WRITE);
    //fs << "disp8" << disp8;
    //fs.open("xyz.yml", CV_STORAGE_WRITE);
    //fs << "xyz" << xyz;
    cout << "End create_point_cloud" << endl;
}

volatile bool threadRun = false;
void visualizer_update() {
    while(threadRun) {
        viewer_mutex.lock();
        viewer->spinOnce(100);
        viewer_mutex.unlock();
        std::chrono::milliseconds dura( 10 );
        std::this_thread::sleep_for( dura );
    }
}

void frame_graber() {

    int k;
    char key;
    int cameraId[2] = {1,0};
    VideoCapture inputCapture[2];
    Mat view[2];

    for (k = 0; k < 2; ++k)
    {
        inputCapture[k].open(cameraId[k]);
        if (!inputCapture[k].isOpened()) {
            cerr << "Inexistent input: " << cameraId[k];
            return;
        }
    }

    while (threadRun) {
        // TODO skip processing on some frames
        //Glib::usleep(50000);

        for( k = 0; k < 2; k++ )
        {
            inputCapture[k].grab();
        }
        for( k = 0; k < 2; k++ )
        {
            Mat aux;
            inputCapture[k].retrieve(view[k]);
            remap(view[k], aux, m_map[k][0], m_map[k][1], INTER_LINEAR);
            view[k]=aux;
        }
        imshow("view", view[0]);

        key = (char)waitKey(50);
        if( key  == ESC_KEY )
            threadRun = false;
        if( key  == g_KEY ) {
            data_mutex.lock();
            stereo_pair.view[0] = view[0].clone();
            stereo_pair.view[1] = view[1].clone();
            data_mutex.unlock();
            cout << "Starting create_point_cloud" << endl;
            std::thread point_cloud_thread(create_point_cloud);
            point_cloud_thread.detach();
        }
    }
}

int main(int argc, char* argv[])
{
    threadRun = true;

    const string intrinsics_file = argc > 1 ? argv[1] : "intrinsics.yml";
    const string extrinsics_file = argc > 2 ? argv[2] : "extrinsics.yml";
    const string bm_file = argc > 3 ? argv[3] : "bm_settings.yml";

    FileStorage fs(bm_file, CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        cout << "Failed to open bm settings file ";
        return -1;
    }
    fs["preFilterCap"] >> sbm.state->preFilterCap;
    fs["SADWindowSize"] >> sbm.state->SADWindowSize;
    fs["minDisparity"] >> sbm.state->minDisparity;
    fs["numberOfDisparities"] >> sbm.state->numberOfDisparities;
    fs["textureThreshold"] >> sbm.state->textureThreshold;
    fs["uniquenessRatio"] >> sbm.state->uniquenessRatio;
    fs["speckleWindowSize"] >> sbm.state->speckleWindowSize;
    fs["speckleRange"] >> sbm.state->speckleRange;
    fs["disp12MaxDiff"] >> sbm.state->disp12MaxDiff;

    fs.open(intrinsics_file, CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        cout << "Failed to open intrinsics file ";
        return -1;
    }

    Mat M1, D1, M2, D2;
    Size imageSize;
    std::vector<int> imgsize;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;
    fs["imageSize"] >> imgsize;
    imageSize = Size(imgsize[0], imgsize[1]);

    fs.open(extrinsics_file, CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        cout << "Failed to open extrinsics file ";
        return -1;
    }

    Mat R, T, R1, P1, R2, P2; //, Q;
    fs["R"] >> R;
    fs["T"] >> T;
    //fs["R1"] >> R1;
    //fs["R2"] >> R2;
    //fs["P1"] >> P1;
    //fs["P2"] >> P2;
    //fs["Q"] >> Q;

    Rect roi1, roi2;
    stereoRectify( M1, D1, M2, D2, imageSize, R, T, R1, R2, P1, P2, Q,
            CALIB_ZERO_DISPARITY, -1, imageSize, &roi1, &roi2 );
    sbm.state->roi1 = roi1;
    sbm.state->roi2 = roi2;
    //stereoRectify( M1, D1, M2, D2, imageSize, R, T, R1, R2, P1, P2, Q,
    //        CALIB_ZERO_DISPARITY, -1, imageSize, &sbm.state->roi1, &sbm.state->roi2 );

    initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2,
            m_map[0][0], m_map[0][1]);
    initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_16SC2,
            m_map[1][0], m_map[1][1]);

    //viewer->setBackgroundColor (0, 0, 0);
    viewer->setBackgroundColor (255, 255, 255);
    viewer->addCoordinateSystem ( 1.0 );

    viewer->initCameraParameters ();
    viewer->spinOnce(100);

    std::thread visualizer_update_thread(visualizer_update);
    std::thread frame_graber_thread(frame_graber);
    frame_graber_thread.join();
    visualizer_update_thread.join();
}
