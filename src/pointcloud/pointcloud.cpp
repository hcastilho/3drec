#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <memory>
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
std::shared_ptr<pcl::visualization::PCLVisualizer> viewer (
        new pcl::visualization::PCLVisualizer ("3D Viewer"));

void create_point_cloud() {
    Mat disp;
    data_mutex.lock();
    sbm(stereo_pair.view[0], stereo_pair.view[1], disp);
    Mat img_rgb = stereo_pair.view[0].clone();
    data_mutex.unlock();

    Mat xyz;
    reprojectImageTo3D(disp, xyz, Q, true);

    //Create point cloud and fill it
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (
            new pcl::PointCloud<pcl::PointXYZRGB>);
    double px, py, pz;
    uchar pr, pg, pb;
    int i,j;

    for (i = 0; i < img_rgb.rows; i++)
    {
        uchar* rgb_ptr = img_rgb.ptr<uchar>(i);
        double* recons_ptr = xyz.ptr<double>(i);
        for (j = 0; j < img_rgb.cols; j++)
        {
            //Get 3D coordinates
            px = recons_ptr[3*j];
            py = recons_ptr[3*j+1];
            pz = recons_ptr[3*j+2];

            //Get RGB info
            pb = rgb_ptr[3*j];
            pg = rgb_ptr[3*j+1];
            pr = rgb_ptr[3*j+2];

            //Insert info into point cloud structure
            pcl::PointXYZRGB point;
            point.x = px;
            point.y = py;
            point.z = pz;
            uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
                    static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
            point.rgb = *reinterpret_cast<float*>(&rgb);
            point_cloud_ptr->points.push_back (point);
        }
    }
    point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;

    // Show point cloud in visualizer
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
    viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, rgb, "reconstruction");

    //pcl::io::savePCDFileASCII ("test_pcd.pcd", point_cloud_ptr);
    pcl::io::savePCDFileBinary("test_pcd.pcd", *point_cloud_ptr);
}

volatile bool threadRun = false;
void frame_graber() {
    threadRun = true;

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
            break;
        if( key  == g_KEY ) {
            data_mutex.lock();
            stereo_pair.view[0] = view[0].clone();
            stereo_pair.view[1] = view[1].clone();
            data_mutex.lock();
            std::thread point_cloud_thread(create_point_cloud);
        }
    }
}

int main(int argc, char* argv[])
{
    const string intrinsics_file = argc > 1 ? argv[1] : "intrinsics.yml";
    const string extrinsics_file = argc > 2 ? argv[2] : "extrinsics.yml";
    const string bm_file = argc > 3 ? argv[3] : "bm_settings.yml";

    FileStorage fs(bm_file, CV_STORAGE_READ);
    StereoBM bm;
    fs["preFilterCap"] >> bm.state->preFilterCap;
    fs["SADWindowSize"] >> bm.state->SADWindowSize;
    fs["minDisparity"] >> bm.state->minDisparity;
    fs["numberOfDisparities"] >> bm.state->numberOfDisparities;
    fs["textureThreshold"] >> bm.state->textureThreshold;
    fs["uniquenessRatio"] >> bm.state->uniquenessRatio;
    fs["speckleWindowSize"] >> bm.state->speckleWindowSize;
    fs["speckleRange"] >> bm.state->speckleRange;
    fs["disp12MaxDiff"] >> bm.state->disp12MaxDiff;


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
    stereoRectify( M1, D1, M2, D2, imageSize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, imageSize, &roi1, &roi2 );
    bm.state->roi1 = roi1;
    bm.state->roi2 = roi2;

    initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2, m_map[0][0], m_map[0][1]);
    initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_16SC2, m_map[1][0], m_map[1][1]);

    //viewer->setBackgroundColor (0, 0, 0);
    //viewer->addCoordinateSystem ( 1.0 );
    //viewer->initCameraParameters ();

    std::thread frame_graber_thread(frame_graber);
    frame_graber_thread.join();
}
