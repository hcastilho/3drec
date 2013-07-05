#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/contrib/contrib.hpp"

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace cv;
using namespace std;

//This function creates a PCL visualizer, sets the point cloud to view and returns a pointer
boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "reconstruction");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
  viewer->addCoordinateSystem ( 1.0 );
  viewer->initCameraParameters ();
  return (viewer);
}

int main(int argc, char* argv[])
{
    int cameraId[2] = {1,0};

    const char ESC_KEY = 27;

    FileStorage fs("matching_bm_settings.yml", CV_STORAGE_READ);
    string intrinsicsFile, extrinsicsFile;
    fs["intrinsicsFile"] >> intrinsicsFile;
    fs["extrinsicsFile"] >> extrinsicsFile;

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


    fs.open(intrinsicsFile, CV_STORAGE_READ);
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

    fs.open(extrinsicsFile, CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        cout << "Failed to open extrinsics file ";
        return -1;
    }

    Mat R, T, R1, P1, R2, P2, Q;
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

    //Mat map11, map12, map21, map22;
    Mat map[2][2];
    initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2, map[0][0], map[0][1]);
    initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_16SC2, map[1][0], map[1][1]);


    //StereoBM bm;
    //StereoVar var;

    Mat disp, disp8;


    VideoCapture inputCapture[2];

    int i, k;
    Mat view[2];
    char key;
    for (k = 0; k < 2; ++k)
    {
        inputCapture[k].open(cameraId[k]);
        if (!inputCapture[k].isOpened()) {
            cerr << "Inexistent input: " << cameraId[k];
            return 1;
        }

    }
    for(i = 0;; ++i)
    {
        for( k = 0; k < 2; k++ )
        {
            inputCapture[k].grab();
        }
        for( k = 0; k < 2; k++ )
        {
            Mat aux;
            inputCapture[k].retrieve(view[k]);
            remap(view[k], aux, map[k][0], map[k][1], INTER_LINEAR);
            cvtColor(aux, view[k], CV_BGR2GRAY);
        }
        bm(view[0], view[1], disp);
        disp.convertTo(disp8, CV_8U);
        imshow("disparity", disp8);

        Mat xyz;
        reprojectImageTo3D(disp, xyz, Q, true);
        //saveXYZ(point_cloud_filename, xyz);


        //Create point cloud and fill it
        std::cout << "Creating Point Cloud..." <<std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

        double px, py, pz;
        uchar pr, pg, pb;
        int m,n;

        for (m = 0; m < img_rgb.rows; m++)
        {
            uchar* rgb_ptr = img_rgb.ptr<uchar>(m);
            double* recons_ptr = xyz.ptr<double>(m);
            for (n = 0; n < img_rgb.cols; n++)
            {
            //Get 3D coordinates
            px = recons_ptr[3*n];
            py = recons_ptr[3*n+1];
            pz = recons_ptr[3*n+2];

            //Get RGB info
            pb = rgb_ptr[3*n];
            pg = rgb_ptr[3*n+1];
            pr = rgb_ptr[3*n+2];

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

        //Create visualizer
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        viewer = createVisualizer( point_cloud_ptr );

        //Main loop
        while ( !viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }

        key = (char)waitKey(50);
        if( key  == ESC_KEY )
            break;
    }
}
