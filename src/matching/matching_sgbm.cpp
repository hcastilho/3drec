#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/contrib/contrib.hpp"

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
    int cameraId[2] = {1,0};

    const char ESC_KEY = 27;

    FileStorage fs("matching_sgbm_settings.yml", CV_STORAGE_READ);
    string intrinsicsFile, extrinsicsFile;
    fs["intrinsicsFile"] >> intrinsicsFile;
    fs["extrinsicsFile"] >> extrinsicsFile;

    StereoSGBM sgbm;
    fs["preFilterCap"] >> sgbm.preFilterCap;
    fs["SADWindowSize"] >> sgbm.SADWindowSize;
    fs["P1"] >> sgbm.P1;
    fs["P2"] >> sgbm.P2;
    fs["minDisparity"] >> sgbm.minDisparity;
    fs["numberOfDisparities"] >> sgbm.numberOfDisparities;
    fs["uniquenessRatio"] >> sgbm.uniquenessRatio;
    fs["speckleWindowSize"] >> sgbm.speckleWindowSize;
    fs["speckleRange"] >> sgbm.speckleRange;
    fs["disp12MaxDiff"] >> sgbm.disp12MaxDiff;
    fs["fullDP"] >> sgbm.fullDP;


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
            view[k] = aux;
        }
        sgbm(view[0], view[1], disp);
        double minVal; double maxVal;
        minMaxLoc(disp, &minVal, &maxVal );
        disp.convertTo(disp8, CV_8U, 255/(maxVal - minVal));
        imshow("disparity", disp8);

        key = (char)waitKey(50);
        if( key  == ESC_KEY )
            break;
    }
}
