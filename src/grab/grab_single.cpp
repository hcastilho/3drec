#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{

    char key;
    const char ESC_KEY = 27;
    int i;
    int cameraId = 1;
    VideoCapture inputCapture;

    cout << "Opening Cameras\n";
    inputCapture.open(cameraId);
    if (!inputCapture.isOpened()) {
        cerr << "Inexistent input: " << cameraId;
        return 1;
    }

    cout << "Done\n";


    Mat view;

    cout << "Starting calibration\n";
    for(i = 0;; ++i)
    {
        //inputCapture.grab();
        //inputCapture.retrieve(view);
        inputCapture >> view;
        imshow("View", view);

        key = (char)waitKey(50);
        if( key  == ESC_KEY )
            break;

    }

}
