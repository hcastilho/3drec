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
    //time_t start, end; // start and end times
    //double fps;// fps calculated using number of frames / seconds
    //double sec;// floating point seconds elapsed since start

    char key;
    const char ESC_KEY = 27;
    //const char SPACE_KEY = 32;
    // nrFrames number of good frames to capture
    int i, j, k;
    int cameraId[2] = {0,1};
    Size imageSize = Size(0, 0);
    VideoCapture inputCapture[2];

    cout << "Opening Cameras\n";
    for (k = 0; k<2; ++k)
    {
        inputCapture[k].open(cameraId[k]);
        if (!inputCapture[k].isOpened()) {
            cerr << "Inexistent input: " << cameraId[k];
            return 1;
        }

    }
    cout << "Done\n";


    Mat canvas;
    double sf;
    int w, h;
    Mat view[2];

    cout << "Starting calibration\n";
    for(i = j = 0;; ++i)
    {
        //cout << "i: " << i << "\n";
        //cout << ">>>>>> Press any key to capture\n";
        //key = (char)waitKey(0);


        //for (int k = 0; k < 1; ++k)
        //{
        //    inputCapture[k].grab();
        //}
        for (int k = 0; k < 2; ++k)
        {
            //inputCapture[k].retrieve(view[k]);
            inputCapture[k] >> view[k];

            if (imageSize.width == 0) {
                imageSize = view[0].size();
                sf = 600./MAX(imageSize.width, imageSize.height);
                w = cvRound(imageSize.width*sf);
                h = cvRound(imageSize.height*sf);
                canvas.create(h, w*2, CV_8UC3);
            }
            Mat canvasPart = canvas(Rect(w*k, 0, w, h));
            resize(view[k], canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
        }
        //cout << "--Done\n";
        imshow("View", canvas);

        key = (char)waitKey(50);
        if( key  == ESC_KEY )
            break;

    }

}
