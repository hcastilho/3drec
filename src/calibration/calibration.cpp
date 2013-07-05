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
    int delay = 2000;
    bool blinkOutput = false;
    clock_t prevTimestamp = 0;
    char key;
    const float squareSize = 24.f;
    const char ESC_KEY = 27;
    //const char SPACE_KEY = 32;
    // nrFrames number of good frames to capture
    int i, j, k;
    int nrFrames = 25;
    Size boardSize = Size(9, 6);
    Size imageSize = Size(0, 0);
    int cameraId[2] = {1,0};
    VideoCapture inputCapture[2];
    vector<vector<Point2f> > imagePoints[2];
    vector<vector<Point3f> > objectPoints;

    cout << "Opening Cameras";
    for (k = 0; k < 2; ++k)
    {
        inputCapture[k].open(cameraId[k]);
        if (!inputCapture[k].isOpened()) {
            cerr << "Inexistent input: " << cameraId[k];
            return -1;
        }

    }
    cout << " [Done]\n";


    Mat canvas;
    double sf;
    int w, h;
    Mat view[2];
    bool stopCapture = false;

    cout << "Capturing Calibration Frames";
    for(i = j = 0;; ++i)
    {
        if (stopCapture)
            break;


        for (k = 0; k < 2; ++k)
        {
            inputCapture[k].grab();
        }
        for (k = 0; k < 2; ++k)
        {
            inputCapture[k].retrieve(view[k]);
            //inputCapture[k] >> view[k];

            if (imageSize.width == 0) {
                imageSize = view[0].size();
                sf = 600./MAX(imageSize.width, imageSize.height);
                w = cvRound(imageSize.width*sf);
                h = cvRound(imageSize.height*sf);
                canvas.create(h, w*2, CV_8UC3);
            }
        }

        key = (char)waitKey(50);
        if( key  == ESC_KEY )
            break;

        vector<Point2f> corners[2];
        bool found[2] = {false, false};
        for (k = 0; k < 2; ++k)
        {
            found[k] = findChessboardCorners(view[k], boardSize, corners[k],
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
            if (found[k])
            {
                Mat viewGray;
                cvtColor(view[k], viewGray, CV_BGR2GRAY);
                cornerSubPix(viewGray, corners[k], Size(11,11), Size(-1,-1),
                         TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                                      30, 0.01));
                drawChessboardCorners(view[k], boardSize, Mat(corners[k]), found[k]);
            }
            if(k == 1 && found[0] == true && found[1] == true &&
                    (clock() - prevTimestamp > delay*1e-3*CLOCKS_PER_SEC))
            {
                prevTimestamp = clock();
                blinkOutput = true;

                imagePoints[0].push_back(corners[0]);
                imagePoints[1].push_back(corners[1]);
                j++;

                if (j == nrFrames)
                {
                    stopCapture = true;
                    break;
                }
            }
        }
        for (k = 0; k < 2; ++k)
        {
            Mat canvasPart = canvas(Rect(w*k, 0, w, h));
            resize(view[k], canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
        }
        if( blinkOutput )
        {
            bitwise_not(canvas, canvas);
            blinkOutput = false;
        }
        imshow("View", canvas);
    }
    cout << " [Done]\n";

    if (j < nrFrames) {
        cerr << "Did no reach required number of frames\n";
        return -1;
    }
    cout << nrFrames << " pairs have been successfully detected.\n";

    cout << "Calibrating";
    objectPoints.resize(nrFrames);
    for (i = 0; i < nrFrames; i++)
    {
        for(j = 0; j < boardSize.height; j++)
            for(k = 0; k < boardSize.width; k++)
                objectPoints[i].push_back(Point3f(j*squareSize, k*squareSize, 0));
    }

    Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
    cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
    Mat R, T, E, F;

    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                    cameraMatrix[0], distCoeffs[0],
                    cameraMatrix[1], distCoeffs[1],
                    imageSize, R, T, E, F,
                    TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                    CV_CALIB_FIX_ASPECT_RATIO +
                    CV_CALIB_ZERO_TANGENT_DIST +
                    CV_CALIB_SAME_FOCAL_LENGTH +
                    CV_CALIB_RATIONAL_MODEL +
                    CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
    cout << "done with RMS error=" << rms << endl;


// CALIBRATION QUALITY CHECK
// because the output fundamental matrix implicitly
// includes all the output information,
// we can check the quality of calibration using the
// epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for( i = 0; i < nrFrames; i++ )
    {
        int npt = (int)imagePoints[0][i].size();
        Mat imgpt[2];
        for( k = 0; k < 2; k++ )
        {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
        for( j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    cout << "average reprojection err = " <<  err/npoints << endl;

    // save intrinsic parameters
    FileStorage fs("intrinsics.yml", CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
            "M2" << cameraMatrix[1] << "D2" << distCoeffs[1] <<
            "imageSize" << imageSize;
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";

    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];

    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

    fs.open("extrinsics.yml", CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";



    // Display
    //Precompute maps for cv::remap()
    Mat rmap[2][2];
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    for( i = 0;; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            Mat rimg;
            inputCapture[k] >> view[k];
            remap(view[k], rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
            Mat canvasPart = canvas(Rect(w*k, 0, w, h));
            resize(rimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);

            Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
                        cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
            rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);
        }

        for( j = 0; j < canvas.rows; j += 16 )
            line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
        imshow("View", canvas);
        key = (char)waitKey(50);
        if( key  == ESC_KEY )
            break;
    }
    //cout << " [Done]\n";
}
