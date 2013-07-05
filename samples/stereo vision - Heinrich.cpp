#include "cv.h"
#include "cxmisc.h"
#include "highgui.h"
//#include "cvaux.h"
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <ctype.h>
#define WIDTH 426
#define HEIGHT 320


using namespace std;
int main()
{
	//---------Initial--------
	int  nx=9, ny=6, frame = 0, n_boards =20, N;
	int count1 = 0,count2 = 0, result1=0, result2=0;	
    int showUndistorted = 1, successes1 = 0,successes2 = 0 ;
   	const int maxScale = 1;
	const float squareSize = 1.f;		//Set this to your actual square size
	CvSize imageSize = {WIDTH,HEIGHT};
	CvCapture *capture1= NULL, *capture2= NULL;
	CvSize board_sz = cvSize( nx,ny );
	
	int i, j, n = nx*ny, N1 = 0, N2 = 0;
    vector<CvPoint2D32f> points[2];
	vector<int> npoints;
	vector<CvPoint3D32f> objectPoints;
	vector<CvPoint2D32f> temp1(n); 
	vector<CvPoint2D32f> temp2(n);
    
    double M1[3][3], M2[3][3], D1[5], D2[5];
    double R[3][3], T[3], E[3][3], F[3][3];
	double Q[4][4];
	CvMat _Q = cvMat(4,4, CV_64F, Q);
    CvMat _M1 = cvMat(3, 3, CV_64F, M1 );
    CvMat _M2 = cvMat(3, 3, CV_64F, M2 );
    CvMat _D1 = cvMat(1, 5, CV_64F, D1 );
    CvMat _D2 = cvMat(1, 5, CV_64F, D2 );
    CvMat _R = cvMat(3, 3, CV_64F, R );
    CvMat _T = cvMat(3, 1, CV_64F, T );
    CvMat _E = cvMat(3, 3, CV_64F, E );
    CvMat _F = cvMat(3, 3, CV_64F, F );
	
	//---------Starting WebCam----------
		capture1= cvCaptureFromCAM(1);
		assert(capture1!=NULL); cvWaitKey(100);
		capture2= cvCaptureFromCAM(2);
		assert(capture2!=NULL);
		
	//assure capture size is correct...
	int res=cvSetCaptureProperty(capture1,CV_CAP_PROP_FRAME_WIDTH,WIDTH);
	printf("%d",res);
	res=cvSetCaptureProperty(capture1,CV_CAP_PROP_FRAME_HEIGHT,HEIGHT);
	printf("%d",res);
	res=cvSetCaptureProperty(capture2,CV_CAP_PROP_FRAME_WIDTH,WIDTH);
	printf("%d",res);
	res=cvSetCaptureProperty(capture2,CV_CAP_PROP_FRAME_HEIGHT,HEIGHT);
	printf("%d",res); fflush(stdout); 
	
	
		IplImage *frame1 = cvQueryFrame( capture1 );
		IplImage* gray_fr1 = cvCreateImage( cvGetSize(frame1), 8, 1 );
		IplImage *frame2 = cvQueryFrame( capture2 );
		IplImage* gray_fr2 = cvCreateImage( cvGetSize(frame1), 8, 1 );
		//imageSize = cvGetSize(frame1);
		
	//Show Window	
		cvNamedWindow( "camera2", 1 );
		cvNamedWindow( "camera1", 1 );
		cvNamedWindow("corners camera1",1);
		cvNamedWindow("corners camera2",1);		
		while((successes1<n_boards)||(successes2<n_boards))						
	{
	    	
	//--------Find chessboard corner--------------------------------------------------	
			
		if((frame++ % 20) == 0)	
		{
			//----------------CAM1-------------------------------------------------------------------------------------------------------
			result1 = cvFindChessboardCorners( frame1, board_sz,&temp1[0], &count1,CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_FILTER_QUADS);
			cvCvtColor( frame1, gray_fr1, CV_BGR2GRAY );
			

			//----------------CAM2--------------------------------------------------------------------------------------------------------
			result2 = cvFindChessboardCorners( frame2, board_sz,&temp2[0], &count2,CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_FILTER_QUADS);
			cvCvtColor( frame2, gray_fr2, CV_BGR2GRAY );
			
		
			if(count1==n&&count2==n&&result1&&result2)
			{
					cvFindCornerSubPix( gray_fr1, &temp1[0], count1,cvSize(11, 11), cvSize(-1,-1),cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,30, 0.01) );
					cvDrawChessboardCorners( frame1, board_sz, &temp1[0], count1, result1 );
					cvShowImage( "corners camera1", frame1 );
					N1 = points[0].size();
					points[0].resize(N1 + n, cvPoint2D32f(0,0));
					copy( temp1.begin(), temp1.end(), points[0].begin() + N1 );
					++successes1;
					
					cvFindCornerSubPix( gray_fr2, &temp2[0], count2,cvSize(11, 11), cvSize(-1,-1),cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,30, 0.01) );
					cvDrawChessboardCorners( frame2, board_sz, &temp2[0], count2, result2 );
					cvShowImage( "corners camera2", frame2 );
					N2 = points[1].size();
					points[1].resize(N2 + n, cvPoint2D32f(0,0));
					copy( temp2.begin(), temp2.end(), points[1].begin() + N2 );
					++successes2;

					putchar('$');
			}
			
			else
			{		cvShowImage( "corners camera2", gray_fr2 );	
					cvShowImage( "corners camera1", gray_fr1 );	
			}
				
			frame1 = cvQueryFrame( capture1 );
			cvShowImage("camera1", frame1);
			frame2 = cvQueryFrame( capture2 );
			cvShowImage("camera2", frame2);
			
			if(cvWaitKey(15)==27) break;
		}
	}
		cvReleaseCapture( &capture1 ); 
		cvReleaseCapture( &capture2 );
		cvDestroyWindow("camera1");
		cvDestroyWindow("camera2");
		cvDestroyWindow("corners camera1");
		cvDestroyWindow("corners camera2");	
		printf("\n");
		
		
		//--------------Calibaration-------------------
		N = n_boards*n;
		objectPoints.resize(N);
		for( i = 0; i < ny; i++ )
			for(j = 0; j < nx; j++ )   objectPoints[i*nx + j] = cvPoint3D32f(i*squareSize, j*squareSize, 0);
		for( i = 1; i < n_boards; i++ ) copy( objectPoints.begin(), objectPoints.begin() + n, objectPoints.begin() + i*n );
		npoints.resize(n_boards,n);
		
		CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0] );
		CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
		CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
		CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0] );
		cvSetIdentity(&_M1);
		cvSetIdentity(&_M2);
		cvZero(&_D1);
		cvZero(&_D2);
		
		printf("Running stereo calibration ...");
		fflush(stdout);
		cvStereoCalibrate( &_objectPoints, &_imagePoints1, &_imagePoints2, &_npoints,&_M1, &_D1, &_M2, &_D2,imageSize, &_R, &_T, &_E, &_F,
		cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
        CV_CALIB_FIX_ASPECT_RATIO+CV_CALIB_ZERO_TANGENT_DIST + CV_CALIB_SAME_FOCAL_LENGTH );
		printf("done\n");
		//-------------Undistort------------------------------------------
		cvUndistortPoints( &_imagePoints1, &_imagePoints1,&_M1, &_D1, 0, &_M1 );
		cvUndistortPoints( &_imagePoints2, &_imagePoints2,&_M2, &_D2, 0, &_M2 );
		
		//--------Using bouguet algorithm-------------------
		CvMat* mx1 = cvCreateMat( imageSize.height,imageSize.width, CV_32F );
        CvMat* my1 = cvCreateMat( imageSize.height,imageSize.width, CV_32F );
        CvMat* mx2 = cvCreateMat( imageSize.height,imageSize.width, CV_32F );
        CvMat* my2 = cvCreateMat( imageSize.height,imageSize.width, CV_32F );
        CvMat* frame1r = cvCreateMat( imageSize.height,imageSize.width, CV_8U );
        CvMat* frame2r = cvCreateMat( imageSize.height,imageSize.width, CV_8U );
        CvMat* disp = cvCreateMat( imageSize.height, imageSize.width, CV_16S );
        CvMat* vdisp = cvCreateMat( imageSize.height,imageSize.width, CV_8U );
		CvMat* Image3D = cvCreateMat(imageSize.height, imageSize.width, CV_32FC3);	
        CvMat* pair;
        double R1[3][3], R2[3][3], P1[3][4], P2[3][4];
        CvMat _R1 = cvMat(3, 3, CV_64F, R1);
        CvMat _R2 = cvMat(3, 3, CV_64F, R2);
		//Calib with Bouguet algrithm
            CvMat _P1 = cvMat(3, 4, CV_64F, P1);
            CvMat _P2 = cvMat(3, 4, CV_64F, P2);
            cvStereoRectify( &_M1, &_M2, &_D1, &_D2, imageSize,&_R, &_T,&_R1, &_R2, &_P1, &_P2, &_Q,0/*CV_CALIB_ZERO_DISPARITY*/ );
        //Find matrix for cvRemap()
            cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_P1,mx1,my1);
            cvInitUndistortRectifyMap(&_M2,&_D2,&_R2,&_P2,mx2,my2);
			
		
            pair = cvCreateMat( imageSize.height, imageSize.width*2,CV_8UC3 );
			//Paramater for stereo corrrespondences
		CvStereoBMState *BMState = cvCreateStereoBMState();
        assert(BMState != 0);
        BMState->preFilterSize=31;
        BMState->preFilterCap=31;
        BMState->SADWindowSize=35;
        BMState->minDisparity= 0;
        BMState->numberOfDisparities=48;	
        BMState->textureThreshold=20;		//reduce noise 
        BMState->uniquenessRatio=15;		// uniquenessRatio > (match_val–min_match)/min_match.
		/*	CvStereoBMState *state = cvCreateStereoBMState(CV_STEREO_BM_BASIC);
			BMState->speckleRange = 50;
			BMState->textureThreshold = 400;*/
			
		//Bat camera va hien thi
			//cvNamedWindow( "camera2", 1 );
			//cvNamedWindow( "camera1", 1 );
			cvNamedWindow( "rectified",1 );
			cvNamedWindow( "disparity",1);
			cvNamedWindow("depthmap",1);
		
		capture1= cvCaptureFromCAM(1);
		assert(capture1!=NULL); cvWaitKey(100);
		capture2= cvCaptureFromCAM(2);
		assert(capture2!=NULL);
			res=cvSetCaptureProperty(capture1,CV_CAP_PROP_FRAME_WIDTH,WIDTH);
			printf("%d",res);
			res=cvSetCaptureProperty(capture1,CV_CAP_PROP_FRAME_HEIGHT,HEIGHT);
			printf("%d",res);
			res=cvSetCaptureProperty(capture2,CV_CAP_PROP_FRAME_WIDTH,WIDTH);
			printf("%d",res);
			res=cvSetCaptureProperty(capture2,CV_CAP_PROP_FRAME_HEIGHT,HEIGHT);
			printf("%d",res); fflush(stdout); 
			
			frame1 = cvQueryFrame( capture1 );
			frame2 = cvQueryFrame( capture2 );
		

			while(1)
			{
					CvMat part;
					cvCvtColor( frame1, gray_fr1, CV_BGR2GRAY );
					cvCvtColor( frame2, gray_fr2, CV_BGR2GRAY );
					cvRemap( gray_fr1, frame1r, mx1, my1 );
					cvRemap( gray_fr2, frame2r, mx2, my2 );
                    cvFindStereoCorrespondenceBM( frame1r, frame2r, disp, BMState);
					
			/*		cvShowImage("camera1", frame1);
					cvShowImage("camera2", frame2);			*/		
			//		cvConvertScale( disp, disp, 16, 0 );
					cvNormalize( disp, vdisp, 0, 256, CV_MINMAX );                   
                    cvShowImage( "disparity", vdisp );				
					cvReprojectImageTo3D(disp, Image3D, &_Q);				
					cvShowImage("depthmap",Image3D);
					
					

                
				//Hien thi anh da rectify
					cvGetCols( pair, &part, 0, imageSize.width );
                    cvCvtColor( frame1r, &part, CV_GRAY2BGR );
                    cvGetCols( pair, &part, imageSize.width, imageSize.width*2 );
                    cvCvtColor( frame2r, &part, CV_GRAY2BGR ); //CV_GRAY2BGR
                    for( j = 0; j < imageSize.height; j += 16 )
								cvLine( pair, cvPoint(0,j), cvPoint(imageSize.width*2,j), CV_RGB(0,255,0));          			
					cvShowImage( "rectified", pair );		
					frame1 = cvQueryFrame( capture1 );
					frame2 = cvQueryFrame( capture2 );
					if( cvWaitKey(15) == 27 )  break;            	
			}	
        
        cvReleaseStereoBMState(&BMState);
        cvReleaseMat( &mx1 );
        cvReleaseMat( &my1 );
        cvReleaseMat( &mx2 );
        cvReleaseMat( &my2 );	
		cvReleaseCapture( &capture1 ); 
		cvReleaseCapture( &capture2 );
        cvReleaseMat( &frame1r );
        cvReleaseMat( &frame2r );
        cvReleaseMat( &disp );
		cvReleaseMat(&Image3D);

}