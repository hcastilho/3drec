
/**
*       @file main.c
*       @brief Gtk Application to tune the parameters of OpenCV StereoBM.
*       @author Martin Peris (http://www.martinperis.com)
*       @date 21/08/2011
*/

/*
    SSC32.cpp - Control Lynxmotion's SSC-32 V2 with C++
    Copyright (c) 2011 Martin Peris (http://www.martinperis.com).
    All right reserved.
    
    This application is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.
    
    This application is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with this application; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

//#include <opencv/cv.h>
//#include <opencv/highgui.h>
#include <stdio.h> 
#include <gtk/gtk.h>
#include <string.h>
#include <iostream>

//#include <opencv/cv.h>
//#include <opencv/highgui.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/contrib/contrib.hpp"


using namespace cv;
using namespace std;

/* Main data structure definition */
typedef struct _ChData ChData;
struct _ChData
{
    /* Widgets */
    GtkWidget *main_window;  /* Main application window */
    GtkImage *image_left;
    GtkImage *image_right;
    GtkImage *image_depth;

    /* OpenCV */
    //CvStereoBMState *BMState; /* Block Matching State */
    //IplImage *cv_image_left;
    //IplImage *cv_image_right;
    //CvMat *cv_image_depth;
    //IplImage *cv_image_depth_aux;
    StereoBM sbm;
    //Mat *imageLeft;
    //Mat *imageRight;
    Mat view[2];
    Mat imageDepth;
    //Mat *imageDepthAux;

};
volatile ChData data;

/* Function to compute StereoBM and update the result on the window */
void computeStereoBM ( ChData *Oata )
{
    int i, j, aux;
    GdkPixbuf *pix;
    IplImage *img;
    uchar *ptr_dst;
    //data.sbm->operator()(*(data.view)[0], *(data.view)[1], *data.imageDepth);
    cout << "Here A" << endl;
    data.sbm(data.view[0], data.view[1], data.imageDepth);
    cout << "Here B" << endl;
    pix = gdk_pixbuf_new_from_data (
        data.imageDepth.data,
        GDK_COLORSPACE_RGB,
        FALSE,
        8,
        data.imageDepth.cols,
        data.imageDepth.rows,
        data.imageDepth.step,
        NULL,
        NULL
    );
    //Normalize the result so we can display it
    // TODO
    //cvNormalize( data.imageDepth, data.imageDepth, 0, 256, CV_MINMAX, NULL );
    //for ( i = 0; i < data.imageDepth->rows; i++)
    //{
    //    aux = data.imageDepth->cols * i;
    //    ptr_dst = (uchar*)(data.imageDepthAux->imageData + i*data.imageDepthAux->widthStep);
    //    for ( j = 0; j < data.imageDepth->cols; j++ )
    //    {
    //        //((float*)(mat->data.ptr + mat->step*i))[j]
    //        ptr_dst[3*j] = (uchar)((short int*)(data.imageDepth->data.ptr + data.imageDepth->step*i))[j];
    //        ptr_dst[3*j+1] = (uchar)((short int*)(data.imageDepth->data.ptr + data.imageDepth->step*i))[j];
    //        ptr_dst[3*j+2] = (uchar)((short int*)(data.imageDepth->data.ptr + data.imageDepth->step*i))[j];
    //    }
    //}
    
    //Transform IplImage to GtkImage
    //img = data.imageDepthAux;
    //pix = gdk_pixbuf_new_from_data (
    //    (guchar*)img->imageData, 
    //    GDK_COLORSPACE_RGB, 
    //    FALSE, 
    //    img->depth, 
    //    img->width, 
    //    img->height, 
    //    (img->widthStep), 
    //    NULL, 
    //    NULL
    //);
    //Update the depth image on the window
    cout << "Here C" << endl;
    gtk_image_set_from_pixbuf(data.image_depth, pix); 
    
}


/* Define callbacks */

//Callback for adjustment preFilterSize
G_MODULE_EXPORT void on_adjustment1_value_changed( GtkAdjustment *adjustment, ChData *Oata )
{
    gint value;

    //if (data == NULL) {
    //    fprintf(stderr,"WARNING: data is null\n");
    //    return;
    //}

    value = (gint) gtk_adjustment_get_value( adjustment );

    //the value must be odd, if it is not then set it to the next odd value
    if (value % 2 == 0) 
    {
        value += 1;
        gtk_adjustment_set_value( adjustment, (gdouble)value);
        return;
    }

    //set the parameter, 
    data.sbm.state->preFilterSize = value;
    computeStereoBM( data );
}

//Callback for adjustment preFilterCap
G_MODULE_EXPORT void on_adjustment2_value_changed( GtkAdjustment *adjustment, ChData *Oata )
{
    gint value;

    //if (data == NULL) {
    //    fprintf(stderr,"WARNING: data is null\n");
    //    return;
    //}

    value = (gint) gtk_adjustment_get_value ( adjustment );

    //set the parameter
    data.sbm.state->preFilterCap = value;
    computeStereoBM( data );
}

//Callback for adjustment SADWindowSize
G_MODULE_EXPORT void on_adjustment3_value_changed( GtkAdjustment *adjustment, ChData *Oata )
{
    gint value;

    //if (data == NULL) {
    //    fprintf(stderr,"WARNING: data is null\n");
    //    return;
    //}

    value = (gint) gtk_adjustment_get_value( adjustment );

    //the value must be odd, if it is not then set it to the next odd value
    if (value % 2 == 0) 
    {
        value += 1;
        gtk_adjustment_set_value( adjustment, (gdouble)value);
        return;
    }

    //the value must be smaller than the image size
    ///if ( value >= (*(data.view)[0]).size().width || value >= (*(data.view)[1]).size().height)
    cout << "Here DANGER!!!" << endl;
    if ( value >= (*(data.view)).size().width || value >= (*(data.view + 1)).size().height)
    {
        fprintf(stderr,"WARNING: SADWindowSize larger than image size\n");
        return;
    }

    //set the parameter, 
    data.sbm.state->SADWindowSize = value;
    computeStereoBM( data );

}

//Callback for adjustment minDisparity
G_MODULE_EXPORT void on_adjustment4_value_changed( GtkAdjustment *adjustment, ChData *Oata )
{
    gint value;

    //if (data == NULL) {
    //    fprintf(stderr,"WARNING: data is null\n");
    //    return;
    //}

    value = (gint) gtk_adjustment_get_value( adjustment );

    data.sbm.state->minDisparity = value;
    computeStereoBM( data );
}

//Callback for adjustment numberOfDisparities
G_MODULE_EXPORT void on_adjustment5_value_changed( GtkAdjustment *adjustment, ChData *Oata )
{
    gint value;

    //if (data == NULL) {
    //    fprintf(stderr,"WARNING: data is null\n");
    //    return;
    //}

    value = (gint) gtk_adjustment_get_value( adjustment );

    //te value must be divisible by 16, if it is not set it to the nearest multiple of 16
    if (value % 16 != 0)
    {
        value += (16 - value%16);
        gtk_adjustment_set_value( adjustment, (gdouble)value);
        return;
    }

    data.sbm.state->numberOfDisparities = value;
    computeStereoBM( data );
}

//Callback for adjustment textureThreshold
G_MODULE_EXPORT void on_adjustment6_value_changed( GtkAdjustment *adjustment, ChData *Oata )
{
    gint value;

    //if (data == NULL) {
    //    fprintf(stderr,"WARNING: data is null\n");
    //    return;
    //}

    value = (gint) gtk_adjustment_get_value( adjustment );

    data.sbm.state->textureThreshold = value;
    computeStereoBM( data );
}

//Callback for adjustment uniquenessRatio
G_MODULE_EXPORT void on_adjustment7_value_changed( GtkAdjustment *adjustment, ChData *Oata )
{
    gint value;

    //if (data == NULL) {
    //    fprintf(stderr,"WARNING: data is null\n");
    //    return;
    //}

    value = (gint) gtk_adjustment_get_value( adjustment );

    data.sbm.state->uniquenessRatio = value;
    computeStereoBM( data );
}

//Callback for adjustment speckleWindowSize
G_MODULE_EXPORT void on_adjustment8_value_changed( GtkAdjustment *adjustment, ChData *Oata )
{
    gint value;

    if (data == NULL) {
        fprintf(stderr,"WARNING: data is null\n");
        return;
    }

    value = (gint) gtk_adjustment_get_value( adjustment );

    data.sbm.state->speckleWindowSize = value;
    computeStereoBM( data );
}

//Callback for adjustment speckleRange
G_MODULE_EXPORT void on_adjustment9_value_changed( GtkAdjustment *adjustment, ChData *Oata )
{
    gint value;

    //if (data == NULL) {
    //    fprintf(stderr,"WARNING: data is null\n");
    //    return;
    //}

    value = (gint) gtk_adjustment_get_value( adjustment );

    data.sbm.state->speckleRange = value;
    computeStereoBM( data );
}


int
main( int    argc,
      char **argv )
{
    GtkBuilder *builder;
    GError *error = NULL;
    //ChData *data;

    //data.imageDepth = new Mat;
    //Mat a[2];
    //Mat aaa;
    //*(data.view)[0] = aaa;
    //data.view = new Mat[2];
    //data.sbm = new StereoBM;
    //TODO: Get all the BM Default parameters from the GUI definition
    cout << "START" << endl;
    data.sbm.state->preFilterSize        = 5;
    data.sbm.state->preFilterCap         = 1;
    data.sbm.state->SADWindowSize        = 5;
    data.sbm.state->minDisparity         = 0;
    data.sbm.state->numberOfDisparities  = 64;
    data.sbm.state->textureThreshold     = 0;
    data.sbm.state->uniquenessRatio  = 0;
    data.sbm.state->speckleWindowSize    = 0;
    data.sbm.state->speckleRange     = 0;

    int cameraId[2] = {1,0};
    FileStorage fs("intrinsics.yml", CV_STORAGE_READ);
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

    fs.open("extrinsics.yml", CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        cout << "Failed to open extrinsics file ";
        return -1;
    }

    Mat R, T;
    Mat R1, R2, P1, P2, Q;
    fs["R"] >> R;
    fs["T"] >> T;

    Rect roi1, roi2;
    stereoRectify(M1, D1, M2, D2, imageSize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, imageSize, &roi1, &roi2);
    data.sbm.state->roi1 = roi1;
    data.sbm.state->roi2 = roi2;

    Mat map[2][2];
    initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2, map[0][0], map[0][1]);
    initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_16SC2, map[1][0], map[1][1]);


    //Mat disp, disp8;
    VideoCapture inputCapture[2];
    int i, k;
    char key;
    for (k = 0; k < 2; ++k)
    {
        inputCapture[k].open(cameraId[k]);
        if (!inputCapture[k].isOpened()) {
            cerr << "Inexistent input: " << cameraId[k];
            return 1;
        }

    }
    for( k = 0; k < 2; k++ )
    {
        inputCapture[k].grab();
    }
    for( k = 0; k < 2; k++ )
    {
        Mat aux;
        cout << "ST1" << endl;
        //inputCapture[k].retrieve(*(data.view)[k]);
        inputCapture[k].retrieve(data.view[k]);
        cout << "ST2" << endl;
        //remap(*(data.view)[k], aux, map[k][0], map[k][1], INTER_LINEAR);
        remap(data.view[k], aux, map[k][0], map[k][1], INTER_LINEAR);
        cout << "ST3" << endl;
        //cvtColor(aux, *(data.view)[k], CV_BGR2GRAY);
        cvtColor(aux, data.view[k], CV_BGR2GRAY);
        cout << "ST4" << endl;
    }
    cout << "START 5" << endl;

    /* Init GTK+ */
    gtk_init( &argc, &argv );

    /* Create data */
    ChData *Oata;
    Oata = g_slice_new(ChData);

    //data.sbm = new StereoBM;
    //data.BMState = cvCreateStereoBMState(CV_STEREO_BM_BASIC, 64);

    /* Create new GtkBuilder object */
    builder = gtk_builder_new();

    /* Load UI from file. If error occurs, report it and quit application.
     * Replace "tut.glade" with your saved project. */
    if( ! gtk_builder_add_from_file( builder, "StereoBMTuner.glade", &error ) )
    {
        g_warning( "%s", error->message );
        g_free( error );
        return( 1 );
    }

    cout << "Here 0" << endl;
    /* Get main window pointer from UI */
    data.main_window = GTK_WIDGET( gtk_builder_get_object( builder, "window1" ) );
    data.image_left = GTK_IMAGE( gtk_builder_get_object( builder, "image_left" ) );
    data.image_right = GTK_IMAGE( gtk_builder_get_object( builder, "image_right" ) );
    data.image_depth = GTK_IMAGE( gtk_builder_get_object( builder, "image_disparity" ) );

    cout << "Here 1" << endl;
    //Put images on place
    // TODO
    //gtk_image_set_from_file ( data.image_left, left_filename );
    //gtk_image_set_from_file ( data.image_right, right_filename );

    cout << "Here 2" << endl;
    /* Execute first iteration */
    computeStereoBM ( Oata );

    cout << "Here 3" << endl;
    /* Connect signals */
    gtk_builder_connect_signals( builder, Oata );

    cout << "Here 4" << endl;
    /* Destroy builder, since we don't need it anymore */
    g_object_unref( G_OBJECT( builder ) );

    /* Show window. All other widgets are automatically shown by GtkBuilder */
    gtk_widget_show( data.main_window );

    /* Start main loop */
    gtk_main();

    return( 0 );
}

