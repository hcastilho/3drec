
//#include <opencv/cv.h>
//#include <opencv/highgui.h>
#include <stdio.h> 
//#include <gtk/gtk.h>
#include <string.h>
#include <iostream>
//#include <thread>
//#include <mutex>
#include <gtkmm.h>
//#include <glibmm/threads.h>

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
    Gtk::Window *main_window;  /* Main application window */
    Gtk::Image *image_left;
    Gtk::Image *image_right;
    Gtk::Image *image_depth;

    /* OpenCV */
    StereoBM sbm;
    Mat view[2];
    Mat viewDisplay[2];
    Mat imageDepth;
    Mat imageDepthDisplay;
    Mat map[2][2];

    /* GTK */
    Glib::RefPtr<Gdk::Pixbuf> pix_left;
    Glib::RefPtr<Gdk::Pixbuf> pix_right;
    Glib::RefPtr<Gdk::Pixbuf> pix_depth;
};
ChData data;

volatile bool threadRun;
Glib::Threads::Thread *theThread;
Glib::Dispatcher theDispatcher;
Glib::Threads::Mutex theMutex;

/* Function to compute StereoBM and update the result on the window */
void computeStereoBM ()
{
    Mat gray[2];
    cvtColor(data.view[0], gray[0], CV_BGR2GRAY);
    cvtColor(data.view[1], gray[1], CV_BGR2GRAY);
    data.sbm(gray[0], gray[1], data.imageDepth);

    Mat imageDepthDisplay;
    // Convert to CV_8U to display
    data.imageDepth.convertTo(imageDepthDisplay, CV_8U);

    // Repeat and merge to display in gtk
    Mat rgb;
    vector<Mat> mergeMat;
    mergeMat.resize(3);
    for(int k = 0; k < 3; k++ )
    {
        mergeMat[k] = imageDepthDisplay;
    }
    merge(mergeMat, data.imageDepthDisplay);
}


/* Define callbacks */

//Callback for adjustment preFilterSize
void on_adjustment1_value_changed(Glib::RefPtr<Gtk::Adjustment> adjustment)
{
    gint value;
    value = (gint) adjustment->get_value();

    //the value must be odd, if it is not then set it to the next odd value
    if (value % 2 == 0) 
    {
        value += 1;
        adjustment->set_value((gdouble)value);
        return;
    }
    {
    Glib::Threads::Mutex::Lock lock (theMutex);
    data.sbm.state->preFilterSize = value;
    }
}

//Callback for adjustment preFilterCap
void on_adjustment2_value_changed(Glib::RefPtr<Gtk::Adjustment> adjustment)
{
    gint value;
    value = (gint) adjustment->get_value();

    //set the parameter
    {
    Glib::Threads::Mutex::Lock lock (theMutex);
    data.sbm.state->preFilterCap = value;
    }
}

//Callback for adjustment SADWindowSize
void on_adjustment3_value_changed(Glib::RefPtr<Gtk::Adjustment> adjustment)
{
    gint value;
    value = (gint) adjustment->get_value();

    //the value must be odd, if it is not then set it to the next odd value
    if (value % 2 == 0) 
    {
        value += 1;
        adjustment->set_value((gdouble)value);
        return;
    }

    //the value must be smaller than the image size
    ///if ( value >= (*(data.view)[0]).size().width || value >= (*(data.view)[1]).size().height)
    //cout << "Here DANGER!!!" << endl;
    if ( value >= data.view[0].cols || value >= data.view[1].rows)
    {
        fprintf(stderr,"WARNING: SADWindowSize larger than image size\n");
        return;
    }

    //set the parameter,
    {
    Glib::Threads::Mutex::Lock lock (theMutex);
    data.sbm.state->SADWindowSize = value;
    }

}

//Callback for adjustment minDisparity
void on_adjustment4_value_changed(Glib::RefPtr<Gtk::Adjustment> adjustment)
{
    gint value;
    value = (gint) adjustment->get_value();
    {
    Glib::Threads::Mutex::Lock lock (theMutex);
    data.sbm.state->minDisparity = value;
    }
}

//Callback for adjustment numberOfDisparities
void on_adjustment5_value_changed(Glib::RefPtr<Gtk::Adjustment> adjustment)
{
    gint value;
    value = (gint) adjustment->get_value();

    //te value must be divisible by 16, if it is not set it to the nearest multiple of 16
    if (value % 16 != 0)
    {
        value += (16 - value%16);
        adjustment->set_value((gdouble)value);
        return;
    }
    {
    Glib::Threads::Mutex::Lock lock (theMutex);
    data.sbm.state->numberOfDisparities = value;
    }
}

//Callback for adjustment textureThreshold
void on_adjustment6_value_changed(Glib::RefPtr<Gtk::Adjustment> adjustment)
{
    gint value;
    value = (gint) adjustment->get_value();
    {
    Glib::Threads::Mutex::Lock lock (theMutex);
    data.sbm.state->textureThreshold = value;
    }
}

//Callback for adjustment uniquenessRatio
void on_adjustment7_value_changed(Glib::RefPtr<Gtk::Adjustment> adjustment)
{
    gint value;
    value = (gint) adjustment->get_value();
    {
    Glib::Threads::Mutex::Lock lock (theMutex);
    data.sbm.state->uniquenessRatio = value;
    }
}

//Callback for adjustment speckleWindowSize
void on_adjustment8_value_changed(Glib::RefPtr<Gtk::Adjustment> adjustment)
{
    gint value;
    value = (gint) adjustment->get_value();
    {
    Glib::Threads::Mutex::Lock lock (theMutex);
    data.sbm.state->speckleWindowSize = value;
    }
}

//Callback for adjustment speckleRange
void on_adjustment9_value_changed(Glib::RefPtr<Gtk::Adjustment> adjustment)
{
    gint value;
    value = (gint) adjustment->get_value();
    {
    Glib::Threads::Mutex::Lock lock (theMutex);
    data.sbm.state->speckleRange = value;
    }
}


void thread_worker() {
    threadRun = true;

    int i, k;
    char key;
    int cameraId[2] = {1,0};
    VideoCapture inputCapture[2];
    {
    Glib::Threads::Mutex::Lock lock (theMutex);
    for (k = 0; k < 2; ++k)
    {
        inputCapture[k].open(cameraId[k]);
        if (!inputCapture[k].isOpened()) {
            cerr << "Inexistent input: " << cameraId[k];
            return;
        }
    }
    }

    while (threadRun) {
        // TODO skip processing on some frames
        Glib::usleep(50000);
        {
        Glib::Threads::Mutex::Lock lock (theMutex);

        for( k = 0; k < 2; k++ )
        {
            inputCapture[k].grab();
        }
        for( k = 0; k < 2; k++ )
        {
            Mat aux;
            inputCapture[k].retrieve(data.view[k]);
            remap(data.view[k], aux, data.map[k][0], data.map[k][1], INTER_LINEAR);
            data.view[k]=aux;
        }

        cvtColor(data.view[0], data.viewDisplay[0], CV_BGR2RGB);
        data.pix_left = Gdk::Pixbuf::create_from_data(
            data.viewDisplay[0].data,
            Gdk::COLORSPACE_RGB,
            false,
            8,
            data.viewDisplay[0].cols,
            data.viewDisplay[0].rows,
            data.viewDisplay[0].step
        );
        cvtColor(data.view[1], data.viewDisplay[1], CV_BGR2RGB);
        data.pix_right = Gdk::Pixbuf::create_from_data(
            data.viewDisplay[1].data,
            Gdk::COLORSPACE_RGB,
            false,
            8,
            data.viewDisplay[1].cols,
            data.viewDisplay[1].rows,
            data.viewDisplay[1].step
        );

        computeStereoBM();
        data.pix_depth = Gdk::Pixbuf::create_from_data(
            data.imageDepthDisplay.data,
            Gdk::COLORSPACE_RGB,
            false,
            8,
            data.imageDepthDisplay.cols,
            data.imageDepthDisplay.rows,
            data.imageDepthDisplay.step
        );

        }
        theDispatcher.emit();
    }
}

void dispatch_worker() {

    {
    Glib::Threads::Mutex::Lock lock (theMutex);

    data.image_left->set(data.pix_left);
    data.image_right->set(data.pix_right);
    data.image_depth->set(data.pix_depth);


    }
}

int main(int argc, char **argv )
{

    //TODO: Get all the BM Default parameters from the GUI definition
    data.sbm.state->preFilterSize        = 5;
    data.sbm.state->preFilterCap         = 1;
    data.sbm.state->SADWindowSize        = 5;
    data.sbm.state->minDisparity         = 0;
    data.sbm.state->numberOfDisparities  = 64;
    data.sbm.state->textureThreshold     = 0;
    data.sbm.state->uniquenessRatio  = 0;
    data.sbm.state->speckleWindowSize    = 0;
    data.sbm.state->speckleRange     = 0;

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

    initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2, data.map[0][0], data.map[0][1]);
    initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_16SC2, data.map[1][0], data.map[1][1]);


    /* Init GTK+ */
    Gtk::Main kit(argc, argv);
    /* Create new GtkBuilder object */
    Glib::RefPtr<Gtk::Builder> builder = Gtk::Builder::create_from_file("StereoBMTuner.glade");
    builder->get_widget("window1", data.main_window);
    builder->get_widget("image_left", data.image_left);
    builder->get_widget("image_right", data.image_right);
    builder->get_widget("image_disparity", data.image_depth);

    ///* Connect signals */
    Glib::RefPtr<Gtk::Adjustment> adjustment1;
    adjustment1 = Glib::RefPtr<Gtk::Adjustment>::cast_dynamic(builder->get_object("adjustment1"));
    adjustment1->signal_value_changed().connect(
            sigc::bind< Glib::RefPtr<Gtk::Adjustment> >(
                sigc::ptr_fun(&on_adjustment1_value_changed),
                adjustment1
                )
            );
    // OR TRY
    //https://developer.gnome.org/gtkmm-tutorial/2.24/sec-builder-using-derived-widgets.html.en
    Glib::RefPtr<Gtk::Adjustment> adjustment2;
    adjustment2 = Glib::RefPtr<Gtk::Adjustment>::cast_dynamic(builder->get_object("adjustment2"));
    adjustment2->signal_value_changed().connect(
            sigc::bind< Glib::RefPtr<Gtk::Adjustment> >(
                sigc::ptr_fun(&on_adjustment2_value_changed),
                adjustment2
                )
            );
    Glib::RefPtr<Gtk::Adjustment> adjustment3;
    adjustment3 = Glib::RefPtr<Gtk::Adjustment>::cast_dynamic(builder->get_object("adjustment3"));
    adjustment3->signal_value_changed().connect(
            sigc::bind< Glib::RefPtr<Gtk::Adjustment> >(
                sigc::ptr_fun(&on_adjustment3_value_changed),
                adjustment3
                )
            );
    Glib::RefPtr<Gtk::Adjustment> adjustment4;
    adjustment4 = Glib::RefPtr<Gtk::Adjustment>::cast_dynamic(builder->get_object("adjustment4"));
    adjustment4->signal_value_changed().connect(
            sigc::bind< Glib::RefPtr<Gtk::Adjustment> >(
                sigc::ptr_fun(&on_adjustment4_value_changed),
                adjustment4
                )
            );
    Glib::RefPtr<Gtk::Adjustment> adjustment5;
    adjustment5 = Glib::RefPtr<Gtk::Adjustment>::cast_dynamic(builder->get_object("adjustment5"));
    adjustment5->signal_value_changed().connect(
            sigc::bind< Glib::RefPtr<Gtk::Adjustment> >(
                sigc::ptr_fun(&on_adjustment5_value_changed),
                adjustment5
                )
            );
    Glib::RefPtr<Gtk::Adjustment> adjustment6;
    adjustment6 = Glib::RefPtr<Gtk::Adjustment>::cast_dynamic(builder->get_object("adjustment6"));
    adjustment6->signal_value_changed().connect(
            sigc::bind< Glib::RefPtr<Gtk::Adjustment> >(
                sigc::ptr_fun(&on_adjustment6_value_changed),
                adjustment6
                )
            );
    Glib::RefPtr<Gtk::Adjustment> adjustment7;
    adjustment7 = Glib::RefPtr<Gtk::Adjustment>::cast_dynamic(builder->get_object("adjustment7"));
    adjustment7->signal_value_changed().connect(
            sigc::bind< Glib::RefPtr<Gtk::Adjustment> >(
                sigc::ptr_fun(&on_adjustment7_value_changed),
                adjustment7
                )
            );
    Glib::RefPtr<Gtk::Adjustment> adjustment8;
    adjustment8 = Glib::RefPtr<Gtk::Adjustment>::cast_dynamic(builder->get_object("adjustment8"));
    adjustment8->signal_value_changed().connect(
            sigc::bind< Glib::RefPtr<Gtk::Adjustment> >(
                sigc::ptr_fun(&on_adjustment8_value_changed),
                adjustment8
                )
            );
    Glib::RefPtr<Gtk::Adjustment> adjustment9;
    adjustment9 = Glib::RefPtr<Gtk::Adjustment>::cast_dynamic(builder->get_object("adjustment9"));
    adjustment9->signal_value_changed().connect(
            sigc::bind< Glib::RefPtr<Gtk::Adjustment> >(
                sigc::ptr_fun(&on_adjustment9_value_changed),
                adjustment9
                )
            );

    theDispatcher.connect(sigc::ptr_fun(&dispatch_worker));
    theThread = Glib::Threads::Thread::create( sigc::ptr_fun(&thread_worker));
    //myThread = std::thread(&run);
    kit.run(*data.main_window);

    //return( 0 );
}

