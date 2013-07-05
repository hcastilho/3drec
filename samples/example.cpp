#include <cv.h>
#include <ml.h>
#include <cxcore.h>
#include <highgui.h>

using namespace cv;
using namespace std;

#define THRESH_VALUE 200
#define THRESH_MODE 0
#define THRESH_MAX_VALUE 255

int main()
{
    Mat original_image = imread("GreenApple.jpg");
    Mat thresh_img;

    //Show original image
    imshow("Original", original_image);

    //convert the image to gray
    cvtColor( original_image, thresh_img, CV_RGB2GRAY );

    //Threshold the image
    threshold( thresh_img, thresh_img, THRESH_VALUE, THRESH_MAX_VALUE, THRESH_MODE );

    //lets show the output
    imshow("Thresholded", thresh_img);

    //Wait until any key is pressed
    waitKey(0);
    cout << "Exiting application\n";

    return 0;
}
