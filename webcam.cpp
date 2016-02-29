#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;

int main( int argc, char** argv )
{
    char* imageName = (char*) "img/test.jpg";
    Mat image;
    image = imread( imageName, 1 );
    if( !image.data )
    {
        printf( " No image data \n " );
        return -1;
    }
    Mat gray_image;
    cvtColor( image, gray_image, COLOR_BGR2GRAY );
    imwrite( "img/test_gray.jpg", gray_image );
    namedWindow( imageName, WINDOW_AUTOSIZE );
    namedWindow( "Gray image", WINDOW_AUTOSIZE );
    imshow( imageName, image );
    imshow( "Gray image", gray_image );
    waitKey(0);
    return 0;
}
