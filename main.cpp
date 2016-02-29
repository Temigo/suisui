#include <stdio.h>
#include <iostream>
#include <list>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/background_segm.hpp>
#include <stdexcept>
#include <vector>
// OpenCV 2.4.9
using namespace std;
using namespace cv;

//char* VIDEO_PATH = (char*) "videos/#29 UAAP Mens 100 Fly.mp4";
char* VIDEO_PATH = (char*) "videos/Vidéo(1).MOV";
/*
Mat detect_lines(Mat img) {
    FastFeatureDetector detector(400);
    std::vector<KeyPoint> keypoints;
    detector.detect(img, keypoints);
    Mat img_keypoints;
    drawKeypoints(img, keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    return img_keypoints;
}

Mat detect_corners(Mat img) {
    Mat gray_img;
    cvtColor( img, gray_img, COLOR_BGR2GRAY );
    Mat output;

    Ptr<LineSegmentDetector> ls = createLineSegmentDetector();

    std::vector<Vec4f> lines_std;
    ls.detect(gray_img, lines_std);
    Mat output(gray_img);
    ls.drawSegments(output, lines_std);
    return output;
}
*/

Mat toGray(Mat frame) {
    Mat gray_frame;
    cvtColor( frame, gray_frame, COLOR_BGR2GRAY );
    return gray_frame;
}

Mat detect_lines(Mat frame) {
    Mat edge, draw;
    //imshow("SuiSui", gray_frame);
    Canny(toGray(frame), edge, 50, 150, 3);
    edge.convertTo(draw, CV_8U);
    return draw;
}

Mat detect_corners(Mat frame) {
    int threshold = 200;
    Mat res = Mat::zeros( frame.size(), CV_32FC1 );
    cornerHarris(toGray(frame), res, 2, 3, 0.04);

    // Normalizing
    Mat dst;
    normalize( res, dst, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    convertScaleAbs( dst, dst);

    // Drawing a circle around corners
    for( int j = 0; j < dst.rows ; j++ )
    { for( int i = 0; i < dst.cols; i++ )
    {
        if( (int) dst.at<float>(j,i) > threshold )
        {
           circle( dst, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
        }
    }
    }

    return dst;
}

Mat extract_background(Mat frame, Ptr<BackgroundSubtractor> pMOG2) {
    Mat blurred_frame;
    //GaussianBlur(frame, blurred_frame, Size(5, 5), 0, 0);
    blur(frame, blurred_frame, Size(5, 5), Point(-1, -1));

    Mat fgMaskMOG2;
    pMOG2->apply(blurred_frame, fgMaskMOG2);
    return fgMaskMOG2;
}

Mat detect_blobs(Mat draw) {
    vector< vector<Point> > outputs;
    vector<Vec4i> hierarchy;

    findContours(draw, outputs, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE );
    list<int> largestComp;
    double maxArea = 60;
    for(int idx = 0 ; idx >= 0; idx = hierarchy[idx][0] ) {
        const vector<Point>& c = outputs[idx];
        //cout << c << endl;
        double area = fabs(contourArea(Mat(c)));
        if( area > maxArea ) {
            //maxArea = area;
            largestComp.push_back(idx);
        }
    }

    Mat dst = Mat::zeros(draw.size(), CV_8UC3);
    Scalar color( 0, 0, 255 );

    list<int>::iterator i;
    for (i=largestComp.begin(); i != largestComp.end(); i++) {
        if (outputs[*i].size() >= 5) {
            RotatedRect box = fitEllipse(outputs[*i]);
            if (box.size.width > 10) {
                ellipse(dst, box, Scalar(0,255,0), 1, LINE_AA);
                drawContours(dst, outputs, *i, color, FILLED, LINE_8, hierarchy);
            }
        }
    }

    return dst;
}

int open_video() {
    //VideoCapture cap(0);
    VideoCapture capture;
    capture.open(VIDEO_PATH);
    double debut = 57000; // Offset
    Ptr<BackgroundSubtractor> pMOG2;
    pMOG2 = createBackgroundSubtractorMOG2(100, 50, true);

    Ptr<BackgroundSubtractor> pMOG;
    pMOG = createBackgroundSubtractorKNN();

    if(!capture.isOpened()) {
        throw std::runtime_error("Erreur dans l'ouverture du fichier.");
    }

    //capture.set(CV_CAP_PROP_POS_MSEC, debut);
    Mat frame;
    namedWindow("SuiSui", 0);

    for (;;) {
        capture >> frame;
        //printf("frame");
        if (frame.empty())
            break;
        // Work with frame
        //Mat draw = detect_lines(frame);
        //Mat draw = detect_corners(frame);

        // State : position, velocity, acceleration
        KalmanFilter KF(4, 2, 0);
        KF.transitionMatrix = Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1;
        Mat_<float> measurement(2,1); measurement.setTo(Scalar(0));

        Mat draw = extract_background(frame, pMOG2);
        Mat dst = detect_blobs(draw);

        imshow("Original", frame);
        imshow("SuiSui", dst);
        waitKey(30);
    }
    waitKey(0);
    return 0;
}



int main( int argc, char** argv )
{
    return open_video();
}
