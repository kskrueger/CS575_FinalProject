#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <vector>
#include <string>
#include <math.h>
#include <opencv2/imgproc.hpp>

using namespace cv;

Mat convert(Mat input, int code);

int main() {

    using namespace cv;
    int a = 400 / 8;
    // Create black empty images
    Mat image = Mat::zeros(400, 400, CV_8UC3);

    // Draw a rectangle ( 5th argument is not -ve)
    for (int y = 0; y <= 3; y++)
        for (int x = 0; x <= 3; x++) {
            rectangle(image, Point(x * a * 2, y * a * 2), Point(a * (2 * x + 1), a * (2 * y + 1)),
                      Scalar(255, 255, 255), -1, 4);
            imshow("Image1", image);
            waitKey(250);
        }
    for (int y = 0; y <= 3; y++)
        for (int x = 0; x <= 3; x++) {
            rectangle(image, Point(a * (2 * x + 1), a * (2 * y + 1)), Point((x + 1) * a * 2, (y + 1) * a * 2),
                      Scalar(255, 255, 255), -1, 4);
            imshow("Image1", image);
            waitKey(250);
        }
    waitKey(0);
    return (0);

    /*Mat image = imread("~/image.jpg");
    imshow("Image", image);

    Mat gray;
    cvtColor(image, gray, COLOR_BGR2GRAY);
    //imshow("gray", gray);

    Mat binary;
    threshold(gray, binary, 240, 255, THRESH_BINARY_INV);
    //imshow("binary", binary);


    auto se_v = getStructuringElement(MORPH_RECT, Size(1, 50));
    Mat vert;
    erode(binary, vert, se_v);
    dilate(vert, vert, se_v);
    //imshow("V", vert);

    auto se_h = getStructuringElement(MORPH_RECT, Size(50, 1));
    Mat horz;
    erode(binary, horz, se_h);
    dilate(horz, horz, se_h);
    //imshow("H", horz);

    Mat grid;
    bitwise_or(vert, horz, grid);
    imshow("All", grid);

    std::vector<std::vector<cv::Point>> contours;
    findContours(grid, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

    int area = 0;
    std::vector<cv::Point> cont;
    for (const auto &x : contours) {
        int area1 = (int)contourArea(x);
        if (area1 > area) {
            area = area1;
            cont = x;
        }
    }

    Rect rect1 = boundingRect(cont);
    Mat output = image(rect1);
    imshow("Output", output);

    waitKey(0);*/
}
