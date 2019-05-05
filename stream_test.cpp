//
// Created by Karter Krueger on 2019-05-02.
//

//***** OPENCV *****
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>

int main() {

    cv::VideoCapture cap(0);
    // Check if camera opened successfully
    if (!cap.isOpened()) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    int count = 0;

    while (1) {
        cv::Mat frame;
        // Capture frame-by-frame
        cap >> frame;

        // If the frame is empty, break immediately
        if (frame.empty())
            break;

        cv::imshow("Input", frame);

        // Press  ESC on keyboard to exit
        char c = (char) cv::waitKey(25);
        if (c == 27)
            break;
    }

    // When everything is done, release the video capture object
    cap.release();

    // Closes all the frames
    cv::destroyAllWindows();

    return 0;
}