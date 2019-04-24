//
// Created by Karter Krueger on 2019-04-23.
//

//***** OPENCV *****
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <zconf.h>

int main() {
    using namespace std;

    ifstream in("hallMap3.csv");
    double curr_y = (int) 4701.90 - 800;
    double curr_x = (int) 4016.99 - 45;
    float heading = 0;
    int firstRow = 3062;
    int firstCol = 3831;

    string line, field;

    vector<vector<string> > array;  // the 2D array
    vector<string> v;                // array of values for one line only

    while (getline(in, line))    // get next line in file
    {
        v.clear();
        stringstream ss(line);

        while (getline(ss, field, ','))  // break line into comma delimitted fields
        {
            v.push_back(field);  // add each field to the 1D array
        }

        array.push_back(v);  // add the 1D array to the 2D array
    }

    // print out what was read in
    int HEIGHT = static_cast<int>(array.size());
    int WIDTH = static_cast<int>(array[0].size());
    uint **mapArr = new uint *[HEIGHT];
    for (int i = 0; i < HEIGHT; i++) {
        mapArr[i] = new uint[WIDTH];
    }

    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            mapArr[y][x] = static_cast<uint>(atoi(array[y][x].c_str()));
        }
    }

    /*for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            cout << mapArr[y][x] << " ";
        }
        cout << endl;
    }*/

    double encoder_cm = 0;
    double last_encoders = 0;

    while (curr_y - firstRow < HEIGHT) {
        cv::Mat canvas(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(0));
        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                cv::Mat ROI = canvas(cv::Rect(x, y, 1, 1));
                cv::Scalar color = cv::Scalar::all(0);// = mapArr[y][x] >= 2 ? 255 : 0;
                int value = mapArr[y][x];
                if (value >= 7) {
                    color = cv::Scalar(255, 0, 0);
                } else if (value >= 5) {
                    color = cv::Scalar(100, 0, 0);
                } else if (value >= 1) {
                    color = cv::Scalar(50, 100, 0);
                }
                ROI.setTo(color);
            }
        }
        cv::circle(canvas, cv::Point((int) curr_x - firstCol, (int) curr_y - firstRow), 4,
                   cv::Scalar(255, 0, 0), -1);
        cv::RotatedRect rRect(cv::Point2f((int)curr_x
        -firstCol, (int) curr_y - firstRow),
        cv::Size2f(35, 45), 360 - heading);
        cv::Point2f vertices[4];
        rRect.points(vertices);
        for (int i = 0; i < 4; i++) {
            cv::line(canvas, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
        }

        cv::Mat canvas2 = canvas.clone();

        int stopDist = 55;
        int start_y = (int) curr_y - firstRow;
        int start_x = (int) curr_x - firstCol;
        for (int fwd = 0; fwd < stopDist; fwd++) {
            for (int side = -20; side < 20; side++) {
                double radian_angle = fmod((heading), 360) * 3.14 / 180;
                int y = std::max(
                        std::min(int(start_y + fwd * cos(radian_angle) + side * sin(radian_angle)), HEIGHT - 1),
                        0);
                int x = std::max(std::min(int(start_x + fwd * sin(radian_angle) - side * cos(radian_angle)), WIDTH - 1),
                                 0);
                //printf("OBSTACLE DETECTED! %dcm\n", y-(int)curr_y);
                cv::Mat ROI = canvas2(cv::Rect(x, y, 1, 1));
                ROI.setTo(cv::Scalar(0, 0, 100));

            }
        }

        stopDist = 250;
        int left_dist = 0;
        int right_dist = 0;
        for (int fwd = 0; fwd < stopDist; fwd++) {
            for (int side = -5; side < 5; side++) {
                double radian_angle = fmod((heading - 15), 360) * 3.14 / 180;
                int y = std::max(
                        std::min(int(start_y + fwd * cos(radian_angle) + side * sin(radian_angle)), HEIGHT - 1),
                        0);
                int x = std::max(std::min(int(start_x + fwd * sin(radian_angle) - side * cos(radian_angle)), WIDTH - 1),
                                 0);
                //printf("OBSTACLE DETECTED! %dcm\n", y-(int)curr_y);
                cv::Mat ROI = canvas2(cv::Rect(x, y, 1, 1));
                if (mapArr[y][x] >= 1) {
                    //printf("Left Dist! %dcm\n", (int) std::sqrt(std::pow(curr_x - x, 2) + std::pow(curr_y - y, 2)));
                    ROI.setTo(cv::Scalar(200, 225, 255));
                    left_dist = (int) std::sqrt(std::pow(curr_x - x, 2) + std::pow(curr_y - y, 2));
                } else {
                    ROI.setTo(cv::Scalar(0, 0, 255));
                }
            }
        }

        for (int fwd = 0; fwd < stopDist; fwd++) {
            for (int side = -5; side < 5; side++) {
                double radian_angle = fmod((heading + 15), 360) * 3.14 / 180;
                int y = std::max(
                        std::min(int(start_y + fwd * cos(radian_angle) + side * sin(radian_angle)), HEIGHT - 1),
                        0);
                int x = std::max(std::min(int(start_x + fwd * sin(radian_angle) - side * cos(radian_angle)), WIDTH - 1),
                                 0);
                //printf("OBSTACLE DETECTED! %dcm\n", y-(int)curr_y);
                cv::Mat ROI = canvas2(cv::Rect(x, y, 1, 1));
                if (mapArr[y][x] >= 1) {
                    //printf("Right Dist! %dcm\n", (int) std::sqrt(std::pow(curr_x - x, 2) + std::pow(curr_y - y, 2)));
                    ROI.setTo(cv::Scalar(200, 225, 255));
                    right_dist = (int) std::sqrt(std::pow(curr_x - x, 2) + std::pow(curr_y - y, 2));
                } else {
                    ROI.setTo(cv::Scalar(0, 0, 255));
                }

            }
        }

        if (abs(left_dist - right_dist) < 10) {

        } else if (left_dist < right_dist) {
            heading -= 2;
        } else if (right_dist < left_dist) {
            heading += 2;
        }

        cv::flip(canvas, canvas, 0);
        cv::rotate(canvas, canvas, 0);

        cv::flip(canvas2, canvas2, 0);
        cv::rotate(canvas2, canvas2, 0);

        //cv::imwrite("hallMapColor.jpg", canvas);
        //cv::imshow("Final Map", canvas);
        cv::imshow("Final Map2", canvas2);
        cv::waitKey(1);
        encoder_cm += 5;
        curr_y += (encoder_cm - last_encoders) * sin((heading + 90) * 3.14 / 180);
        curr_x += (encoder_cm - last_encoders) * -cos((heading + 90) * 3.14 / 180);
        last_encoders = encoder_cm;
    }

    return 0;
}
