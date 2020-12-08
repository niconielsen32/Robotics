#ifndef PATHPLANNING_H
#define PATHPLANNING_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <vector>
#include <bits/stdc++.h>

class pathPlanning
{
public:
    pathPlanning(std::string ImageIn);
    void brushfire();


private:
    int black = 0;
    int white = 255;
    std::vector<std::vector<int>> brushValues;
    cv::Mat imageOrg;
    void non_maxima_suppression(const cv::Mat& image, cv::Mat& mask, bool remove_plateaus);

};

#endif // PATHPLANNING_H
