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
#include <fstream>
#include <ostream>

class pathPlanning
{
public:
    pathPlanning(std::string ImageIn);
    void brushfire(bool showImages, int ImageScaleFactor = 3);
    std::vector<std::vector<int>> getBrushfire(){return brushValues;};
    std::vector<cv::Point> getNodes() {return nodes;};


private:
    void findNodes(bool showImages, int scalefactor);

    int black = 0;
    int white = 255;
    std::vector<std::vector<int>> brushValues;
    cv::Mat imageOrg;
    cv::Mat Brushfire;
    std::vector<cv::Point> nodes;
};

#endif // PATHPLANNING_H
