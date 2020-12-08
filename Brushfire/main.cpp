#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
//#include <stdlib.h>

const cv::Vec3b black {0,0,0};
const cv::Vec3b white {255, 255, 255};

using namespace std;

int main()
{
    std::vector<std::vector<long int>> brushValues;

    cv::Mat image = cv::imread("floor_plan.jpg");
    if (image.empty()){
        std::cerr << "Could not load image";
        return 1;
    }

    int x = image.cols;
    int y = image.rows;

    // Assing each pixel a value increasing from a wall from left to right
    int startFromWall = 2;
    for (int j = 0; j < y; j++){
        std::vector<long int> tempRows; // Temporary vector to contain values of each row.
        for (int i = 0; i < x; i++) {

            if (image.at<cv::Vec3b>(j, i) != black){
                tempRows.push_back(startFromWall++);
            } else if (image.at<cv::Vec3b>(j, i) == black){
                tempRows.push_back(0);
                startFromWall = 2;
            }
        }
        brushValues.push_back(tempRows);
    }
    int index = 2;
    //Kigger fra venstre mod højre
    for (int j = 0; j < y; j++){
        for (int i = 0; i < x; i++) {
            if (brushValues[j][x - 1 - i] != 0 && brushValues[j][image.cols - 1 - i] > index){
                brushValues[j][x - 1 - i] = index++;
            } else if (brushValues[j][x - 1 - i] == 0){
                index = 2;
            }
        }
    }
    index = 2;
    //Kigger fra top til bund
    for (int j = 0; j < image.cols; j++){
        for (int i = 0; i < image.rows; i++) {
            if (brushValues[i][j] != 0 && brushValues[i][j] > index){
                brushValues[i][j] = index++;
            } else if (brushValues[i][j] == 0){
                index = 2;
            }
        }
    }
    index = 2;
    //Kigger fra bund til top
    for (int j = 0; j < image.cols; j++){
        for (int i = 0; i < image.rows; i++) {
            if (brushValues[image.rows - 1 - i][j] != 0 && brushValues[image.rows - 1 - i][j] > (index )){
                brushValues[image.rows - 1 - i][j] = index++;
            } else if (brushValues[image.rows - 1 - i][j] == 0){
                index = 2;
            }
        }
    }


    // Shows the outut of the brushfire before correcting
    for (size_t j = 0; j < brushValues.size(); j++){
        for (size_t i = 0; i < brushValues[j].size(); i++){
            std::cout << std::setw(3) << brushValues[j][i] ;
        }
        std::cout << "next line "<< std::endl;
    }
    std::cout << std::endl;

    // Correction of numbers so they only jump one up or down in each direction
    int value = 0;
    while(value < 15){

        for (int j = 0; j < image.cols; j++){

            for (int i = 0; i < image.rows; i++) {
                if (brushValues[i][j] != 0 && brushValues[i][j] != 2){
                    if (abs(brushValues[i][j] - brushValues[i + 1][j]) >= 2 ){


                        if (brushValues[i][j] - brushValues[i + 1][j] <= -2 ){
                            brushValues[i][j]++;
                        } else if (brushValues[i][j] - brushValues[i + 1][j] >= 2 ) {
                            brushValues[i][j]--;
                        }
                    }

                    if (abs(brushValues[i][j] - brushValues[i - 1][j]) >= 2 ){


                        if (brushValues[i][j] - brushValues[i - 1][j] <= -2 ){
                            brushValues[i][j]++;
                        } else if (brushValues[i][j] - brushValues[i - 1][j] >= 2 ){
                            brushValues[i][j]--;
                        }
                    }

                    if (abs(brushValues[i][j] - brushValues[i][j + 1]) >= 2 ){

                        if (brushValues[i][j] - brushValues[i][j + 1] <= -2 ){
                            brushValues[i][j]++;
                        } else if (brushValues[i][j] - brushValues[i][j + 1] >= 2 ){
                            brushValues[i][j]--;
                        }
                    }

                    if (abs(brushValues[i][j] - brushValues[i][j - 1]) >= 2 ){

                        if (brushValues[i][j] - brushValues[i][j - 1] <= -2 ){
                            brushValues[i][j]++;
                        } else if (brushValues[i][j] - brushValues[i][j - 1] >= 2 ){
                            brushValues[i][j]--;
                        }
                    }
                }
            }
        }
        value++;
    }

    //Show the result of the brushfire algorithme
    for (size_t j = 0; j < brushValues.size(); j++){
        for (size_t i = 0; i < brushValues[j].size(); i++){
            std::cout << std::setw(3) << brushValues[j][i] ;
        }
        std::cout << "next line "<< std::endl;
    }

    const double divider = 255 / 15; // 15 is choosen because it is the highest value in the grid

    // Create a grayscale image of the brushfire algoritme
    cv::Mat Brushfire (image.rows, image.cols, CV_8UC1);
    for (size_t j = 0; j < brushValues.size(); j++){
        for (size_t i = 0; i < brushValues[j].size(); i++){
            Brushfire.at<uchar>(j, i) = divider * brushValues[j][i];
        }
    }


    cv::Mat BigBrush;
    cv::Size size(300, 300);

    cv::resize(Brushfire, BigBrush, size);
    std::cout << std::endl;

    cv::namedWindow("Brushfire");

    cv::imshow("Brushfire", BigBrush);
    cv::imwrite("brusfire.png", Brushfire);
    cv::waitKey(0);
    return 0;
}
