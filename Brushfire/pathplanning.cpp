#include "pathplanning.h"

pathPlanning::pathPlanning(std::string ImageIn)
{

    // loading image
    imageOrg = cv::imread(ImageIn);
    if (imageOrg.empty()){
        std::cerr << "Could not load image";
    }

}

void pathPlanning::non_maxima_suppression(const cv::Mat& image, cv::Mat& mask, bool remove_plateaus) {
    // find pixels that are equal to the local neighborhood not maximum (including 'plateaus')
    cv::dilate(image, mask, cv::Mat());
    cv::compare(image, mask, mask, cv::CMP_GE);

    // optionally filter out pixels that are equal to the local minimum ('plateaus')
    if (remove_plateaus) {
        cv::Mat non_plateau_mask;
        cv::erode(image, non_plateau_mask, cv::Mat());
        cv::compare(image, non_plateau_mask, non_plateau_mask, cv::CMP_GT);
        cv::bitwise_and(mask, non_plateau_mask, mask);
    }
}

void pathPlanning::brushfire() {
    // Grayscale image to get one channel
    cv::Mat image;
    cv::cvtColor(imageOrg, image, cv::COLOR_RGB2GRAY);

    size_t cols = image.cols;
    size_t rows = image.rows;

    // Assining each pixel a value increasing from a wall from left to right
    int index = 2;
    for (size_t i = 0; i < rows; i++){
        std::vector<int> tempRows; // Temporary vector to contain values of each row.
        for (size_t j = 0; j < cols; j++) {
            if ((int)image.at<uchar>(i, j) != black){
                tempRows.push_back(index++);
            } else if ((int)image.at<uchar>(i, j) == black){
                tempRows.push_back(0);
                index = 2;
            }
        }
        brushValues.push_back(tempRows);
    }

    //Pixel values from right to left
    index = 2;
    for (size_t i = 0; i < rows; i++){
        for (size_t j = 0; j < cols; j++) {
            if (brushValues[i][cols - 1 - j] != 0 && brushValues[i][cols - 1 - j] > index){
                brushValues[i][cols - 1 - j] = index++;
            } else if (brushValues[i][cols - 1 - j] == 0){
                index = 2;
            }
        }
    }

    //Pixel values from top to bottom
    index = 2;
    for (size_t j = 0; j < cols; j++){
        for (size_t i = 0; i < rows; i++) {
            if (brushValues[i][j] != 0 && brushValues[i][j] > index){
                brushValues[i][j] = index++;
            } else if (brushValues[i][j] == 0){
                index = 2;
            }
        }
    }

    //Pixel values from bottom to top
    index = 2;
    for (size_t j = 0; j < cols; j++){
        for (size_t i = 0; i < rows; i++) {
            if (brushValues[rows - 1 - i][j] != 0 && brushValues[rows - 1 - i][j] > index){
                brushValues[rows - 1 - i][j] = index++;
            } else if (brushValues[rows - 1 - i][j] == 0){
                index = 2;
            }
        }
    }


//    // Shows the outut of the brushfire before correcting
//    for (size_t i = 0; i < brushValues.size(); i++){
//        for (size_t j = 0; j < brushValues[i].size(); j++){
//            std::cout << std::setw(3) << brushValues[i][j] ;
//        }
//        std::cout << "next line "<< std::endl;
//    }
//    std::cout << std::endl;

    // Correction of numbers so they only jump one up or down in each direction
    int value = 0;
    while(value < 15){

        for (size_t j = 0; j < cols; j++){
            for (size_t i = 0; i < rows; i++) {
                // Checks if the neighbour right is 2 or more values from the current pixel value
                if (brushValues[i][j] != 0 && brushValues[i][j] != 2){
                    if (abs(brushValues[i][j] - brushValues[i + 1][j]) >= 2 ){


                        if (brushValues[i][j] - brushValues[i + 1][j] <= -2 ){
                            brushValues[i][j]++;
                        } else if (brushValues[i][j] - brushValues[i + 1][j] >= 2 ) {
                            brushValues[i][j]--;
                        }
                    }

                    // Checks neighbour left
                    if (abs(brushValues[i][j] - brushValues[i - 1][j]) >= 2 ){


                        if (brushValues[i][j] - brushValues[i - 1][j] <= -2 ){
                            brushValues[i][j]++;
                        } else if (brushValues[i][j] - brushValues[i - 1][j] >= 2 ){
                            brushValues[i][j]--;
                        }
                    }

                    // Checks neighbour down
                    if (abs(brushValues[i][j] - brushValues[i][j + 1]) >= 2 ){

                        if (brushValues[i][j] - brushValues[i][j + 1] <= -2 ){
                            brushValues[i][j]++;
                        } else if (brushValues[i][j] - brushValues[i][j + 1] >= 2 ){
                            brushValues[i][j]--;
                        }
                    }

                    // Checks neighbour up
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

//    //Show the result of the brushfire algorithme
//    for (size_t i = 0; i < brushValues.size(); i++){
//        for (size_t j = 0; j < brushValues[i].size(); j++){
//            std::cout << std::setw(3) << brushValues[i][j] ;
//        }
//        std::cout << "next line "<< std::endl;
//    }

    const double divider = 255 / 10; // 15 is choosen because it is the highest value in the grid

    // Create a grayscale image of the brushfire algoritme
    cv::Mat Brushfire (image.rows, image.cols, CV_8UC1);
    for (size_t i = 0; i < brushValues.size(); i++){
        for (size_t j = 0; j < brushValues[i].size(); j++){
            divider * brushValues[i][j] > 255 ? Brushfire.at<uchar>(i, j) = 255 : Brushfire.at<uchar>(i, j) = divider * brushValues[i][j];        }
    }


    cv::Mat BigBrush, originalScaled, dilated, Points, filter;
    cv::Size size(300, 300);

    cv::resize(Brushfire, BigBrush, size);
    cv::resize(imageOrg, originalScaled, size);
    std::cout << std::endl;

//    cv::dilate(BigBrush, dilated, cv::Mat());
//    cv::compare(BigBrush, dilated, dilated, cv::CMP_GE);
//    cv::erode(BigBrush, filter, cv::Mat());
//    cv::compare(BigBrush, filter, filter, cv::CMP_GT);
//    cv::bitwise_and(dilated, filter, dilated);

    non_maxima_suppression(BigBrush, dilated, true);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(dilated, contours, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    for( auto& contour:contours) {
        auto localMax = contour.begin() + contour.size() / 4;
//        std::cout << localMax << std::endl;
        cv::circle(originalScaled, *localMax, 2, cv::Scalar(0,0,255));
    }

    cv::namedWindow("Brushfire");
    cv::imshow("Scaled original map", originalScaled);
    cv::imshow("Brushfire", BigBrush);
    cv::imwrite("brushfire.png", Brushfire);
    cv::imshow("Dilated", dilated);
    cv::waitKey(0);
    std::cerr << "time taken: " << (float)clock()/CLOCKS_PER_SEC << " secs" << std::endl;
}
