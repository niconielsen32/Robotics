#include "pathplanning.h"

pathPlanning::pathPlanning(std::string ImageIn)
{

    // loading image
    imageOrg = cv::imread(ImageIn);
    if (imageOrg.empty()){
        std::cerr << "Could not load image" << std::endl;
    }

}

void pathPlanning::brushfire(bool showImages, int imageScaleFactor) { //If showImages is set to true, all the steps in finding the node is shown.
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

    //Pixel values from bottom to topdilated
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

    const double divider = 255 / 10; // 15 is choosen because it is the highest value in the grid

    // Create a grayscale image of the brushfire algoritme
    Brushfire = image;

    for (size_t i = 0; i < brushValues.size(); i++){
        for (size_t j = 0; j < brushValues[i].size(); j++){
            divider * brushValues[i][j] > 255 ? Brushfire.at<uchar>(i, j) = 255 : Brushfire.at<uchar>(i, j) = divider * brushValues[i][j];
        }
    }


    findNodes(showImages, imageScaleFactor);
    std::cerr << "time taken: " << (float)clock()/CLOCKS_PER_SEC << " secs" << std::endl;
}

void pathPlanning::findNodes(bool showImages, int scaleFactor)
{
    //Finding the nodes using center of mass
    //--------------------------------------------------

    cv::Mat dilated, thresholded_matching_space, local_maxima, thresholded_8bit;

    cv::dilate(Brushfire, dilated, cv::Mat());

    //Compare the dilated image to the orginal and put it in a new Mat
    cv::compare(Brushfire, dilated, local_maxima, cv::CMP_EQ);

    cv::Mat scaled_local_max;
    local_maxima.copyTo(scaled_local_max);


    //Skal ligge omkring 150
    int threshold = 150;
    //Change the original grayscale image to a binary image of black and white, depending on the value of each pixel.
    //If the pixel value is greater than 150 then it turns white.
    cv::threshold(Brushfire, thresholded_matching_space, threshold, 255, cv::THRESH_BINARY);

    thresholded_matching_space.convertTo(thresholded_8bit, CV_8U);

    cv::bitwise_and(local_maxima, thresholded_8bit, local_maxima);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(local_maxima, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0,0));

    //get moments
    //A moment is a shape difined by a contour.
    std::vector<cv::Moments> mu(contours.size());
    for (size_t i = 0; i < contours.size(); i++) {
        //Each object in mu is a shape made from the contours
        mu[i] = cv::moments(contours[i], false);
    }

    //get centroids of figures
    std::vector<cv::Point> mc(contours.size());
    for (size_t i = 0; i < contours.size(); i++) {
        //https://docs.opencv.org/3.4/d8/d23/classcv_1_1Moments.html
        //The center of mass of each shape in mu i caluclated and put in vector mc.
        mc[i] = cv::Point( mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00);
    }

    //Remove "not a number" points (NaN)
    //"Since the contour moments are computed using Green formula, you may get seemingly odd results for contours with
    //self-intersections, e.g. a zero area (m00) for butterfly-shaped contours."
    for(size_t i = mc.size() - 1; i > 0; i--) {
        if (mc[i].x < 0) {
            mc.erase(mc.begin() + i);
        }
    }


    //Remove points that are close to eachother within a set threshold
    int closnes_threshold = 11;
    for (size_t i = mc.size() - 1 ; i > 0 ; i--) {
        for (size_t j = i - 1 ; j > 1; j--) {
            if (i == j) {
                //Do nothing
            } else if (abs(mc[i].x - mc[j].x) < closnes_threshold && abs(mc[i].y - mc[j].y) < closnes_threshold) {
                mc.erase(mc.begin() + i);
            }
        }
    }

    //Draw the final points in the original image.
    for (size_t i = 0; i < mc.size(); i++) {
        cv::Scalar color = cv::Scalar(255,0,0);
        cv::drawMarker(imageOrg, mc[i], color, cv::MARKER_STAR, 1, 1, 8);
        //std::cout << mc[i] << std::endl;
    }
    std::cout << std::endl;
    std::cout << std::endl;


    if (showImages) {
        cv::Mat scaledImage(imageOrg.cols * 3, imageOrg.rows * 3, CV_8UC3);
        cv::resize(imageOrg, scaledImage, cv::Size(imageOrg.cols * scaleFactor, imageOrg.rows * scaleFactor), 0, 0, cv::INTER_NEAREST);
        cv::resize(Brushfire, Brushfire, cv::Size(imageOrg.cols * scaleFactor, imageOrg.rows * scaleFactor), 0, 0, cv::INTER_NEAREST);
        cv::resize(dilated, dilated, cv::Size(imageOrg.cols * scaleFactor, imageOrg.rows * scaleFactor), 0, 0, cv::INTER_NEAREST);
        cv::resize(local_maxima, local_maxima, cv::Size(imageOrg.cols * scaleFactor, imageOrg.rows * scaleFactor), 0, 0, cv::INTER_NEAREST);
        cv::resize(thresholded_matching_space, thresholded_matching_space, cv::Size(imageOrg.cols * scaleFactor, imageOrg.rows * scaleFactor), 0, 0, cv::INTER_NEAREST);
        cv::resize(thresholded_8bit, thresholded_8bit, cv::Size(imageOrg.cols * scaleFactor, imageOrg.rows * scaleFactor), 0, 0, cv::INTER_NEAREST);
        cv::resize(scaled_local_max, scaled_local_max, cv::Size(imageOrg.cols * scaleFactor, imageOrg.rows * scaleFactor), 0, 0, cv::INTER_NEAREST);

        cv::imshow("Original Brushfire", Brushfire);
        cv::waitKey(0);
        cv::imshow("Brushfire Dilated", dilated);
        cv::waitKey(0);
        cv::imshow("local max", scaled_local_max);
        cv::waitKey(0);
        cv::imshow("max", local_maxima);
        cv::waitKey(0);
        cv::imshow("Thresholded Brushfire", thresholded_matching_space);
        cv::waitKey(0);
        cv::imshow("8_bit", thresholded_8bit);
        cv::waitKey(0);

        cv::imshow("Original with point", scaledImage);
        cv::waitKey(0);
    }

    cv::imwrite("Points_on_image.png", imageOrg);

    std::ofstream write;
    write.open("nodes found.txt");

    for (size_t i = 0; i < mc.size(); i++) {
        write << mc[i] << std::endl;
    }
    write.close();

    nodes = mc;


}
