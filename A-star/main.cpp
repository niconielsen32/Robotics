#include "astar.h"
#include "pathplanning.h"
#include "travel.h"
#include "tests.h"

#include <limits>
#include <iostream>
#include <fstream>
#include <iomanip>


using namespace std;





int main()
{
    std::string image = "floor_plan.png";
    std::vector<cv::Point> locations = {cv::Point(80,70), cv::Point(54,70), cv::Point(85,52), cv::Point(31,60),       //0, 1, 2, 3
                                           cv::Point(9,47), cv::Point(35, 42), cv::Point(106,56), cv::Point(93,33),    //4, 5, 6, 7
                                           cv::Point(68,25), cv::Point(11,24), cv::Point(22,18), cv::Point(42,14),      //8, 9, 10, 11
                                           cv::Point(108,11), cv::Point(66,9), cv::Point(9,7)};

    //createLookUpTable(locations, image, "NewlookUp.txt");
    //createLookUpTable(locations, image, "lookuptable.txt");
    //testOfGcostValue();
//    testShortesPathUsingAPopulation(200);
//    testShortesPathUsingALexicographicalOrder();
//    resizeImages("floor_plan.png", 5);

//    Astar test(image);
//    test.findPath( cv::Point(9,47), cv::Point(108,11));
//    test.finalPath();
//    test.showImage(2);

    return 0;

}
