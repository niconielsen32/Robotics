#ifndef TRAVEL_H
#define TRAVEL_H

#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

//#include <algorithm>
#include <vector>

struct location {
    int id;
    cv::Point point;
    std::vector<std::pair<int, double>> distance;
};


class travel
{
public:
    travel(std::vector<int> init) {initialPath = init;};
    double calcPath(std::vector<int> order);
    std::vector<int> calcPathLexi(std::vector<std::vector<int>> &comb, std::vector<location> &lookup);
    void normalizeFitness();
    void nextGeneration();
    void calculateFitness();
    void createPopulation(int num);
    std::vector<int> pickOne (std::vector<std::vector<int>> population, std::vector<double> fitness);
    void mutate(std::vector<int> order, int mutationRate);

    void lexi();

    void getLookUptable(std::vector<location> vec) {lookupTable = vec;};
    double getBestDistance() {return overallBest;};
    std::vector<int> getBestPath(){return overallBestPath;};
    std::vector<int> getPathToCheck(){return initialPath;};


private:
    double overallBest = std::numeric_limits<double>::infinity();
    std::vector<int> overallBestPath;

    std::vector<double> fitness;
    std::vector<std::vector<int>> population;
    std::vector<location> lookupTable;

//    std::vector <int> initialPath = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
    std::vector <int> initialPath = {1, 2, 3, 4, 8, 13, 10, 5, 7, 14, 11, 16, 6, 9, 12, 17, 15 };


};

#endif // TRAVEL_H
