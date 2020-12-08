#ifndef TESTS_H
#define TESTS_H

#endif // TESTS_H

#include "astar.h"
#include "pathplanning.h"
#include "travel.h"
#include "globalFunctions.h"

#include <limits>
#include <iostream>
#include <fstream>
#include <iomanip>

void testOfGcostValue() {

        std::string image = "floor_plan.png";
        Astar test(image);
        test.findPath(cv::Point(9,7),cv::Point(106,56));
        test.finalPath();
        test.showImage(2);
}

void testShortesPathUsingAPopulation(int populationSize) {

    std::string image = "floor_plan.png";
    std::vector <int> initialPath = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

    //Path planning creates the nodes on the map.
    pathPlanning nodes(image);
    nodes.brushfire(false);
    std::vector<cv::Point> locations = nodes.getNodes();

    // The class travel vil finde the shortes route between all the nodes fom locations.
    travel gen(initialPath);
    gen.getLookUptable(readLookUptable("NewlookUp.txt"));
    gen.getLookUptable(readLookUptable("lookuptable.txt"));
    gen.createPopulation(populationSize);


    double currentBestDistance = 1000000;
    std::vector<int> currentBestPath = initialPath;
    int iterations = 0;
    std::string name = "Final Path generation ";
    while (1) {
        gen.calculateFitness();
        gen.normalizeFitness();
        gen.nextGeneration();

        if (currentBestDistance > gen.getBestDistance()) {

            currentBestDistance = gen.getBestDistance();
            currentBestPath = gen.getBestPath();
            std::cout << "The best distance: " << currentBestDistance << " with:" << std::endl;
            for (size_t i = 0; i < currentBestPath.size(); i++) {
                std::cout << currentBestPath[i] << " ";
            }
            std::cout << std::endl;


            for (size_t i = 0; i < currentBestPath.size() - 1; i++) {
                //To create a new .png image to with different names.
                name = "Final_Path_generation_";
                name += std::to_string(iterations);
                name += ".png";
                //std::cout << locations[currentBestPath[i] - 1] << " " << locations[currentBestPath[i + 1] - 1] << std::endl;

                if (i == 0) {
                    Astar draw(image);
                    draw.findPath(locations[currentBestPath[i] - 1], locations[currentBestPath[i + 1] - 1]);
                    draw.finalPath();
                    draw.showAndWriteFinalPath(name, i + 1);
                } else {
                    Astar draw(name);
                    draw.findPath(locations[currentBestPath[i] - 1], locations[currentBestPath[i + 1] - 1]);
                    draw.finalPath();
                    draw.showAndWriteFinalPath(name, i + 1 );
                }
            }
            iterations++;
        }


    }
}

void testShortesPathUsingALexicographicalOrder() {
        std::string image = "floor_plan.png";
        pathPlanning nodes(image);
        nodes.brushfire(false); //Making brushfire to get the nodes in the map
        std::vector<cv::Point> locations = nodes.getNodes();

        std::vector<location> lookupTable;
        lookupTable = readLookUptable("lookuptable.txt");

        std::pair<double, std::vector<int>> currentBest = readBest("Best_Route.txt");
        std::pair<double, std::vector<int>> toCheck = readBest("Current_Progress.txt");
        std::ofstream BestRoute;
        std::ofstream NextCheck;
        travel travel(toCheck.second);

        travel.getLookUptable(lookupTable);

        double currentBestDistance = currentBest.first;
        std::vector<int> currentBestPath = currentBest.second;
        std::vector<int> currentCombinationToCheck;


        long long int numberOfCombinationsTried = toCheck.first;
        while (1) {
            travel.lexi();
            if (currentBestDistance > travel.getBestDistance()) {
                currentBestDistance = travel.getBestDistance();
                currentBestPath = travel.getBestPath();

                BestRoute.open("Best_Route.txt");

                BestRoute << currentBestDistance << std::endl;
                std::cout << "The best distance: " << currentBestDistance << " with:" << std::endl;
                for (size_t i = 0; i < currentBestPath.size(); i++) {
                    std::cout << currentBestPath[i] << " ";
                    BestRoute << currentBestPath[i] << " ";
                }
                std::cout << std::endl;
                BestRoute << std::endl;
                BestRoute.close();

            }

            if (numberOfCombinationsTried % 1000000 == 0)
            {

                NextCheck.open("Current_Progress.txt");
                NextCheck << numberOfCombinationsTried << std::endl;

                std::cout << "Combinations tried: " << numberOfCombinationsTried << std::endl;
                std::cout << "Current combination being checked: " << std::endl;
                currentCombinationToCheck = travel.getPathToCheck();
                for (size_t i = 0; i < currentCombinationToCheck.size(); i++ ) {
                    std::cout << currentCombinationToCheck[i] << " " ;
                    NextCheck << currentCombinationToCheck[i] << " ";
                }

                std::cout << std::endl;


                long double top = (long double)numberOfCombinationsTried;
                long double buttom = factorial(15);
                long double finished = (top/buttom) * 100;

                std::cout << "It is: " << std::fixed << std::setprecision(30) << finished << " % " << "finished" << std::endl;
                std::cout << std::endl;
                NextCheck.close();
            }

            numberOfCombinationsTried++;
        }

}

