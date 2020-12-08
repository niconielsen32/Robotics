#ifndef GLOBALFUNCTIONS_H
#define GLOBALFUNCTIONS_H

#endif // GLOBALFUNCTIONS_H

#include "astar.h"
#include "pathplanning.h"
#include "travel.h"

#include <limits>
#include <iostream>
#include <fstream>
#include <iomanip>

void writeFile (std::vector<location> lookupTable, std::string fileName) {
    // Write out to a file all the distances
    std::ofstream outfile;
    outfile.open(fileName);
    for (size_t i = 0; i < lookupTable.size(); i++) {
        outfile << lookupTable[i].id << " " << lookupTable[i].point << "   ";
        for (size_t j = 0; j < lookupTable[i].distance.size(); j++) {
            outfile << "[ " << lookupTable[i].distance[j].first << " " << lookupTable[i].distance[j].second << " ]" ;
        }
        outfile << std::endl;
    }
    outfile.close();
}

void createLookUpTable (std::vector<cv::Point> locations, std::string image, std::string fileName) {
//        Alle punkter har fået et id nummer
        std::vector<location> lookupTable;
        for (size_t i = 0; i < locations.size(); i++) {
            location newLocation;
            newLocation.id = i + 1;
            newLocation.point = locations[i];
            lookupTable.push_back((newLocation));
        }


        // Finde the distance between all the points above
        for(size_t j = 0; j < lookupTable.size(); j++) {
            for(size_t k = j + 1; k < lookupTable.size(); k++) {

                // Start to find the distance between two points
                cv::Point currentPosition = locations[j];
                cv::Point targetPoint = locations[k];

                Astar path(image);
                path.findPath(currentPosition, targetPoint);
                path.finalPath();

                //Add that distance to the lookup table
                lookupTable[j].distance.push_back(std::make_pair(k + 1, path.getFinalDistance()));
                std::cout << "j = " << j << " og k = " << k << std::endl;
            }
        }
        writeFile(lookupTable, fileName);
}

std::pair<double, std::vector<int>> readBest(std::string str){
    std::ifstream read;
    read.open(str);
//    read.open("lexiograf.txt");

    if (!read) {
        std::cout << "Unable to open file" << std::endl;
    }

    std::pair <double, std::vector<int>> result;
    std::string line;
    std::getline(read, line);
    double currentBestDistence = std::stod(line);
    result.first = currentBestDistence;


    std::vector<int> currentBestPath;
    std::string temp;
    std::getline(read, line);
    size_t index = 0;
    while (line.length() >= index) {

        if (line[index] != ' ' && line[index] >=48 && line[index] <= 57) {
            temp += line[index];

        }
        if (line[index] == ' ' && !temp.empty() ) {
            currentBestPath.push_back(std::stoi(temp));
            temp.clear();
        }
        index++;
    }
    if (!temp.empty() ) {
        currentBestPath.push_back(std::stoi(temp));
        temp.clear();
    }

    result.second = currentBestPath;

    return result;

}

std::vector<location> readLookUptable(std::string NameOfTable) {

    std::vector<location> lookuptable;
    std::ifstream read;
    read.open(NameOfTable);

    if (!read) {
        std::cout << "Unable to open file" << std::endl;
    }

    std::string line;
    while (std::getline(read, line)) {

        location newLocation;
        int index = 0;
        std::string str;

        std::pair<int, double> temp;

        while (index < line.length()) {
            while (line[index] != ' ' && line[index] != '[' && line[index] != ']') {
                str += line[index];
                index++;
            }

            if (index < 3 && str.length()) {
                newLocation.id = std::stoi(str);
                str.clear();
            } else if (index <= 12) {
                int x;
                int y;
                if (str.length()) {
                    x = std::stoi(str);
                    str.clear();
                    index ++;
                }
                while (line[index] != ' ' && line[index] != '[' && line[index] != ']') {
                    str += line[index];
                    index++;
                }
                if (str.length()) {
                    y = std::stod(str);
                    str.clear();
                    newLocation.point = cv::Point(x, y);
                }

            } else if (str.length() < 3 && str.length() > 0) {
                temp.first = std::stoi(str);
                str.clear();
            } else if (str.length() > 3) {
                temp.second = std::stod(str);
                newLocation.distance.push_back(temp);
                str.clear();
            }
            index++;
        }

        lookuptable.push_back(newLocation);

    }

    return lookuptable;

}

long int factorial(int n) {

    if (n == 1) {
        return 1;
    }
    return n * factorial(n - 1);

}

void resizeImages(std::string fileName, int scaleFactor) {

    cv::Mat inImage;
    inImage = cv::imread(fileName);
    if (fileName.empty()){
        std::cerr << "Could not load image to resize" << std::endl;
    }

    cv::resize(inImage, inImage, cv::Size(inImage.cols * scaleFactor, inImage.rows * scaleFactor), 0,0, cv::INTER_NEAREST);
    cv::imwrite(fileName, inImage);
}

