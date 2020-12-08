#include "travel.h"
#include <time.h>
#include <exception>



double travel::calcPath(std::vector<int> order)
{

    double distance = 0;
    for (size_t j = 0; j < order.size() - 1; j++) {
        long long int index;
        long long int nextIndex;

        if (order[j] < order[j + 1]) {
            index = order[j];
            nextIndex = order[j + 1];
        } else {
            index = order[j + 1];
            nextIndex = order[j];
        }

        for (size_t k = 0; k < lookupTable[index - 1].distance.size(); k++) {
            if (lookupTable[index - 1].distance[k].first == nextIndex) {
                distance += lookupTable[index - 1].distance[k].second;
                break;
            }
        }
    }

    if (distance < overallBest) {
        overallBest = distance;
        overallBestPath = order;
    }
    if (distance == 0)
        std::cout << "Distance er ikke stor nok" << std::endl;
    return distance;
}

void travel::createPopulation(int num)
{
    for (int i = 0; i < num; i++) {
        //Push back vectors
        population.push_back(initialPath);
        std::random_shuffle(initialPath.begin(), initialPath.end());
    }  
}

void travel::calculateFitness()
{
    fitness.clear();
    for (size_t i = 0; i < population.size(); i++) {
        double d = calcPath(population[i]);
        double fitnessScore = (1 / d) ;
        fitness.push_back(fitnessScore);
    }
}

void travel::normalizeFitness()
{
    double sum = 0;
    for (size_t i = 0; i < fitness.size(); i++) {
        sum += fitness[i];
    }
    for (size_t i = 0; i < fitness.size(); i++) {
        fitness[i] = fitness[i] / sum;
    }
}

std::vector<int> travel::pickOne(std::vector<std::vector<int> > population, std::vector<double> fitness)
{
    int index = 0;
    double r = (rand() % 100) / 100.0;

    while (r > 0) {
        r = r - fitness[index];
        index++;
    }
    index--;
    if (index < 0) {
        return population[0];
    }
        return population[index];

}

void travel::mutate(std::vector<int> &order, int mutationRate)
{
    int indexA = rand() % order.size();
    int indexB = rand() % order.size();
    std::swap(order[indexA], order[indexB]);
}

void travel::nextGeneration()
{
    std::vector<std::vector<int>> newPopulation;
    std::vector<std::vector<int>> oldPopulation = population;
    std::vector<int> order;
    population.clear();

    for (size_t i = 0; i < oldPopulation.size(); i++) {
        order = pickOne(oldPopulation, fitness);
        mutate(order, 1);
        newPopulation.push_back(order);
    }

    population = newPopulation;
}

void travel::lexi()
{
    double distance = calcPath(initialPath);
    std::next_permutation(initialPath.begin(), initialPath.end());
    if (distance < overallBest) {
        overallBest = distance;
        overallBestPath = initialPath;
    }

}
