#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H


#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <random>

struct Particle {

    int id;
    int x;
    int y;
    double orientation;
    double weight;
    std::vector<double> associations;
    std::vector<double> sense_x;
    std::vector<double> sense_y;

    double lidar[70];

};


class ParticleFilter
{

public:
    ParticleFilter();
    ParticleFilter(int numberOfParticles)
        :numberOfParticles(numberOfParticles) {}

    std::vector<Particle> particles;


    void drawParticles(cv::Mat &map);

    void drawLidarData(cv::Mat &map);

    void lidarParticles(cv::Mat &map);

    void drawLidarParticles(cv::Mat &map);

    void robotLidar(Particle &robot, cv::Mat &map);

    void drawRobotParticle(Particle &robot, cv::Mat &map);

    void drawLidarRobot(Particle &robot, cv::Mat &map);

    void moveRobot(double delta_timestep, double stdPos, double velocity, double orientation_rate, Particle &robot);


    bool particleFilterInitialized(){ return particleFilterInit; }


    // Function to run the particle filter on map. Implement as the last thing if its possible
    void runParticleFilter(int map);

    // Function for particle filter
    void initializeParticleFilter(double x, double y, double orientation, double std[], cv::Mat &map);

    void prediction(double delta_timestep, double stdPos, double velocity, double orientation_rate, Particle robot);

    void associateParticlesWithRobot(Particle &robot);

    void updateTheWeights(cv::Mat &map, Particle &robot);

    void normalizeWeights(std::vector<double> &weights);

    void resampleParticles(int numberOfResample, double stddivPosition);


private:

    // True random numbers
    std::random_device rd;
    // Pseudo random numbers
    std::default_random_engine dre;


    double lidarMaxRange = 50.0;
    double lidarFov = 270.0;
    double noPoints = 70.0;
    double angle_increment = (lidarFov / noPoints) * (3.14 / 180.0);

    int numberOfParticles = 0;
    bool particleFilterInit = false;

    std::vector<double> weights;
    std::vector<double> normalizedWeights;

};

#endif // PARTICLEFILTER_H
