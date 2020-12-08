#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

#include "particlefilter.h"

using namespace std;

int main()
{

    cv::Mat map;
    map = cv::imread("floor_plan.png");

    cv::resize(map, map, cv::Size(map.cols * 3, map.rows * 3), 0, 0, cv::INTER_NEAREST);

    char key;

    ParticleFilter pf(2000);

    double std[] = {2,2,2};

    Particle robot = {0, int(map.cols/2-60), int(map.rows/2-10), 0, 1.0};

    for(auto & range : robot.lidar){
        range = 10.0;
    }

    cv::imshow("Map", map);

    // keys to control robot
    const int key_left = 81;
    const int key_up = 82;
    const int key_down = 84;
    const int key_right = 83;
    const int key_esc = 27;

    // set speed and direction for the robot
    float speed = 0.0;
    double dir = 0.0;
    double oldDir = 0.0;
    double orientation_rate = 0.0;

    bool run = false;

    int numberOfResample = 300;
    double stdPos = 10.0;
    int counterResample = 0;

    while(true){


        if(!pf.particleFilterInitialized()){
            pf.initializeParticleFilter(0,0,0, std, map);
        }


        int key = cv::waitKey(1);
        bool keyPressed = false;


        // robot controller with arrow keys
        if ((key == key_up)){
          speed += 0.5;
          keyPressed = true;
        }
        else if ((key == key_down)){
          speed -= 0.5;
          keyPressed = true;
        }
        else if ((key == key_right)){
          dir += 10.0;
          keyPressed = true;
        }
        else if ((key == key_left)){
          dir -= 10.0;
          keyPressed = true;
        }
        else {
          // slow down
               //speed *= 0.99;
               //dir *= 0.1;
               keyPressed = false;
        }



//        if(dir != oldDir){
//            dir -= oldDir;
//        }

        //std::cout << "oldDir: " << oldDir << std::endl;
        //std::cout << "Dir: " << dir << std::endl;


        if(key == 'r'){
            run = true;
        }

        if(run == true){

            map = cv::imread("floor_plan.png");

            cv::resize(map, map, cv::Size(map.cols * 3, map.rows * 3), 0, 0, cv::INTER_NEAREST);

           // cv::resize(map, map, cv::Size(0,0), 5,5 );


            orientation_rate = dir - oldDir;


            if (keyPressed) {
                pf.prediction(1, 2.0, speed, orientation_rate, robot);
                pf.moveRobot(1, 2.0, speed, orientation_rate, robot);
            }

            oldDir = dir;

            pf.lidarParticles(map);

            pf.robotLidar(robot, map);

            // Draw
            pf.drawParticles(map);

            //pf.drawLidarParticles(map);

            pf.drawRobotParticle(robot, map);

           // pf.updateTheWeights(map, robot);

            // Find the good particles when compared to the robots view
            pf.associateParticlesWithRobot(robot);


            if(key == 's'){
                pf.resampleParticles(numberOfResample, stdPos);
                if(numberOfResample > 50){
                    if (stdPos != 2.0) {
                      stdPos -= 1.0;
                    }
                    numberOfResample -= 30;

                } else {
                    numberOfResample = 50;
                    stdPos = 2.0;
                }
                counterResample++;
                std::cout << "counterResample: " << counterResample << std::endl;
                std::cout << "std: " << stdPos << std::endl;
            }


            pf.drawLidarParticles(map);

            //cv::resize(resample, resample, cv::Size(0,0), 5,5);
            cv::imshow("Map", map);


        }
    }

    return 0;
}
