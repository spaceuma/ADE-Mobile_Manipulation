#include <gtest/gtest.h>
#include "MotionPlan.h"
#include "readMatrixFile.h"
#include "MobileManipMap.h"
#include "FastMarching.h"
#include <ctime>
#include <fstream>

using namespace FastMarching_lib;

TEST(MMMotionPlanTest, roverbaseplanning){
  MotionPlan dummyPlan;
  MobileManipMap dummyMap;
  double res = 0.04; //meters
  double n_row, n_col;
  std::vector<double> vector_elevationData;
  base::Waypoint roverPos, samplePos;
  std::vector<base::Waypoint> *roverPath = new std::vector<base::Waypoint>;

  RoverGuidance_Dem* dummyDem = new RoverGuidance_Dem;
  readMatrixFile("test/unit/data/ColmenarRocks_smaller_4cmDEM.csv", res, n_row, n_col, vector_elevationData);
  std::cout << "The size is " << vector_elevationData.size() << std::endl;
  double dummyArray[(int)n_row*(int)n_col];
  dummyDem->p_heightData_m = dummyArray;
  for (uint i = 0; i<vector_elevationData.size(); i++)
  {
    dummyDem->p_heightData_m[i] = vector_elevationData[i];
  }
  
  dummyDem->cols = n_col;
  dummyDem->rows = n_row;
  dummyDem->nodeSize_m = res;
    dummyMap.setRGDem((* dummyDem));
    BiFastMarching dummyFM;
    clock_t begin = clock();

    roverPos.position[0] = 1.0;
    roverPos.position[1] = 1.2;
    roverPos.heading = 0;

    samplePos.position[0] = 1.0;
    samplePos.position[1] = 6.4;
    samplePos.position[2] = 1.1;
    samplePos.heading = 0;

    std::vector<int> *nodeJoin = new std::vector<int>;
    std::vector<std::vector<double>> costMap;
    dummyMap.getCostMap(costMap);

    clock_t ini2D = clock();
    dummyFM.planPath(&costMap, dummyDem->nodeSize_m, roverPos, samplePos, roverPath);
    clock_t end2D = clock();    
    double t = double(end2D - ini2D) / CLOCKS_PER_SEC;
    std::cout << "Elapsed execution time planning 2D: " << t << std::endl;

    std::ofstream pathFile;

    pathFile.open("test/unit/data/results/path.txt");

    for (int j = 0; j < roverPath->size(); j++)
    {
        pathFile << (*roverPath)[j].position[0] << " " << (*roverPath)[j].position[1] << "\n";
    }

    pathFile.close();


    /*std::vector<std::vector<double>> *TMapGoal = new std::vector<std::vector<double>>;
    std::vector<std::vector<double>> *TMapStart = new std::vector<std::vector<double>>;

    std::vector<int> *nodeJoin = new std::vector<int>;
    std::vector<std::vector<double>> costMap;
    dummyMap.getCostMap(costMap); 
    dummyFM.computeTMap(&costMap, goal, start, TMapGoal, TMapStart, nodeJoin);

    std::cout << "Node join: [" << (*nodeJoin)[0] << ", " << (*nodeJoin)[1] << "]" << std::endl;
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Elapsed execution time computeT: " << elapsed_secs << std::endl;

    std::ofstream TMapGoalFile;
    std::ofstream TMapStartFile;

    TMapGoalFile.open("test/unit/data/results/TMapGoal.txt");
    TMapStartFile.open("test/unit/data/results/TMapStart.txt");

    for (int j = 0; j < costMap.size(); j++)
    {
        for (int i = 0; i < costMap[0].size(); i++)
        {
            TMapGoalFile << (*TMapGoal)[j][i] << " ";
            TMapStartFile << (*TMapStart)[j][i] << " ";
        }
        TMapGoalFile << "\n";
        TMapStartFile << "\n";
    }

    TMapGoalFile.close();
    TMapStartFile.close();*/


}
