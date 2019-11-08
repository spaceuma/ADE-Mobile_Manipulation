#include <gtest/gtest.h>
#include <fstream>
#include "FastMarching.h"
#include "Waypoint.hpp"
#include "WaypointNavigation.hpp"
#include <math.h>
#include <ctime>

using namespace FastMarching_lib;

std::vector<std::vector<double>> readMatrixFile(std::string cost_map_file)
{
    std::cout << "Reading cost_map " << cost_map_file<<std::endl;
    std::vector<std::vector<double>> cost_map_matrix;
    std::string line;
    std::ifstream e_file(cost_map_file.c_str(), std::ios::in);
    double n_row = 0, n_col = 0;
    std::vector<double> row;

    if (e_file.is_open())
    {
        while (std::getline(e_file, line))
        {
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ' '))
            {
                double val;
                std::stringstream numeric_value(cell);
                numeric_value >> val;
                row.push_back(val);
                n_col++;
            }
            cost_map_matrix.push_back(row);
            row.clear();
            n_row++;
        }
        e_file.close();

        n_col /= n_row;
	std::cout << "Cost map of " << n_col << " x " << n_row << " loaded."<<std::endl;
    }
    else
    {
	    std::cout << "Problem opening the cost_map file"<<std::endl;
        return cost_map_matrix;
    }
    return cost_map_matrix;
}


TEST(FastMarchingTests, computingTMap)
{
	BiFastMarching dummyFM;
	
	std::vector<std::vector<double>> * costMap = new std::vector<std::vector<double>>;
	std::string costMapFile = "../data/dummyCostMap.txt";
	(*costMap) = readMatrixFile(costMapFile);
	for(int i = 0; i < costMap->size(); i++)
		for(int j = 0; j < (*costMap)[0].size(); j++)
			if(i == 0 || j == 0 || i == costMap->size()-1 || j == (*costMap)[0].size()-1)
				(*costMap)[i][j] = INFINITY;

	clock_t begin = clock();		

	std::vector<int> goal;
	std::vector<int> start;

	goal.push_back(30);
	goal.push_back(5);

	start.push_back(65);
	start.push_back(94);

	std::vector<std::vector<double>> * TMapGoal = new std::vector<std::vector<double>>;
	std::vector<std::vector<double>> * TMapStart = new std::vector<std::vector<double>>;

	std::vector<int> * nodeJoin = new std::vector<int>;

	dummyFM.computeTMap(costMap, goal, start, TMapGoal, TMapStart, nodeJoin);

	std::cout<<"Node join: ["<<(*nodeJoin)[0]<<", "<<(*nodeJoin)[1]<<"]"<<std::endl;
	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	std::cout<<"Elapsed execution time computeT: "<<elapsed_secs<<std::endl;

	std::ofstream TMapGoalFile;
	std::ofstream TMapStartFile;

	TMapGoalFile.open("../data/results/TMapGoal.txt");
	TMapStartFile.open("../data/results/TMapStart.txt");

	for(int j = 0; j < costMap->size(); j++)
	{
		for(int i = 0; i < costMap[0].size(); i++)
		{
			TMapGoalFile << (*TMapGoal)[j][i] << " ";
			TMapStartFile << (*TMapStart)[j][i] << " ";
		}
		TMapGoalFile << "\n";
		TMapStartFile << "\n";
	}

	TMapGoalFile.close();
	TMapStartFile.close();

  	//EXPECT_EQ(INFINITY,dummyFM.getEikonal(a,b,c));

}

TEST(FastMarchingTests, planningRoverPath)
{
	BiFastMarching dummyFM;
	
	std::vector<std::vector<double>> * costMap = new std::vector<std::vector<double>>;
	std::string costMapFile = "../data/dummyCostMap.txt";
	(*costMap) = readMatrixFile(costMapFile);
	for(int i = 0; i < costMap->size(); i++)
		for(int j = 0; j < (*costMap)[0].size(); j++)
			if(i == 0 || j == 0 || i == costMap->size()-1 || j == (*costMap)[0].size()-1)
				(*costMap)[i][j] = INFINITY;

	clock_t begin = clock();	
	double mapResolution = 1;
	base::Waypoint roverPos, samplePos;

	roverPos.position[0] = 30;
	roverPos.position[1] = 5;
	roverPos.heading = 0;

	samplePos.position[0] = 65;
	samplePos.position[1] = 94;
	samplePos.heading = 0;

	std::vector<base::Waypoint> * roverPath = new std::vector<base::Waypoint>;

	dummyFM.planRoverPath(costMap, mapResolution, roverPos, samplePos, roverPath);	

	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	std::cout<<"Elapsed execution time planning: "<<elapsed_secs<<std::endl;

	std::ofstream pathFile;

	pathFile.open("../data/results/path.txt");

	for(int j = 0; j < roverPath->size(); j++)
	{
		pathFile << (*roverPath)[j].position[0] <<" " << (*roverPath)[j].position[1] << "\n";
	}

	pathFile.close();

}
