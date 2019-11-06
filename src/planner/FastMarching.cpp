#include "FastMarching.h"
#include <math.h>
#include <iostream>

using namespace FastMarching_lib;

void BiFastMarching::computeTMap(const std::vector<std::vector<double>> * currentCostMap,
				    std::vector<int> goal,
				    std::vector<int> start,
				    std::vector<std::vector<double>> * TMapGoal,
				    std::vector<std::vector<double>> * TMapStart)
{
	int n = currentCostMap->size();
	int m = currentCostMap[0].size();

	std::vector<std::vector<double>> * closedMapGoal;
	std::vector<std::vector<double>> * closedMapStart;

	closedMapGoal->resize(n);
	closedMapStart->resize(n);

	TMapGoal->resize(n);
	TMapStart->resize(n);

	for(int i = 0; i < n; i++)
	{
		(*closedMapGoal)[i].resize(m);
		(*closedMapStart)[i].resize(m);

		(*TMapGoal)[i].resize(m);
		(*TMapStart)[i].resize(m);

		for(int j = 0; j < m; j++)
		{
			if(isinf((*currentCostMap)[i][j]))
			{
				(*closedMapGoal)[i][j] = 1;
				(*closedMapStart)[i][j] = 1;
			}
			else
			{
				(*closedMapGoal)[i][j] = 0;
				(*closedMapStart)[i][j] = 0;
			}

			(*TMapGoal)[i][j] = INFINITY;
			(*TMapStart)[i][j] = INFINITY;

		}
	}
	
	(*closedMapGoal)[goal[0]][goal[1]] = 1;
	(*closedMapStart)[start[0]][start[1]] = 1;

	(*TMapGoal)[goal[0]][goal[1]] = 0;
	(*TMapStart)[start[0]][start[1]] = 0;

	std::vector<double> * nbTGoal;
	std::vector<std::vector<int>> * nbNodesGoal;
	std::vector<double> * nbTStart;
	std::vector<std::vector<int>> * nbNodesStart;

	updateNode(goal, currentCostMap, TMapGoal, nbTGoal, nbNodesGoal, closedMapGoal);
	updateNode(goal, currentCostMap, TMapStart, nbTStart, nbNodesStart, closedMapStart);

	std::vector<int> nodeJoin(2, 0);
	//return nodeJoin;
}


void BiFastMarching::updateNode(std::vector<int> nodeTarget,
		const std::vector<std::vector<double>> * currentCostMap,
		std::vector<std::vector<double>> * TMap,
		std::vector<double> * nbT,
		std::vector<std::vector<int>> * nbNodes,
		const std::vector<std::vector<double>> * closedMap)
{
	std::vector<int> nodeChild(2,0);
	for(int i = 1; i < 4+1; i++)
	{
		switch (i)
		{
			case 1:
				nodeChild[0] = nodeTarget[0];
				nodeChild[1] = nodeTarget[1] - 1;
				break;
			case 2:
				nodeChild[0] = nodeTarget[0];
				nodeChild[1] = nodeTarget[1] + 1;
				break;
			case 3:
				nodeChild[0] = nodeTarget[0] - 1;
				nodeChild[1] = nodeTarget[1];
				break;
			case 4:
				nodeChild[0] = nodeTarget[0] + 1;
				nodeChild[1] = nodeTarget[1];
				break;
		}

		if((*closedMap)[nodeChild[0]][nodeChild[1]] == 0)
		{
			double THor1 = (*TMap)[nodeChild[0] + 1][nodeChild[1]];
			double THor2 = (*TMap)[nodeChild[0] - 1][nodeChild[1]];
			double THor = std::min(THor1, THor2);

			double TVer1 = (*TMap)[nodeChild[0]][nodeChild[1] + 1];
			double TVer2 = (*TMap)[nodeChild[0]][nodeChild[1] - 1];
			double TVer = std::min(TVer1, TVer2);

			double T = getEikonal(THor, TVer, (*currentCostMap)[nodeChild[0]][nodeChild[1]]);
			if(isinf((*TMap)[nodeChild[0]][nodeChild[1]]))
			{
				int index = bisect(nbT, T);
				std::vector<double>::iterator indexT = nbT->begin() + index;
				nbT->insert(indexT, T);
				std::vector<std::vector<int>>::iterator indexN = nbNodes->begin() + index;
				nbNodes->insert(indexN, nodeChild);
				(*TMap)[nodeChild[0]][nodeChild[1]] = T;
			}
			else
				if(T < (*TMap)[nodeChild[0]][nodeChild[1]])
				{
					double tempT = (*TMap)[nodeChild[0]][nodeChild[1]];
					int index = bisect(nbT, tempT);
					for(int i = index; i < nbNodes->size(); i++)
						if((*nbNodes)[i] == nodeChild)
						{
							index = i;
							break;
						}

					nbT->erase(nbT->begin()+index);
					nbNodes->erase(nbNodes->begin()+index);

					index = bisect(nbT, T);
					std::vector<double>::iterator indexT = nbT->begin() + index;
					nbT->insert(indexT, T);
					std::vector<std::vector<int>>::iterator indexN = nbNodes->begin() + index;
					nbNodes->insert(indexN, nodeChild);
					(*TMap)[nodeChild[0]][nodeChild[1]] = T;
				}
		}
	}
}
 
double BiFastMarching::getEikonal(double THor, double TVer, double cost)
{
	if(isinf(THor))
		if(isinf(TVer))
			return INFINITY;
		else
			return TVer + cost;
	if(isinf(TVer))
		return THor + cost;
	else
		if(cost < abs(THor-TVer))
			return std::min(THor, TVer) + cost;
		else
			return 0.5*(THor + TVer + sqrt(2*pow(cost,2) - pow(THor-TVer,2)));
}

int BiFastMarching::bisect(std::vector<double> * nbT, double T)
{
	int i;
	for(i = 0; i < nbT->size(); i++)
		if(T < i)
			break;
	return i;
}
