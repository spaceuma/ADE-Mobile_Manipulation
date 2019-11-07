#include "FastMarching.h"
#include <math.h>
#include <iostream>

using namespace FastMarching_lib;

std::vector<int> BiFastMarching::computeTMap(const std::vector<std::vector<double>> * currentCostMap,
					     std::vector<int> goal,
	 				     std::vector<int> start,
	 				     std::vector<std::vector<double>> * TMapGoal,
	 				     std::vector<std::vector<double>> * TMapStart)
{
	int n = currentCostMap->size();
	int m = currentCostMap[0].size();

	std::vector<int> nodeTargetGoal = goal;
	std::vector<int> nodeTargetStart = start;

	std::vector<std::vector<double>> * closedMapGoal = new std::vector<std::vector<double>>;
	std::vector<std::vector<double>> * closedMapStart = new std::vector<std::vector<double>>;

	closedMapGoal->resize(n, std::vector<double>(m));
	closedMapStart->resize(n, std::vector<double>(m));

	TMapGoal->resize(n, std::vector<double>(m));
	TMapStart->resize(n, std::vector<double>(m));

	for(int i = 0; i < n; i++)
	{
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
	
	(*closedMapGoal)[nodeTargetGoal[1]][nodeTargetGoal[0]] = 1;
	(*closedMapStart)[nodeTargetStart[1]][nodeTargetStart[0]] = 1;

	(*TMapGoal)[nodeTargetGoal[1]][nodeTargetGoal[0]] = 0;
	(*TMapStart)[nodeTargetStart[1]][nodeTargetStart[0]] = 0;

	std::vector<double> * nbTGoal = new std::vector<double>;
	std::vector<std::vector<int>> * nbNodesGoal = new std::vector<std::vector<int>>;
	std::vector<double> * nbTStart = new std::vector<double>;
	std::vector<std::vector<int>> * nbNodesStart = new std::vector<std::vector<int>>;

	updateNode(nodeTargetGoal, currentCostMap, TMapGoal, nbTGoal, nbNodesGoal, closedMapGoal);
	updateNode(nodeTargetStart, currentCostMap, TMapStart, nbTStart, nbNodesStart, closedMapStart);

	std::vector<int> nodeJoin;

	while((nbTGoal->size() > 0) || (nbTStart->size() > 0))
	{
		if(nbTGoal->size() > 0)
		{
			nodeTargetGoal = (*nbNodesGoal)[0];
			nbNodesGoal->erase(nbNodesGoal->begin());
			nbTGoal->erase(nbTGoal->begin());
			(*closedMapGoal)[nodeTargetGoal[1]][nodeTargetGoal[0]] = 1;
			updateNode(nodeTargetGoal, currentCostMap, TMapGoal, nbTGoal, nbNodesGoal, closedMapGoal);
		}
		if(nbTStart->size() > 0)
		{
			nodeTargetStart = (*nbNodesStart)[0];
			nbNodesStart->erase(nbNodesStart->begin());
			nbTStart->erase(nbTStart->begin());
			(*closedMapStart)[nodeTargetStart[1]][nodeTargetStart[0]] = 1;
			updateNode(nodeTargetStart, currentCostMap, TMapStart, nbTStart, nbNodesStart, closedMapStart);
		}
		if((*closedMapStart)[nodeTargetGoal[1]][nodeTargetGoal[0]] == 1)
		{
			nodeJoin = nodeTargetGoal;
			break;
		}
		if((*closedMapGoal)[nodeTargetStart[1]][nodeTargetStart[0]] == 1)
		{
			nodeJoin = nodeTargetStart;
			break;
		}
	}

	return nodeJoin;
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

		if((*closedMap)[nodeChild[1]][nodeChild[0]] == 0)
		{
			double THor1 = (*TMap)[nodeChild[1]][nodeChild[0] + 1];
			double THor2 = (*TMap)[nodeChild[1]][nodeChild[0] - 1];
			double THor = std::min(THor1, THor2);

			double TVer1 = (*TMap)[nodeChild[1] + 1][nodeChild[0]];
			double TVer2 = (*TMap)[nodeChild[1] - 1][nodeChild[0]];
			double TVer = std::min(TVer1, TVer2);

			double T = getEikonal(THor, TVer, (*currentCostMap)[nodeChild[1]][nodeChild[0]]);
			if(isinf((*TMap)[nodeChild[1]][nodeChild[0]]))
			{
				int index = getInsertIndex(nbT, T);
				std::vector<double>::iterator indexT = nbT->begin() + index;
				nbT->insert(indexT, T);
				std::vector<std::vector<int>>::iterator indexN = nbNodes->begin() + index;
				nbNodes->insert(indexN, nodeChild);
				(*TMap)[nodeChild[1]][nodeChild[0]] = T;
			}
			else
				if(T < (*TMap)[nodeChild[1]][nodeChild[0]])
				{
					double tempT = (*TMap)[nodeChild[1]][nodeChild[0]];
					int index = getInsertIndex(nbT, tempT);
					for(int i = index; i < nbNodes->size(); i++)
						if((*nbNodes)[i] == nodeChild)
						{
							index = i;
							break;
						}

					nbT->erase(nbT->begin()+index);
					nbNodes->erase(nbNodes->begin()+index);

					index = getInsertIndex(nbT, T);
					std::vector<double>::iterator indexT = nbT->begin() + index;
					nbT->insert(indexT, T);
					std::vector<std::vector<int>>::iterator indexN = nbNodes->begin() + index;
					nbNodes->insert(indexN, nodeChild);
					(*TMap)[nodeChild[1]][nodeChild[0]] = T;
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

int BiFastMarching::getInsertIndex(std::vector<double> * nbT, double T)
{
	int i;
	for(i = 0; i < nbT->size(); i++)
		if(T < nbT->at(i))
			break;
	return i;
}
