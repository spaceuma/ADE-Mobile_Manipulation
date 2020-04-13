#include "FastMarching.h"
#include <iostream>
#include <math.h>

using namespace FastMarching_lib;

void FastMarching::planPath(const std::vector<std::vector<double>> *costMap,
                            double mapResolution,
                            base::Waypoint iniPos,
                            base::Waypoint finalPos,
                            std::vector<base::Waypoint> *path)
{
    std::vector<int> goal(2, 0);
    std::vector<int> start(2, 0);

    goal[0] = (int)(finalPos.position[0] / mapResolution + 0.5);
    goal[1] = (int)(finalPos.position[1] / mapResolution + 0.5);

    start[0] = (int)(iniPos.position[0] / mapResolution + 0.5);
    start[1] = (int)(iniPos.position[1] / mapResolution + 0.5);

    std::vector<std::vector<double>> *TMap
        = new std::vector<std::vector<double>>;

    computeTMap(costMap, goal, start, TMap);

    std::vector<std::vector<double>> *pathPos
        = new std::vector<std::vector<double>>;

    computePathGDM(TMap, start, goal, 0.5, pathPos);

    path->resize(pathPos->size());
    (*path)[0].position[0] = mapResolution * (*pathPos)[0][0];
    (*path)[0].position[1] = mapResolution * (*pathPos)[0][1];

    for (int i = 1; i < path->size() - 1; i++)
    {
        (*path)[i].position[0] = mapResolution * (*pathPos)[i][0];
        (*path)[i].position[1] = mapResolution * (*pathPos)[i][1];
        (*path)[i].heading = atan2((*pathPos)[i + 1][1] - (*pathPos)[i - 1][1],
                                   (*pathPos)[i + 1][0] - (*pathPos)[i - 1][0]);
    }
    (*path)[0].heading = (*path)[1].heading;

    (*path)[path->size() - 1].position[0]
        = mapResolution * (*pathPos)[path->size() - 1][0];
    (*path)[path->size() - 1].position[1]
        = mapResolution * (*pathPos)[path->size() - 1][1];
    (*path)[path->size() - 1].heading = (*path)[path->size() - 2].heading;
}

void FastMarching::getShadowedCostMap(
    std::vector<std::vector<int>> &vvi_obstacle_map,
    double d_map_resolution,
    double d_min_distance,
    double d_max_distance,
    base::Waypoint finalPos)
{
    std::vector<int> vi_goal(2, 0);
    vi_goal[0] = (int)(finalPos.position[0] / d_map_resolution + 0.5);
    vi_goal[1] = (int)(finalPos.position[1] / d_map_resolution + 0.5);

    std::vector<std::vector<double>> vvd_clear_costmap;
    std::vector<std::vector<double>> vvd_obstacle_costmap;
    std::vector<std::vector<double>> *pvvd_clear_totalcostmap
        = new std::vector<std::vector<double>>;
    std::vector<std::vector<double>> *pvvd_obstacle_totalcostmap
        = new std::vector<std::vector<double>>;

    vvd_clear_costmap.clear();
    vvd_obstacle_costmap.clear();

    std::vector<double> clear_row, obstacle_row;
    double value;
    for (int j = 0; j < vvi_obstacle_map.size(); j++)
    {
        for (int i = 0; i < vvi_obstacle_map[0].size(); i++)
        {
            if ((i == 0) || (j == 0) || (i == vvi_obstacle_map[0].size() - 1)
                || (j == vvi_obstacle_map.size() - 1)
                || (sqrt(pow(((float)i - vi_goal[0]), 2)
                         + pow(((float)j - vi_goal[1]), 2))
                    > d_max_distance / d_map_resolution))
            {//Nodes further from a certain distance to goal are omitted
                clear_row.push_back(INFINITY);
                obstacle_row.push_back(INFINITY);
            }
            else
            {
                clear_row.push_back(1.0);
                if (vvi_obstacle_map[j][i] == 0)
                {//Obstacle inside sampling area
                    obstacle_row.push_back(1000000.0);
                }
                else
                {//Sampling Area
                    vvi_obstacle_map[j][i] = 1;
                    obstacle_row.push_back(1.0);
                }
            }
        }
        vvd_clear_costmap.push_back(clear_row);
        vvd_obstacle_costmap.push_back(obstacle_row);
        clear_row.clear();
        obstacle_row.clear();
    }

    computeEntireTMap(&vvd_clear_costmap, vi_goal, pvvd_clear_totalcostmap);
    computeEntireTMap(
        &vvd_obstacle_costmap, vi_goal, pvvd_obstacle_totalcostmap);

    for (int j = 0; j < vvi_obstacle_map.size(); j++)
    {
        for (int i = 0; i < vvi_obstacle_map[0].size(); i++)
        {
            float d_distToGoal = sqrt(pow(((float)i - vi_goal[0]), 2)
                         + pow(((float)j - vi_goal[1]), 2));
            if (d_distToGoal < d_min_distance / d_map_resolution)
	    {
	        vvi_obstacle_map[j][i] = 1; 
	    }
	    else if ((d_distToGoal < d_max_distance / d_map_resolution)&&(fabs((*pvvd_clear_totalcostmap)[j][i]
                     - (*pvvd_obstacle_totalcostmap)[j][i]) > d_map_resolution*sqrt(2)))
            {
                vvi_obstacle_map[j][i] = 0;
            }
        }
    }
}

// ToDo: this can be mixed with the function below
void FastMarching::computeEntireTMap(
    const std::vector<std::vector<double>> *costMap,
    std::vector<int> goal,
    std::vector<std::vector<double>> *TMap)
{
    int n = costMap->size();
    int m = costMap[0].size();

    std::vector<int> nodeTarget = goal;

    std::vector<std::vector<double>> *closedMap
        = new std::vector<std::vector<double>>;

    closedMap->resize(n, std::vector<double>(m));

    TMap->resize(n, std::vector<double>(m));

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        {
            if (isinf((*costMap)[i][j]))
            {
                (*closedMap)[i][j] = 1;
            }
            else
            {
                (*closedMap)[i][j] = 0;
            }
            (*TMap)[i][j] = INFINITY;
        }
    }

    (*closedMap)[nodeTarget[1]][nodeTarget[0]] = 1;

    (*TMap)[nodeTarget[1]][nodeTarget[0]] = 0;

    std::vector<double> *nbT = new std::vector<double>;
    std::vector<std::vector<int>> *nbNodes = new std::vector<std::vector<int>>;

    updateNode(nodeTarget, costMap, TMap, nbT, nbNodes, closedMap);

    while (nbT->size() > 0)
    {
        nodeTarget = (*nbNodes)[0];
        nbNodes->erase(nbNodes->begin());
        nbT->erase(nbT->begin());
        (*closedMap)[nodeTarget[1]][nodeTarget[0]] = 1;
        updateNode(nodeTarget, costMap, TMap, nbT, nbNodes, closedMap);
    }
}

void FastMarching::computeTMap(const std::vector<std::vector<double>> *costMap,
                               std::vector<int> goal,
                               std::vector<int> start,
                               std::vector<std::vector<double>> *TMap)
{
    int n = costMap->size();
    int m = costMap[0].size();

    std::vector<int> nodeTarget = goal;

    std::vector<std::vector<double>> *closedMap
        = new std::vector<std::vector<double>>;

    closedMap->resize(n, std::vector<double>(m));

    TMap->resize(n, std::vector<double>(m));

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        {
            if (isinf((*costMap)[i][j]))
            {
                (*closedMap)[i][j] = 1;
            }
            else
            {
                (*closedMap)[i][j] = 0;
            }
            (*TMap)[i][j] = INFINITY;
        }
    }

    (*closedMap)[nodeTarget[1]][nodeTarget[0]] = 1;

    (*TMap)[nodeTarget[1]][nodeTarget[0]] = 0;

    std::vector<double> *nbT = new std::vector<double>;
    std::vector<std::vector<int>> *nbNodes = new std::vector<std::vector<int>>;

    updateNode(nodeTarget, costMap, TMap, nbT, nbNodes, closedMap);

    while (nbT->size() > 0)
    {
        nodeTarget = (*nbNodes)[0];
        nbNodes->erase(nbNodes->begin());
        nbT->erase(nbT->begin());
        (*closedMap)[nodeTarget[1]][nodeTarget[0]] = 1;
        updateNode(nodeTarget, costMap, TMap, nbT, nbNodes, closedMap);

        if ((nodeTarget[0] == start[0]) && (nodeTarget[1] == start[1]))
        {
            break;
        }
    }
}

void FastMarching::updateNode(std::vector<int> nodeTarget,
                              const std::vector<std::vector<double>> *costMap,
                              std::vector<std::vector<double>> *TMap,
                              std::vector<double> *nbT,
                              std::vector<std::vector<int>> *nbNodes,
                              const std::vector<std::vector<double>> *closedMap)
{
    std::vector<int> nodeChild(2, 0);
    for (int i = 1; i < 4 + 1; i++)
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

        if ((*closedMap)[nodeChild[1]][nodeChild[0]] == 0)
        {
            double THor1 = (*TMap)[nodeChild[1]][nodeChild[0] + 1];
            double THor2 = (*TMap)[nodeChild[1]][nodeChild[0] - 1];
            double THor = std::min(THor1, THor2);

            double TVer1 = (*TMap)[nodeChild[1] + 1][nodeChild[0]];
            double TVer2 = (*TMap)[nodeChild[1] - 1][nodeChild[0]];
            double TVer = std::min(TVer1, TVer2);

            double T = getEikonal(
                THor, TVer, (*costMap)[nodeChild[1]][nodeChild[0]]);
            if (isinf((*TMap)[nodeChild[1]][nodeChild[0]]))
            {
                int index = getInsertIndex(nbT, T);
                std::vector<double>::iterator indexT = nbT->begin() + index;
                nbT->insert(indexT, T);
                std::vector<std::vector<int>>::iterator indexN
                    = nbNodes->begin() + index;
                nbNodes->insert(indexN, nodeChild);
                (*TMap)[nodeChild[1]][nodeChild[0]] = T;
            }
            else if (T < (*TMap)[nodeChild[1]][nodeChild[0]])
            {
                double tempT = (*TMap)[nodeChild[1]][nodeChild[0]];
                int index = getInsertIndex(nbT, tempT);
                for (int i = index; i < nbNodes->size(); i++)
                {
                    if ((*nbNodes)[i] == nodeChild)
                    {
                        index = i;
                        break;
                    }
                }

                nbT->erase(nbT->begin() + index);
                nbNodes->erase(nbNodes->begin() + index);

                index = getInsertIndex(nbT, T);
                std::vector<double>::iterator indexT = nbT->begin() + index;
                nbT->insert(indexT, T);
                std::vector<std::vector<int>>::iterator indexN
                    = nbNodes->begin() + index;
                nbNodes->insert(indexN, nodeChild);
                (*TMap)[nodeChild[1]][nodeChild[0]] = T;
            }
        }
    }
}

double FastMarching::getEikonal(double THor, double TVer, double cost)
{
    if (isinf(THor))
        if (isinf(TVer))
            return INFINITY;
        else
            return TVer + cost;
    if (isinf(TVer))
        return THor + cost;
    else if (cost < abs(THor - TVer))
        return std::min(THor, TVer) + cost;
    else
        return 0.5
               * (THor + TVer + sqrt(2 * pow(cost, 2) - pow(THor - TVer, 2)));
}

int FastMarching::getInsertIndex(std::vector<double> *nbT, double T)
{
    int i;
    for (i = 0; i < nbT->size(); i++)
        if (T <= nbT->at(i)) break;
    return i;
}

void FastMarching::computePathGDM(const std::vector<std::vector<double>> *TMap,
                                  std::vector<int> initNode,
                                  std::vector<int> endNode,
                                  double tau,
                                  std::vector<std::vector<double>> *path)
{
    std::vector<double> auxVector;
    auxVector.push_back((double)initNode[0]);
    auxVector.push_back((double)initNode[1]);

    path->push_back(auxVector);

    std::vector<int> nearNode = {0, 0};

    std::vector<std::vector<double>> *G1 = new std::vector<std::vector<double>>;
    std::vector<std::vector<double>> *G2 = new std::vector<std::vector<double>>;

    G1->resize(TMap->size(), std::vector<double>(TMap[0].size()));
    G2->resize(TMap->size(), std::vector<double>(TMap[0].size()));

    for (int k = 0; k < (int)15000 / tau; k++)
    {
        computeGradient(TMap, path->at(path->size() - 1), G1, G2);
        double dx = getInterpolatedPoint(path->at(path->size() - 1), G1);
        double dy = getInterpolatedPoint(path->at(path->size() - 1), G2);

        if (isnan(dx) || isnan(dy))
        {
            try
            {
                nearNode[0] = (int)(*path)[path->size() - 1][0] + 0.5;
                nearNode[1] = (int)(*path)[path->size() - 1][1] + 0.5;
                while (isinf((*TMap)[nearNode[1]][nearNode[0]]))
                {
                    path->erase(path->end());
                    nearNode[0] = (int)(*path)[path->size() - 1][0] + 0.5;
                    nearNode[1] = (int)(*path)[path->size() - 1][1] + 0.5;
                }

                if (path->size() > 0)
                {
                    std::vector<double> distVector;
                    distVector.push_back((*path)[path->size() - 1][0]
                                         - (double)nearNode[0]);
                    distVector.push_back((*path)[path->size() - 1][1]
                                         - (double)nearNode[1]);
                    while (sqrt(pow(distVector[0], 2) + pow(distVector[1], 2))
                           < 1)
                    {
                        path->erase(path->end());
                        if (path->size() == 0) break;
                        distVector[0] = (*path)[path->size() - 1][0]
                                        - (double)nearNode[0];
                        distVector[1] = (*path)[path->size() - 1][1]
                                        - (double)nearNode[1];
                    }
                }

                auxVector[0] = (double)nearNode[0];
                auxVector[1] = (double)nearNode[1];
                path->push_back(auxVector);

                double currentT = (*TMap)[nearNode[1]][nearNode[0]];
                for (int i = 1; i < 5; i++)
                {
                    std::vector<int> nodeChild(2, 0);
                    if (i == 1)
                    {
                        nodeChild[0] = nearNode[0];
                        nodeChild[1] = nearNode[1] - 1;
                    }
                    else if (i == 2)
                    {
                        nodeChild[0] = nearNode[0];
                        nodeChild[1] = nearNode[1] + 1;
                    }
                    else if (i == 3)
                    {
                        nodeChild[0] = nearNode[0] - 1;
                        nodeChild[1] = nearNode[1];
                    }
                    else if (i == 4)
                    {
                        nodeChild[0] = nearNode[0] + 1;
                        nodeChild[1] = nearNode[1];
                    }

                    if ((*TMap)[nodeChild[1]][nodeChild[0]] < currentT)
                    {
                        currentT = (*TMap)[nodeChild[1]][nodeChild[0]];
                        dx = (nearNode[0] - nodeChild[0]) / tau;
                        dy = (nearNode[1] - nodeChild[1]) / tau;
                    }
                }
            }
            catch (std::exception &e)
            {
                std::cout << "Exception was caught during GDM, with message "
                          << e.what() << std::endl;
                break;
            }
        }

        double d = sqrt(pow(dx, 2) + pow(dy, 2));
        if (d < 0.01)
        {
            double dnx = dx / d;
            double dny = dy / d;

            auxVector[0] = (*path)[path->size() - 1][0] - tau * dnx;
            auxVector[1] = (*path)[path->size() - 1][1] - tau * dny;
            path->push_back(auxVector);
        }
        else
        {
            dx = dx / d;
            dy = dy / sqrt(pow(dx, 2) + pow(dy, 2));

            auxVector[0] = (*path)[path->size() - 1][0] - tau * dx;
            auxVector[1] = (*path)[path->size() - 1][1] - tau * dy;
            path->push_back(auxVector);
        }

        auxVector[0] = (*path)[path->size() - 1][0] - (double)endNode[0];
        auxVector[1] = (*path)[path->size() - 1][1] - (double)endNode[1];
        if (sqrt(pow(auxVector[0], 2) + pow(auxVector[1], 2)) < 1.5) break;
    }

    auxVector[0] = (double)endNode[0];
    auxVector[1] = (double)endNode[1];
    path->push_back(auxVector);
}

void FastMarching::computeGradient(const std::vector<std::vector<double>> *TMap,
                                   std::vector<double> point,
                                   std::vector<std::vector<double>> *Gnx,
                                   std::vector<std::vector<double>> *Gny)
{
    int n = TMap->size();
    int m = (*TMap)[0].size();

    int jmax, imax, jmin, imin;
    if (point.size() == 0)
    {
        jmax = n;
        imax = m;

        jmin = 0;
        imin = 0;
    }
    else
    {
        jmax = std::min(n, (int)(point[1] + 3.5));
        imax = std::min(m, (int)(point[0] + 3.5));

        jmin = std::max(0, (int)(point[1] - 2.5));
        imin = std::max(0, (int)(point[0] - 2.5));
    }

    std::vector<std::vector<double>> Gx, Gy;

    Gx.resize(n, std::vector<double>(m));
    Gy.resize(n, std::vector<double>(m));

    for (int i = imin; i < imax; i++)
        for (int j = jmin; j < jmax; j++)
        {
            if (j == 0)
                Gy[0][i] = (*TMap)[1][i] - (*TMap)[0][i];
            else
            {
                if (j == n - 1)
                    Gy[j][i] = (*TMap)[j][i] - (*TMap)[j - 1][i];
                else
                {
                    if (isinf((*TMap)[j + 1][i]))
                    {
                        if (isinf((*TMap)[j - 1][i]))
                            Gy[j][i] = 0;
                        else
                            Gy[j][i] = (*TMap)[j][i] - (*TMap)[j - 1][i];
                    }
                    else
                    {
                        if (isinf((*TMap)[j - 1][i]))
                            Gy[j][i] = (*TMap)[j + 1][i] - (*TMap)[j][i];
                        else
                            Gy[j][i]
                                = ((*TMap)[j + 1][i] - (*TMap)[j - 1][i]) / 2;
                    }
                }
            }

            if (i == 0)
                Gx[j][0] = (*TMap)[j][1] - (*TMap)[j][0];
            else
            {
                if (i == m - 1)
                    Gx[j][i] = (*TMap)[j][i] - (*TMap)[j][i - 1];
                else
                {
                    if (isinf((*TMap)[j][i + 1]))
                    {
                        if (isinf((*TMap)[j][i - 1]))
                            Gx[j][i] = 0;
                        else
                            Gx[j][i] = (*TMap)[j][i] - (*TMap)[j][i - 1];
                    }
                    else
                    {
                        if (isinf((*TMap)[j][i - 1]))
                            Gx[j][i] = (*TMap)[j][i + 1] - (*TMap)[j][i];
                        else
                            Gx[j][i]
                                = ((*TMap)[j][i + 1] - (*TMap)[j][i - 1]) / 2;
                    }
                }
            }

            (*Gnx)[j][i] = Gx[j][i] / sqrt(pow(Gx[j][i], 2) + pow(Gy[j][i], 2));
            (*Gny)[j][i] = Gy[j][i] / sqrt(pow(Gx[j][i], 2) + pow(Gy[j][i], 2));
        }
}

double FastMarching::getInterpolatedPoint(
    std::vector<double> point,
    const std::vector<std::vector<double>> *mapI)
{
    int i = (int)point[0];
    int j = (int)point[1];
    double a = point[0] - double(i);
    double b = point[1] - double(j);

    int n = mapI->size();
    int m = (*mapI)[0].size();

    double I;

    if (i == m)
    {
        if (j == n)
            I = (*mapI)[j][i];
        else
            I = b * (*mapI)[j + 1][i] + (1 - b) * (*mapI)[j][i];
    }
    else
    {
        if (j == n)
            I = a * (*mapI)[j][i + 1] + (1 - a) * (*mapI)[j][i];
        else
        {
            double a00 = (*mapI)[j][i];
            double a10 = (*mapI)[j][i + 1] - (*mapI)[j][i];
            double a01 = (*mapI)[j + 1][i] - (*mapI)[j][i];
            double a11 = (*mapI)[j + 1][i + 1] + (*mapI)[j][i]
                         - (*mapI)[j][i + 1] - (*mapI)[j + 1][i];
            if (a == 0)
            {
                if (b == 0)
                    I = a00;
                else
                    I = a00 + a01 * b;
            }
            else
            {
                if (b == 0)
                    I = a00 + a10 * a;
                else
                    I = a00 + a10 * a + a01 * b + a11 * a * b;
            }
        }
    }

    return I;
}
