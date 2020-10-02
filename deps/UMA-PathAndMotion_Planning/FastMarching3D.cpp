#include "FastMarching.h"
#include <iostream>
#include <math.h>

using namespace FastMarching_lib;

FastMarching3D::FastMarching3D(double _waypointDistance)
{
    this->waypointDistance = _waypointDistance;
}

FastMarching3D::~FastMarching3D()
{
    ;
}

bool FastMarching3D::planPath(
    const std::vector<std::vector<std::vector<double>>> *costMap3D,
    double mapResolution,
    double zResolution,
    base::Waypoint iniPos,
    base::Waypoint finalPos,
    std::vector<base::Waypoint> *path3D,
    unsigned int ui_max_iter)
{
    std::vector<int> goal(3, 0);
    std::vector<int> start(3, 0);

    goal[0] = (int)(finalPos.position[0] / mapResolution + 0.5);
    goal[1] = (int)(finalPos.position[1] / mapResolution + 0.5);
    goal[2] = (int)(finalPos.position[2] / zResolution + 0.5);

    start[0] = (int)(iniPos.position[0] / mapResolution + 0.5);
    start[1] = (int)(iniPos.position[1] / mapResolution + 0.5);
    start[2] = (int)(iniPos.position[2] / zResolution + 0.5);

    std::vector<std::vector<std::vector<double>>> *TMap
        = new std::vector<std::vector<std::vector<double>>>;

    if(!computeTMap(costMap3D, goal, start, TMap, ui_max_iter))
    {
        std::cout << " \033[35m[--WARNING-] [FastMarching3D::planPath()] \033[0m 3D Total Cost Map computation failed" << std::endl;
        return false; //UNREACHABLE
    }

    //std::cout << "3d Tmap is computed" << std::endl;
    std::vector<std::vector<double>> *path
        = new std::vector<std::vector<double>>;

    if(!computePathGDM(TMap, start, goal, waypointDistance, path, ui_max_iter))
    { 
        std::cout << " \033[35m[--WARNING-] [FastMarching3D::planPath()] \033[0m 3D Path computation from Total Cost Map failed" << std::endl;
	return false;
    }

    path3D->resize(path->size());

    for (int i = 0; i < path->size(); i++)
    {
        (*path3D)[i].position[0] = mapResolution * (*path)[i][0];
        (*path3D)[i].position[1] = mapResolution * (*path)[i][1];
        (*path3D)[i].position[2] = zResolution * (*path)[i][2];
    }
    return true;
}

bool FastMarching3D::computeTMap(
    const std::vector<std::vector<std::vector<double>>> *costMap3D,
    std::vector<int> goal,
    std::vector<int> start,
    std::vector<std::vector<std::vector<double>>> *TMap,
    unsigned int ui_max_iter)
{
    int n = costMap3D->size();
    int m = (*costMap3D)[0].size();
    int l = (*costMap3D)[0][0].size();

    std::vector<int> nodeTarget = goal;

    std::vector<std::vector<std::vector<double>>> *closedMap
        = new std::vector<std::vector<std::vector<double>>>;

    closedMap->resize(
        n, std::vector<std::vector<double>>(m, std::vector<double>(l)));

    TMap->resize(n,
                 std::vector<std::vector<double>>(m, std::vector<double>(l)));

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        {
            for (int k = 0; k < l; k++)
            {
                if (isinf((*costMap3D)[i][j][k]))
                {
                    (*closedMap)[i][j][k] = 1;
                }
                else
                {
                    (*closedMap)[i][j][k] = 0;
                }
                (*TMap)[i][j][k] = INFINITY;
            }
        }
    }

    (*closedMap)[nodeTarget[1]][nodeTarget[0]][nodeTarget[2]] = 1;

    (*TMap)[nodeTarget[1]][nodeTarget[0]][nodeTarget[2]] = 0;

    std::vector<double> *nbT = new std::vector<double>;
    std::vector<std::vector<int>> *nbNodes = new std::vector<std::vector<int>>;

    //std::cout << "Updating the Goal" << std::endl;
    updateNode(nodeTarget, costMap3D, TMap, nbT, nbNodes, closedMap);

    uint ui_iter = 1;
    while (nbT->size() > 0)
    {
        //std::cout << "\r 3D TMap computation - Iteration: " << ui_iter;
        ui_iter++;
	if (ui_iter > ui_max_iter)
	{
            std::cout << " \033[35m[----------]\033[0m [FastMarching3D::computeTMap()] ERROR: number of iterations has surpassed the limit ( " << ui_max_iter << " nodes )" << std::endl;
            std::cout << " \033[35m[----------]\033[0m [FastMarching3D::computeTMap()] Debug info:" << std::endl;
            std::cout << " \033[35m[----------]\033[0m [FastMarching3D::computeTMap()]  - Cost Value at start " << start[0] << ", " << start[1] << ", " << start[2] << ", " << " is " << (*costMap3D)[start[1]][start[0]][start[2]] << std::endl; 
            std::cout << " \033[35m[----------]\033[0m [FastMarching3D::computeTMap()]  - Cost Value at goal " << goal[0] << ", " << goal[1] << ", " << goal[2] << ", " << " is " << (*costMap3D)[goal[1]][goal[0]][goal[2]] << std::endl; 
            std::cout << " \033[35m[----------]\033[0m [FastMarching3D::computeTMap()]  - Cost Value of last node visited " << nodeTarget[0] << ", " << nodeTarget[1] << ", " << nodeTarget[2] << ", " << " is " << (*costMap3D)[nodeTarget[1]][nodeTarget[0]][nodeTarget[2]] << std::endl; 
	    nodeTarget = (*nbNodes)[0];
            std::cout << " \033[35m[----------]\033[0m [FastMarching3D::computeTMap()]  - Cost Value of next node to visit " << nodeTarget[0] << ", " << nodeTarget[1] << ", " << nodeTarget[2] << ", " << " is " << (*costMap3D)[nodeTarget[1]][nodeTarget[0]][nodeTarget[2]] << std::endl; 
            std::cout << " \033[35m[----------]\033[0m [FastMarching3D::computeTMap()]  - Narrow Band Size is: " << nbT->size() << " (nbT), " << nbNodes->size() << " (nbNodes)"  << std::endl;  
	    return false;
	}
	nodeTarget = (*nbNodes)[0];
        nbNodes->erase(nbNodes->begin());
        nbT->erase(nbT->begin());
        (*closedMap)[nodeTarget[1]][nodeTarget[0]][nodeTarget[2]] = 1;
        updateNode(nodeTarget, costMap3D, TMap, nbT, nbNodes, closedMap);

        if ((nodeTarget[0] == start[0]) && (nodeTarget[1] == start[1])
            && (nodeTarget[2] == start[2]))
        {
            //std::cout << "Done" << std::flush << std::endl;
            return true; //The propagation wave reaches the start
        }
    }
    //std::cout << "Done" << std::flush << std::endl;
    std::cout << " \033[35m[----------]\033[0m [FastMarching3D::computeTMap()] INFO The Total Cost Propagation has not reached the start node " << start[0] << ", " << start[1] << ", " << start[2] << " from goal " << goal[0] << ", " << goal[1] << ", " << goal[2] << std::endl;
    return false; //The propagation does not reach the start
}

void FastMarching3D::updateNode(
    std::vector<int> nodeTarget,
    const std::vector<std::vector<std::vector<double>>> *costMap3D,
    std::vector<std::vector<std::vector<double>>> *TMap,
    std::vector<double> *nbT,
    std::vector<std::vector<int>> *nbNodes,
    const std::vector<std::vector<std::vector<double>>> *closedMap)
{
    std::vector<int> nodeChild(3, 0);
    for (int i = 1; i < 6 + 1; i++)
    {
        switch (i)
        {
            case 1:
                nodeChild[0] = nodeTarget[0];
                nodeChild[1] = nodeTarget[1];
                nodeChild[2] = nodeTarget[2] - 1;
                break;
            case 2:
                nodeChild[0] = nodeTarget[0];
                nodeChild[1] = nodeTarget[1];
                nodeChild[2] = nodeTarget[2] + 1;
                break;
            case 3:
                nodeChild[0] = nodeTarget[0] - 1;
                nodeChild[1] = nodeTarget[1];
                nodeChild[2] = nodeTarget[2];
                break;
            case 4:
                nodeChild[0] = nodeTarget[0] + 1;
                nodeChild[1] = nodeTarget[1];
                nodeChild[2] = nodeTarget[2];
                break;
            case 5:
                nodeChild[0] = nodeTarget[0];
                nodeChild[1] = nodeTarget[1] - 1;
                nodeChild[2] = nodeTarget[2];
                break;
            case 6:
                nodeChild[0] = nodeTarget[0];
                nodeChild[1] = nodeTarget[1] + 1;
                nodeChild[2] = nodeTarget[2];
                break;
        }

        if ((*closedMap)[nodeChild[1]][nodeChild[0]][nodeChild[2]] == 0)
        {
            double Tx1 = (*TMap)[nodeChild[1]][nodeChild[0] + 1][nodeChild[2]];
            double Tx2 = (*TMap)[nodeChild[1]][nodeChild[0] - 1][nodeChild[2]];
            double Tx = std::min(Tx1, Tx2);

            double Ty1 = (*TMap)[nodeChild[1] + 1][nodeChild[0]][nodeChild[2]];
            double Ty2 = (*TMap)[nodeChild[1] - 1][nodeChild[0]][nodeChild[2]];
            double Ty = std::min(Ty1, Ty2);

            double Tz1 = (*TMap)[nodeChild[1]][nodeChild[0]][nodeChild[2] + 1];
            double Tz2 = (*TMap)[nodeChild[1]][nodeChild[0]][nodeChild[2] - 1];
            double Tz = std::min(Tz1, Tz2);

            double T = getEikonal(
                Tx,
                Ty,
                Tz,
                (*costMap3D)[nodeChild[1]][nodeChild[0]][nodeChild[2]]);

            if (isinf((*TMap)[nodeChild[1]][nodeChild[0]][nodeChild[2]]))
            {
                int index = getInsertIndex(nbT, T);
                std::vector<double>::iterator indexT = nbT->begin() + index;
                nbT->insert(indexT, T);
                std::vector<std::vector<int>>::iterator indexN
                    = nbNodes->begin() + index;
                nbNodes->insert(indexN, nodeChild);
                (*TMap)[nodeChild[1]][nodeChild[0]][nodeChild[2]] = T;
            }
            else if (T < (*TMap)[nodeChild[1]][nodeChild[0]][nodeChild[2]])
            {
                double tempT
                    = (*TMap)[nodeChild[1]][nodeChild[0]][nodeChild[2]];
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
                (*TMap)[nodeChild[1]][nodeChild[0]][nodeChild[2]] = T;
            }
        }
    }
}

double FastMarching3D::getEikonal(double Tx, double Ty, double Tz, double cost)
{
    if (isinf(Tx))
        if (isinf(Ty))
            if (isinf(Tz))
            {
                return INFINITY;
            }
            else
                return Tz + cost;
        else if (isinf(Tz))
            return Ty + cost;
        else if (cost < abs(Ty - Tz))
            return std::min(Ty, Tz) + cost;
        else
            return 0.5 * (Ty + Tz + sqrt(2 * pow(cost, 2) - pow(Ty - Tz, 2)));
    else if (isinf(Ty))
        if (isinf(Tz))
            return Tx + cost;
        else if (cost < abs(Tx - Tz))
            return std::min(Tx, Tz) + cost;
        else
            return 0.5 * (Tx + Tz + sqrt(2 * pow(cost, 2) - pow(Tx - Tz, 2)));
    else if (isinf(Tz))
        if (cost < abs(Tx - Ty))
            return std::min(Tx, Ty) + cost;
        else
            return 0.5 * (Tx + Ty + sqrt(2 * pow(cost, 2) - pow(Tx - Ty, 2)));
    else
    {
        double Tmax = std::max(Tx, std::max(Ty, Tz));
        if (pow(cost, 2)
            > (pow(Tmax - Tx, 2) + pow(Tmax - Ty, 2) + pow(Tmax - Tz, 2)))
        {
            return (1.0 / 3.0)
                   * (Tx + Ty + Tz
                      + sqrt(3 * pow(cost, 2) + pow(Tx + Ty + Tz, 2)
                             - 3 * (pow(Tx, 2) + pow(Ty, 2) + pow(Tz, 2))));
        }
        else
        {
            double Tmin1, Tmin2;
            Tmin1 = std::min(Tx, std::min(Ty, Tz));
            Tmin2 = std::max(std::min(Tx, Ty),
                             std::max(std::min(Ty, Tz), std::min(Tx, Tz)));
            if (cost < abs(Tmin1 - Tmin2))
                return Tmin1 + cost;
            else
                return 0.5
                       * (Tmin1 + Tmin2
                          + sqrt(2 * pow(cost, 2) - pow(Tmin1 - Tmin2, 2)));
        }
    }
}

int FastMarching3D::getInsertIndex(std::vector<double> *nbT, double T)
{
    int i;
    for (i = 0; i < nbT->size(); i++)
        if (T <= nbT->at(i)) break;
    return i;
}

bool FastMarching3D::computePathGDM(
    const std::vector<std::vector<std::vector<double>>> *TMap,
    std::vector<int> initNode,
    std::vector<int> endNode,
    double tau,
    std::vector<std::vector<double>> *path,
    unsigned int ui_max_iter)
{
    std::vector<double> auxVector;
    auxVector.push_back((double)initNode[0]);
    auxVector.push_back((double)initNode[1]);
    auxVector.push_back((double)initNode[2]);

    path->push_back(auxVector);

    std::vector<int> nearNode = {0, 0, 0};

    std::vector<std::vector<std::vector<double>>> *G1
        = new std::vector<std::vector<std::vector<double>>>;
    std::vector<std::vector<std::vector<double>>> *G2
        = new std::vector<std::vector<std::vector<double>>>;
    std::vector<std::vector<std::vector<double>>> *G3
        = new std::vector<std::vector<std::vector<double>>>;

    G1->resize(
        TMap->size(),
        std::vector<std::vector<double>>(
            (*TMap)[0].size(), std::vector<double>((*TMap)[0][0].size())));
    G2->resize(
        TMap->size(),
        std::vector<std::vector<double>>(
            (*TMap)[0].size(), std::vector<double>((*TMap)[0][0].size())));
    G3->resize(
        TMap->size(),
        std::vector<std::vector<double>>(
            (*TMap)[0].size(), std::vector<double>((*TMap)[0][0].size())));

    //std::cout << "Computing the 3D path" << std::endl;
    for (int k = 0; k < ui_max_iter; k++)
    {
        //std::cout << "\rk = " << k;
        computeGradient(TMap, path->at(path->size() - 1), G1, G2, G3);
        double dx = getInterpolatedPoint(path->at(path->size() - 1), G1);
        double dy = getInterpolatedPoint(path->at(path->size() - 1), G2);
        double dz = getInterpolatedPoint(path->at(path->size() - 1), G3);

        if (isnan(dx) || isnan(dy) || isnan(dz))
        {
            try
            {
                nearNode[0] = (int)(*path)[path->size() - 1][0] + 0.5;
                nearNode[1] = (int)(*path)[path->size() - 1][1] + 0.5;
                nearNode[2] = (int)(*path)[path->size() - 1][2] + 0.5;
                while (isinf((*TMap)[nearNode[1]][nearNode[0]][nearNode[2]]))
                {
                    path->erase(path->end());
                    nearNode[0] = (int)(*path)[path->size() - 1][0] + 0.5;
                    nearNode[1] = (int)(*path)[path->size() - 1][1] + 0.5;
                    nearNode[2] = (int)(*path)[path->size() - 1][2] + 0.5;
                }

                if (path->size() > 0)
                {
                    std::vector<double> distVector;
                    distVector.push_back((*path)[path->size() - 1][0]
                                         - (double)nearNode[0]);
                    distVector.push_back((*path)[path->size() - 1][1]
                                         - (double)nearNode[1]);
                    distVector.push_back((*path)[path->size() - 1][2]
                                         - (double)nearNode[2]);
                    while (sqrt(pow(distVector[0], 2) + pow(distVector[1], 2)
                                + pow(distVector[2], 2))
                           < 1)
                    {
                        path->erase(path->end());
                        if (path->size() == 0) break;
                        distVector[0] = (*path)[path->size() - 1][0]
                                        - (double)nearNode[0];
                        distVector[1] = (*path)[path->size() - 1][1]
                                        - (double)nearNode[1];
                        distVector[2] = (*path)[path->size() - 1][2]
                                        - (double)nearNode[2];
                    }
                }

                auxVector[0] = (double)nearNode[0];
                auxVector[1] = (double)nearNode[1];
                auxVector[2] = (double)nearNode[2];
                path->push_back(auxVector);

                double currentT
                    = (*TMap)[nearNode[1]][nearNode[0]][nearNode[2]];
                for (int i = 1; i < 6 + 1; i++)
                {
                    std::vector<int> nodeChild(3, 0);
                    if (i == 1)
                    {
                        nodeChild[0] = nearNode[0];
                        nodeChild[1] = nearNode[1] - 1;
                        nodeChild[2] = nearNode[2];
                    }
                    else if (i == 2)
                    {
                        nodeChild[0] = nearNode[0];
                        nodeChild[1] = nearNode[1] + 1;
                        nodeChild[2] = nearNode[2];
                    }
                    else if (i == 3)
                    {
                        nodeChild[0] = nearNode[0] - 1;
                        nodeChild[1] = nearNode[1];
                        nodeChild[2] = nearNode[2];
                    }
                    else if (i == 4)
                    {
                        nodeChild[0] = nearNode[0] + 1;
                        nodeChild[1] = nearNode[1];
                        nodeChild[2] = nearNode[2];
                    }
                    else if (i == 5)
                    {
                        nodeChild[0] = nearNode[0];
                        nodeChild[1] = nearNode[1];
                        nodeChild[2] = nearNode[2] - 1;
                    }
                    else if (i == 6)
                    {
                        nodeChild[0] = nearNode[0];
                        nodeChild[1] = nearNode[1];
                        nodeChild[2] = nearNode[2] + 1;
                    }

                    if ((*TMap)[nodeChild[1]][nodeChild[0]][nodeChild[2]]
                        < currentT)
                    {
                        currentT
                            = (*TMap)[nodeChild[1]][nodeChild[0]][nodeChild[2]];
                        dx = (nearNode[0] - nodeChild[0]) / tau;
                        dy = (nearNode[1] - nodeChild[1]) / tau;
                        dz = (nearNode[2] - nodeChild[2]) / tau;
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

        double d = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
        if (d < 0.01)
        {
            double dnx = dx / d;
            double dny = dy / d;
            double dnz = dz / d;

            auxVector[0] = (*path)[path->size() - 1][0] - tau * dnx;
            auxVector[1] = (*path)[path->size() - 1][1] - tau * dny;
            auxVector[2] = (*path)[path->size() - 1][2] - tau * dnz;
            path->push_back(auxVector);
        }
        else
        {
            auxVector[0] = (*path)[path->size() - 1][0] - tau * dx;
            auxVector[1] = (*path)[path->size() - 1][1] - tau * dy;
            auxVector[2] = (*path)[path->size() - 1][2] - tau * dz;
            path->push_back(auxVector);
        }

        auxVector[0] = (*path)[path->size() - 1][0] - (double)endNode[0];
        auxVector[1] = (*path)[path->size() - 1][1] - (double)endNode[1];
        auxVector[2] = (*path)[path->size() - 1][2] - (double)endNode[2];
        if (sqrt(pow(auxVector[0], 2) + pow(auxVector[1], 2)
                 + pow(auxVector[2], 2))
            < 1.5)
            break;
    }

    //std::cout << "...done!" << std::flush << std::endl;
    if (path->size() >= ui_max_iter)
    {
        return false;
    }
    auxVector[0] = (double)endNode[0];
    auxVector[1] = (double)endNode[1];
    auxVector[2] = (double)endNode[2];
    path->push_back(auxVector);
    return true;
}

void FastMarching3D::computeGradient(
    const std::vector<std::vector<std::vector<double>>> *TMap,
    std::vector<double> point,
    std::vector<std::vector<std::vector<double>>> *Gnx,
    std::vector<std::vector<std::vector<double>>> *Gny,
    std::vector<std::vector<std::vector<double>>> *Gnz)
{
    int n = TMap->size();
    int m = (*TMap)[0].size();
    int l = (*TMap)[0][0].size();

    int jmax, imax, kmax, jmin, imin, kmin;
    if (point.size() == 0)
    {
        jmax = n;
        imax = m;
        kmax = l;

        jmin = 0;
        imin = 0;
        kmin = 0;
    }
    else
    {
        jmax = std::min(n, (int)(point[1] + 3.5));
        imax = std::min(m, (int)(point[0] + 3.5));
        kmax = std::min(l, (int)(point[2] + 3.5));

        jmin = std::max(0, (int)(point[1] - 2.5));
        imin = std::max(0, (int)(point[0] - 2.5));
        kmin = std::max(0, (int)(point[2] - 2.5));
    }

    std::vector<std::vector<std::vector<double>>> Gx, Gy, Gz;

    Gx.resize(n, std::vector<std::vector<double>>(m, std::vector<double>(l)));
    Gy.resize(n, std::vector<std::vector<double>>(m, std::vector<double>(l)));
    Gz.resize(n, std::vector<std::vector<double>>(m, std::vector<double>(l)));

    for (int i = imin; i < imax; i++)
        for (int j = jmin; j < jmax; j++)
            for (int k = kmin; k < kmax; k++)
            {
                if (j == 0)
                    Gy[0][i][k] = (*TMap)[1][i][k] - (*TMap)[0][i][k];
                else
                {
                    if (j == n - 1)
                        Gy[j][i][k] = (*TMap)[j][i][k] - (*TMap)[j - 1][i][k];
                    else
                    {
                        if (isinf((*TMap)[j + 1][i][k]))
                        {
                            if (isinf((*TMap)[j - 1][i][k]))
                                Gy[j][i][k] = 0;
                            else
                                Gy[j][i][k]
                                    = (*TMap)[j][i][k] - (*TMap)[j - 1][i][k];
                        }
                        else
                        {
                            if (isinf((*TMap)[j - 1][i][k]))
                                Gy[j][i][k]
                                    = (*TMap)[j + 1][i][k] - (*TMap)[j][i][k];
                            else
                                Gy[j][i][k] = ((*TMap)[j + 1][i][k]
                                               - (*TMap)[j - 1][i][k])
                                              / 2;
                        }
                    }
                }

                if (i == 0)
                    Gx[j][0][k] = (*TMap)[j][1][k] - (*TMap)[j][0][k];
                else
                {
                    if (i == m - 1)
                        Gx[j][i][k] = (*TMap)[j][i][k] - (*TMap)[j][i - 1][k];
                    else
                    {
                        if (isinf((*TMap)[j][i + 1][k]))
                        {
                            if (isinf((*TMap)[j][i - 1][k]))
                                Gx[j][i][k] = 0;
                            else
                                Gx[j][i][k]
                                    = (*TMap)[j][i][k] - (*TMap)[j][i - 1][k];
                        }
                        else
                        {
                            if (isinf((*TMap)[j][i - 1][k]))
                                Gx[j][i][k]
                                    = (*TMap)[j][i + 1][k] - (*TMap)[j][i][k];
                            else
                                Gx[j][i][k] = ((*TMap)[j][i + 1][k]
                                               - (*TMap)[j][i - 1][k])
                                              / 2;
                        }
                    }
                }

                if (k == 0)
                    Gz[j][i][0] = (*TMap)[j][i][1] - (*TMap)[j][i][0];
                else
                {
                    if (k == l - 1)
                        Gz[j][i][k] = (*TMap)[j][i][k] - (*TMap)[j][i][k - 1];
                    else
                    {
                        if (isinf((*TMap)[j][i][k + 1]))
                        {
                            if (isinf((*TMap)[j][i][k - 1]))
                                Gz[j][i][k] = 0;
                            else
                                Gz[j][i][k]
                                    = (*TMap)[j][i][k] - (*TMap)[j][i][k - 1];
                        }
                        else
                        {
                            if (isinf((*TMap)[j][i][k - 1]))
                                Gz[j][i][k]
                                    = (*TMap)[j][i][k + 1] - (*TMap)[j][i][k];
                            else
                                Gz[j][i][k] = ((*TMap)[j][i][k + 1]
                                               - (*TMap)[j][i][k - 1])
                                              / 2;
                        }
                    }
                }

                (*Gnx)[j][i][k]
                    = Gx[j][i][k]
                      / sqrt(pow(Gx[j][i][k], 2) + pow(Gy[j][i][k], 2)
                             + pow(Gz[j][i][k], 2));
                (*Gny)[j][i][k]
                    = Gy[j][i][k]
                      / sqrt(pow(Gx[j][i][k], 2) + pow(Gy[j][i][k], 2)
                             + pow(Gz[j][i][k], 2));
                (*Gnz)[j][i][k]
                    = Gz[j][i][k]
                      / sqrt(pow(Gx[j][i][k], 2) + pow(Gy[j][i][k], 2)
                             + pow(Gz[j][i][k], 2));
            }
}

double FastMarching3D::getInterpolatedPoint(
    std::vector<double> point,
    const std::vector<std::vector<std::vector<double>>> *mapI)
{
    int i = (int)point[0];
    int j = (int)point[1];
    int k = (int)point[2];
    double a = point[0] - double(i);
    double b = point[1] - double(j);
    double c = point[2] - double(k);

    int n = mapI->size();
    int m = (*mapI)[0].size();
    int l = (*mapI)[0][0].size();

    double I;
    if (i == m - 1)
        if (j == n - 1)
            if (k == l - 1)
                I = (*mapI)[j][i][k];
            else
                I = c * (*mapI)[j][i][k + 1] + (1 - c) * (*mapI)[j][i][k];
        else if (k == l - 1)
            I = b * (*mapI)[j + 1][i][k] + (1 - b) * (*mapI)[j][i][k];
        else
        {
            double a0 = (*mapI)[j][i][k];
            double a2 = (*mapI)[j + 1][i][k] - (*mapI)[j][i][k];
            double a3 = (*mapI)[j][i][k + 1] - (*mapI)[j][i][k];
            double a6 = (*mapI)[j + 1][i][k + 1] + (*mapI)[j][i][k]
                        - (*mapI)[j + 1][i][k] - (*mapI)[j][i][k + 1];
            if (b == 0)
                if (c == 0)
                    I = a0;
                else
                    I = a0 + a3 * c;
            else if (c == 0)
                I = a0 + a2 * b;
            else
                I = a0 + a2 * b + a3 * c + a6 * b * c;
        }
    else if (j == n - 1)
        if (k == l - 1)
            I = a * (*mapI)[j][i + 1][k] + (1 - a) * (*mapI)[j][i][k];
        else
        {
            double a0 = (*mapI)[j][i][k];
            double a1 = (*mapI)[j][i + 1][k] - (*mapI)[j][i][k];
            double a3 = (*mapI)[j][i][k + 1] - (*mapI)[j][i][k];
            double a5 = (*mapI)[j][i + 1][k + 1] + (*mapI)[j][i][k]
                        - (*mapI)[j][i + 1][k] - (*mapI)[j][i][k + 1];

            if (a == 0)
                if (c == 0)
                    I = a0;
                else
                    I = a0 + a3 * c;
            else if (c == 0)
                I = a0 + a1 * a;
            else
                I = a0 + a1 * a + a3 * c + a5 * a * c;
        }
    else
    {
        if (k == l - 1)
        {
            double a0 = (*mapI)[j][i][k];
            double a1 = (*mapI)[j][i + 1][k] - (*mapI)[j][i][k];
            double a2 = (*mapI)[j + 1][i][k] - (*mapI)[j][i][k];
            double a4 = (*mapI)[j + 1][i + 1][k] + (*mapI)[j][i][k]
                        - (*mapI)[j][i + 1][k] - (*mapI)[j + 1][i][k];

            if (a == 0)
                if (b == 0)
                    I = a0;
                else
                    I = a0 + a2 * b;
            else if (b == 0)
                I = a0 + a1 * a;
            else
                I = a0 + a1 * a + a2 * b + a4 * a * b;
        }
        else
        {
            double a0 = (*mapI)[j][i][k];
            double a1 = (*mapI)[j][i + 1][k] - (*mapI)[j][i][k];
            double a2 = (*mapI)[j + 1][i][k] - (*mapI)[j][i][k];
            double a3 = (*mapI)[j][i][k + 1] - (*mapI)[j][i][k];
            double a4 = (*mapI)[j + 1][i + 1][k] + (*mapI)[j][i][k]
                        - (*mapI)[j][i + 1][k] - (*mapI)[j + 1][i][k];
            double a5 = (*mapI)[j][i + 1][k + 1] + (*mapI)[j][i][k]
                        - (*mapI)[j][i + 1][k] - (*mapI)[j][i][k + 1];
            double a6 = (*mapI)[j + 1][i][k + 1] + (*mapI)[j][i][k]
                        - (*mapI)[j + 1][i][k] - (*mapI)[j][i][k + 1];
            double a7 = (*mapI)[j + 1][i + 1][k + 1] + (*mapI)[j][i][k]
                        - (*mapI)[j + 1][i][k] - (*mapI)[j][i + 1][k]
                        - (*mapI)[j][i][k + 1];

            if (a == 0)
                if (b == 0)
                    if (c == 0)
                        I = a0;
                    else
                        I = a0 + a3 * c;
                else if (c == 0)
                    I = a0 + a2 * b;
                else
                    I = a0 + a2 * b + a3 * c + a6 * b * c;
            else if (b == 0)
                if (c == 0)
                    I = a0 + a1 * a;
                else
                    I = a0 + a1 * a + a3 * c + a5 * a * c;
            else if (c == 0)
                I = a0 + a1 * a + a2 * b + a4 * a * b;
            else
            {
                I = a0 + a1 * a + a2 * b + a3 * c + a4 * a * b + a5 * a * c
                    + a6 * b * c + a7 * a * b * c;
            }
        }
    }

    return I;
}
