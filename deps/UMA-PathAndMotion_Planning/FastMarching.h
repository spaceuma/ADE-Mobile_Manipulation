// MIT License
// -----------
//
// Copyright (c) 2021 University of Malaga
// Permission is hereby granted, free of charge, to any person
// obtaining a copy of this software and associated documentation
// files (the "Software"), to deal in the Software without
// restriction, including without limitation the rights to use,
// copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following
// conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
// HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// Authors: J. Ricardo Sánchez Ibáñez, Carlos J. Pérez del Pulgar
// Affiliation: Department of Systems Engineering and Automation
// Space Robotics Lab (www.uma.es/space-robotics)

#ifndef __FAST_MARCHING__
#define __FAST_MARCHING__

#include "Waypoint.hpp"
#include <vector>

namespace FastMarching_lib
{
class FastMarching
{
private:
public:
    // -- PARAMETERS --
    double waypointDistance;

    // -- FUNCTIONS --
    FastMarching(double _waypointDistance = 0.5);
    ~FastMarching();

    bool planPath(const std::vector<std::vector<double>> * costMap,
                  double mapResolution,
                  base::Waypoint iniPos,
                  base::Waypoint finalPos,
                  std::vector<base::Waypoint> * path);

    void getShadowedCostMap(std::vector<std::vector<int>> & vvi_obstacle_map,
                            double mapResolution,
                            double d_min_distance,
                            double d_max_distance,
                            base::Waypoint finalPos);

    void computeEntireTMap(const std::vector<std::vector<double>> * costMap,
                           std::vector<int> goal,
                           std::vector<std::vector<double>> * closedMap,
                           std::vector<std::vector<double>> * TMap);

    bool computeTMap(const std::vector<std::vector<double>> * costMap,
                     std::vector<int> goal,
                     std::vector<int> start,
                     std::vector<std::vector<double>> * TMap);

    void updateNode(std::vector<int> nodeTarget,
                    const std::vector<std::vector<double>> * costMap,
                    std::vector<std::vector<double>> * TMap,
                    std::vector<double> * nbT,
                    std::vector<std::vector<int>> * nbNodes,
                    const std::vector<std::vector<double>> * closedMap);

    double getEikonal(double THor, double TVer, double cost);

    int getInsertIndex(std::vector<double> * nbT, double T);

    void computePathGDM(const std::vector<std::vector<double>> * TMap,
                        std::vector<int> initNode,
                        std::vector<int> endNode,
                        double tau,
                        std::vector<std::vector<double>> * path);

    void computeGradient(const std::vector<std::vector<double>> * TMap,
                         std::vector<double> point,
                         std::vector<std::vector<double>> * Gnx,
                         std::vector<std::vector<double>> * Gny);

    double getInterpolatedPoint(std::vector<double> point,
                                const std::vector<std::vector<double>> * mapI);
};

class BiFastMarching
{
private:
public:
    // -- PARAMETERS --
    double waypointDistance;

    // -- FUNCTIONS --
    BiFastMarching(double _waypointDistance = 0.5);
    ~BiFastMarching();

    bool planPath(const std::vector<std::vector<double>> * costMap,
                  double mapResolution,
                  base::Waypoint iniPos,
                  base::Waypoint finalPos,
                  std::vector<base::Waypoint> * path);

    bool computeTMap(const std::vector<std::vector<double>> * costMap,
                     std::vector<int> goal,
                     std::vector<int> start,
                     std::vector<std::vector<double>> * TMapGoal,
                     std::vector<std::vector<double>> * TMapStart,
                     std::vector<int> * nodeJoin);

    void updateNode(std::vector<int> nodeTarget,
                    const std::vector<std::vector<double>> * costMap,
                    std::vector<std::vector<double>> * TMap,
                    std::vector<double> * nbT,
                    std::vector<std::vector<int>> * nbNodes,
                    const std::vector<std::vector<double>> * closedMap);

    double getEikonal(double THor, double TVer, double cost);

    int getInsertIndex(std::vector<double> * nbT, double T);

    void computePathGDM(const std::vector<std::vector<double>> * TMap,
                        std::vector<int> initNode,
                        std::vector<int> endNode,
                        double tau,
                        std::vector<std::vector<double>> * path);

    void computeGradient(const std::vector<std::vector<double>> * TMap,
                         std::vector<double> point,
                         std::vector<std::vector<double>> * Gnx,
                         std::vector<std::vector<double>> * Gny);

    void computeGradient(const std::vector<std::vector<double>> * TMap,
                         const std::vector<int> point,
                         double * Gnx,
                         double * Gny);

    double getInterpolatedPoint(std::vector<double> point,
                                const std::vector<std::vector<double>> * mapI);
};

class BiFastMarching3D
{
private:
public:
    // -- PARAMETERS --
    double waypointDistance;

    // -- FUNCTIONS --
    BiFastMarching3D(double _waypointDistance = 0.5);
    ~BiFastMarching3D();

    void planPath(const std::vector<std::vector<std::vector<double>>> * costMap3D,
                  double mapResolution,
                  double zResolution,
                  base::Waypoint iniPos,
                  base::Waypoint endPos,
                  std::vector<base::Waypoint> * path3D);

    void computeTMap(const std::vector<std::vector<std::vector<double>>> * costMap3D,
                     std::vector<int> goal,
                     std::vector<int> start,
                     std::vector<std::vector<std::vector<double>>> * TMapGoal,
                     std::vector<std::vector<std::vector<double>>> * TMapStart,
                     std::vector<int> * nodeJoin);

    void updateNode(std::vector<int> nodeTarget,
                    const std::vector<std::vector<std::vector<double>>> * costMap3D,
                    std::vector<std::vector<std::vector<double>>> * TMap,
                    std::vector<double> * nbT,
                    std::vector<std::vector<int>> * nbNodes,
                    const std::vector<std::vector<std::vector<double>>> * closedMap);

    double getEikonal(double Tx, double Ty, double Tz, double cost);

    int getInsertIndex(std::vector<double> * nbT, double T);

    void computePathGDM(const std::vector<std::vector<std::vector<double>>> * TMap,
                        std::vector<int> initNode,
                        std::vector<int> endNode,
                        double tau,
                        std::vector<std::vector<double>> * path);

    void computeGradient(const std::vector<std::vector<std::vector<double>>> * TMap,
                         std::vector<double> point,
                         std::vector<std::vector<std::vector<double>>> * Gnx,
                         std::vector<std::vector<std::vector<double>>> * Gny,
                         std::vector<std::vector<std::vector<double>>> * Gnz);

    void computeGradient(const std::vector<std::vector<std::vector<double>>> * TMap,
                         std::vector<int> point,
                         double * Gnx,
                         double * Gny,
                         double * Gnz);

    double getInterpolatedPoint(std::vector<double> point,
                                const std::vector<std::vector<std::vector<double>>> * mapI);
};

class FastMarching3D
{
private:
public:
    // -- PARAMETERS --
    double waypointDistance;

    // -- FUNCTIONS --
    FastMarching3D(double _waypointDistance = 0.5);
    ~FastMarching3D();

    bool planPath(const std::vector<std::vector<std::vector<double>>> * costMap3D,
                  double mapResolution,
                  double zResolution,
                  base::Waypoint iniPos,
                  base::Waypoint endPos,
                  std::vector<base::Waypoint> * path3D,
                  unsigned int ui_max_iter);

    bool computeTMap(const std::vector<std::vector<std::vector<double>>> * costMap3D,
                     std::vector<int> goal,
                     std::vector<int> start,
                     std::vector<std::vector<std::vector<double>>> * TMap,
                     unsigned int ui_max_iter);

    void updateNode(std::vector<int> nodeTarget,
                    const std::vector<std::vector<std::vector<double>>> * costMap3D,
                    std::vector<std::vector<std::vector<double>>> * TMap,
                    std::vector<double> * nbT,
                    std::vector<std::vector<int>> * nbNodes,
                    const std::vector<std::vector<std::vector<double>>> * closedMap);

    double getEikonal(double Tx, double Ty, double Tz, double cost);

    int getInsertIndex(std::vector<double> * nbT, double T);

    bool computePathGDM(const std::vector<std::vector<std::vector<double>>> * TMap,
                        std::vector<int> initNode,
                        std::vector<int> endNode,
                        double tau,
                        std::vector<std::vector<double>> * path,
                        unsigned int ui_max_iter);

    void computeGradient(const std::vector<std::vector<std::vector<double>>> * TMap,
                         std::vector<double> point,
                         std::vector<std::vector<std::vector<double>>> * Gnx,
                         std::vector<std::vector<std::vector<double>>> * Gny,
                         std::vector<std::vector<std::vector<double>>> * Gnz);

    double getInterpolatedPoint(std::vector<double> point,
                                const std::vector<std::vector<std::vector<double>>> * mapI);
};
}    // namespace FastMarching_lib
#endif
