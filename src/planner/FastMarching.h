#include "Waypoint.hpp"
#include <vector>

namespace FastMarching_lib
{
class BiFastMarching
{
private:
public:
    // -- PARAMETERS --

    // -- FUNCTIONS --
    void planPath(const std::vector<std::vector<double>> *costMap,
                  double mapResolution,
                  base::Waypoint iniPos,
                  base::Waypoint finalPos,
                  std::vector<base::Waypoint> *path);

    void computeTMap(const std::vector<std::vector<double>> *costMap,
                     std::vector<int> goal,
                     std::vector<int> start,
                     std::vector<std::vector<double>> *TMapGoal,
                     std::vector<std::vector<double>> *TMapStart,
                     std::vector<int> *nodeJoin);

    void updateNode(std::vector<int> nodeTarget,
                    const std::vector<std::vector<double>> *costMap,
                    std::vector<std::vector<double>> *TMap,
                    std::vector<double> *nbT,
                    std::vector<std::vector<int>> *nbNodes,
                    const std::vector<std::vector<double>> *closedMap);

    double getEikonal(double THor, double TVer, double cost);

    int getInsertIndex(std::vector<double> *nbT, double T);

    void computePathGDM(const std::vector<std::vector<double>> *TMap,
                        std::vector<int> initNode,
                        std::vector<int> endNode,
                        double tau,
                        std::vector<std::vector<double>> *path);

    void computeGradient(const std::vector<std::vector<double>> *TMap,
                         std::vector<double> point,
                         std::vector<std::vector<double>> *Gnx,
                         std::vector<std::vector<double>> *Gny);

    double getInterpolatedPoint(std::vector<double> point, const std::vector<std::vector<double>> *mapI);
};

class BiFastMarching3D
{
private:
public:
    // -- PARAMETERS --

    // -- FUNCTIONS --
    void planPath(const std::vector<std::vector<std::vector<double>>> *costMap3D,
                  double mapResolution,
                  double zResolution,
                  base::Waypoint iniPos,
                  base::Waypoint endPos,
                  std::vector<base::Waypoint> *path3D);

    void computeTMap(const std::vector<std::vector<std::vector<double>>> *costMap3D,
                     std::vector<int> goal,
                     std::vector<int> start,
                     std::vector<std::vector<std::vector<double>>> *TMapGoal,
                     std::vector<std::vector<std::vector<double>>> *TMapStart,
                     std::vector<int> *nodeJoin);

    void updateNode(std::vector<int> nodeTarget,
                    const std::vector<std::vector<std::vector<double>>> *costMap3D,
                    std::vector<std::vector<std::vector<double>>> *TMap,
                    std::vector<double> *nbT,
                    std::vector<std::vector<int>> *nbNodes,
                    const std::vector<std::vector<std::vector<double>>> *closedMap);

    double getEikonal(double Tx, double Ty, double Tz, double cost);

    int getInsertIndex(std::vector<double> *nbT, double T);

    void computePathGDM(const std::vector<std::vector<std::vector<double>>> *TMap,
                        std::vector<int> initNode,
                        std::vector<int> endNode,
                        double tau,
                        std::vector<std::vector<double>> *path);

    void computeGradient(const std::vector<std::vector<std::vector<double>>> *TMap,
                         std::vector<double> point,
                         std::vector<std::vector<std::vector<double>>> *Gnx,
                         std::vector<std::vector<std::vector<double>>> *Gny,
                         std::vector<std::vector<std::vector<double>>> *Gnz);

    double getInterpolatedPoint(std::vector<double> point, const std::vector<std::vector<std::vector<double>>> *mapI);
};
}
