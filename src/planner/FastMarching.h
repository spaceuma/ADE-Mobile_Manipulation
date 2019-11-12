#include <vector>
#include "Waypoint.hpp"


namespace FastMarching_lib 
{
class BiFastMarching
{
	private:

	public:
		// -- PARAMETERS --

		// -- FUNCTIONS --
		void planRoverPath(const std::vector<std::vector<double>> * currentCostMap,
				   double mapResolution,
 				   base::Waypoint roverPos,
 				   base::Waypoint samplePos,
 				   std::vector<base::Waypoint> * roverPath);

		void computeTMap(const std::vector<std::vector<double>> * currentCostMap,
				 std::vector<int> goal,
				 std::vector<int> start,
				 std::vector<std::vector<double>> * TMapGoal,
				 std::vector<std::vector<double>> * TMapStart,
				 std::vector<int> * nodeJoin);

		void updateNode(std::vector<int> nodeTarget,
				const std::vector<std::vector<double>> * currentCostMap,
    				std::vector<std::vector<double>> * TMap,
    				std::vector<double> * nbT,
    				std::vector<std::vector<int>> * nbNodes,
    				const std::vector<std::vector<double>> * closedMap);

		double getEikonal(double THor, double TVer, double cost);

		int getInsertIndex(std::vector<double> * nbT, double T);

		void planPathGDM(const std::vector<std::vector<double>> * TMap,
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

class BiFastMarching3D
{
	private:

	public:
		// -- PARAMETERS --

		// -- FUNCTIONS --
		void planEndEffectorPath(const std::vector<std::vector<std::vector<double>>> * tunnelCostMap,
				   double mapResolution,
				   double zResolution,
 				   base::Waypoint roverPos,
 				   base::Waypoint samplePos,
 				   std::vector<base::Waypoint> * endEffectorPath);

		void computeTMap(const std::vector<std::vector<std::vector<double>>> * currentCostMap,
				 std::vector<int> goal,
				 std::vector<int> start,
				 std::vector<std::vector<std::vector<double>>> * TMapGoal,
				 std::vector<std::vector<std::vector<double>>> * TMapStart,
				 std::vector<int> * nodeJoin);

		void updateNode(std::vector<int> nodeTarget,
				const std::vector<std::vector<std::vector<double>>> * currentCostMap,
    				std::vector<std::vector<std::vector<double>>> * TMap,
    				std::vector<double> * nbT,
    				std::vector<std::vector<int>> * nbNodes,
    				const std::vector<std::vector<std::vector<double>>> * closedMap);

		double getEikonal(double Tx, double Ty, double Tz, double cost);

		int getInsertIndex(std::vector<double> * nbT, double T);

		void planPathGDM(const std::vector<std::vector<std::vector<double>>> * TMap,
				 std::vector<int> initNode,
				 std::vector<int> endNode,
				 double tau,
				 std::vector<std::vector<double>> * path);

		void computeGradient(const std::vector<std::vector<std::vector<double>>> * TMap,
				     std::vector<double> point,
				     std::vector<std::vector<std::vector<double>>> * Gnx,
				     std::vector<std::vector<std::vector<double>>> * Gny,
				     std::vector<std::vector<std::vector<double>>> * Gnz);

		double getInterpolatedPoint(std::vector<double> point,
					    const std::vector<std::vector<std::vector<double>>> * mapI);

};

}
