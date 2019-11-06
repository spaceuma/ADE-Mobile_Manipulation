#include <vector>
#include "../types/Pose.hpp"


namespace FastMarching_lib 
{
class BiFastMarching
{
	private:

	public:
		// -- PARAMETERS --

		// -- FUNCTIONS --
		void computeTMap(const std::vector<std::vector<double>> * currentCostMap,
				    std::vector<int> goal,
				    std::vector<int> start,
				    std::vector<std::vector<double>> * TMapGoal,
				    std::vector<std::vector<double>> * TMapStart);

		void updateNode(std::vector<int> nodeTarget,
				const std::vector<std::vector<double>> * currentCostMap,
    				std::vector<std::vector<double>> * TMap,
    				std::vector<double> * nbT,
    				std::vector<std::vector<int>> * nbNodes,
    				const std::vector<std::vector<double>> * closedMap);

		double getEikonal(double THor, double TVer, double cost);

		int bisect(std::vector<double> * nbT, double T);
};
}
