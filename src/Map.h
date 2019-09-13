/**
 * This class includes the provide DEM, the cost map and the obstacles map.
 */
class Map {

private:
	/**
	 * The mapping of the data (row/column) onto the 1D data arrays (single index) is defined as follows:
	 * * index = row * cols + col
	 */
	RoverGuidance_Dem currentDem;
	/**
	 * The mapping of the data (row/column) onto the 1D data arrays (single index) is defined as follows:
	 * * index = row * cols + col
	 */
	boolean currentObstaclesMap[];

public:
	/**
	 * Constructor that receives the map, process it and generates the cost and obstacles maps.
	 */
	Map(RoverGuidance_Dem dem);

	/**
	 * It updates currentDem and recalculates the cost and obstacles maps.
	 */
	void updateDem(RoverGuidance_Dem dem);

private:
	/**
	 * Based on currentDEM, it calculates or recalculates the currentCostMap.
	 */
	void calculateCostMap();

	/**
	 * It calculates o recalculates the obstacles map based on currentDEM
	 */
	void calculateObstaclesMap();
};
