#include <stdint.h>

/*!
* @struct DEM
*
* @brief Structure designed to hold a digital elevation map which is provided as input to the rover guidance.
*/
typedef struct RoverGuidance_DemStruct
{
/*! @name cols
* @details Number of columns in the map, where columns is given by the x direction and an increase in the
* column index corresponds to an increase in x in the MLG frame.
*/
int cols;
/*!
* @name rows
* @details Number of rows in the map, where rows is given by the y direction and increasing row index
* corresponds to increasing y in the MLG frame.
*/
int rows;
/*!
* @name nodeSize
* @details This distance in x and y between the nodes in the map.
* @units metres
*/
double nodeSize_m;
/*!
* @name mapOrigin
* @details The origin of the map in the Global frame. The origin is defined as the origin of the first element
* in the array, with column = 0 (minimum x) and row = 0 (minimum y).
*
* @unit metres
* @frame MLG
*/
double mapOrigin_m_Mlg[3];

/*!
* @name heightData
* @details This is the height of each node, which is the distance of a node to the map frame along the z_DS
* axis. It is indexed by cols and then by rows which corresponds to x and y respectively.
*
* The mapping of the data (row/column) onto the 1D data arrays (single index) is defined as follows:
* index = row * cols + col
*
* @unit metres
*/
double *p_heightData_m;
/*!
* @name pointValidityFlag
* @details This flag indicates the following for each node, saved in a 1D array:
* 1 - if the node contains valid height data
* 0 - if the node does not contain any valid height data.
* @unit N/A
*/
int8_t *p_pointValidityFlag;
}RoverGuidance_Dem;
