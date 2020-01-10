#include "readMatrixFile.h"

void readMatrixFile(std::string map_file,
                    std::vector<std::vector<double>> &vector_elevationData)
{
    std::string line;
    std::ifstream e_file(map_file.c_str(), std::ios::in);
    double n_row = 0;
    double n_col = 0;
    vector_elevationData.clear();
    std::vector <double> row;
    if (e_file.is_open())
    {
        while (std::getline(e_file, line))
        {
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ' '))
            {
                double val;
                std::stringstream numeric_value(cell);
                numeric_value >> val;
                row.push_back(val);
                n_col++;
            }
	    vector_elevationData.push_back(row);
	    row.clear();
            n_row++;
        }
        e_file.close();

        n_col /= n_row;
    }
    else
    {
        std::cout << "Problem opening the path file" << std::endl;
    }
}

void readPath(std::string s_path_file,
	      std::vector<Waypoint> &vw_path)
{
    std::string line;
    std::ifstream e_file(s_path_file.c_str(), std::ios::in);
    vw_path.clear();
    Waypoint w_current;
    int n_row = 0;
    if (e_file.is_open())
    {
        while (std::getline(e_file, line))
        {
            std::stringstream ss(line);
            std::string cell;
	    double val;

            std::getline(ss, cell, ' ');
            std::stringstream numeric_value(cell);
            numeric_value >> val;
            w_current.position[0] = val;

            std::getline(ss, cell, ' ');
            std::stringstream numeric_value1(cell);
            numeric_value1 >> val;
            w_current.position[1] = val;
            
	    std::getline(ss, cell, ' ');
            std::stringstream numeric_value2(cell);
            numeric_value2 >> val;
            w_current.heading = val;


	    vw_path.push_back(w_current);
	    n_row++;
        }
        e_file.close();
    }
    else
    {
        std::cout << "Problem opening the map file" << std::endl;
    }

}

