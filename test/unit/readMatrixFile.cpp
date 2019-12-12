#include "readMatrixFile.h"

void readMatrixFile(std::string map_file,
                    std::vector<std::vector<double>> &vector_elevationData)
{
    std::cout << "Reading map " << map_file << std::endl;
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
        std::cout << "Map of " << n_col << " x " << n_row << " loaded." << std::endl;
    }
    else
    {
        std::cout << "Problem opening the path file" << std::endl;
    }
    // double elevationData[rgDem->v_elevationData_m.size()];
    // double elevationData[vector_elevationData.size()];
    // rgDem->p_heightData_m = elevationData;
    /*for (uint j = 0; j<rgDem->rows; j++)
    {
        for (uint i = 0; i<rgDem->cols; i++)
        {
            std::cout << "Initial Value is " << rgDem->p_heightData_m[i+j*rgDem->cols] << std::endl;
        }
    }*/
}

void readPath(std::string s_path_file,
	      std::vector<Waypoint> &vw_path)
{
    std::cout << "Reading path file " << s_path_file << std::endl;
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

        std::cout << "Path of " << n_row << " waypoints loaded." << std::endl;
    }
    else
    {
        std::cout << "Problem opening the map file" << std::endl;
    }

}

void readIntVector(std::string s_vector_file,
		   std::vector<int> &vi_input)
{
    std::cout << "Reading integer vector file " << s_vector_file << std::endl;
    std::string line;
    std::ifstream e_file(s_vector_file.c_str(), std::ios::in);
    vi_input.clear();
    int n_row = 0;
    if (e_file.is_open())
    {
        while (std::getline(e_file, line))
        {
            std::stringstream ss(line);
            std::string cell;
	    int val;

            std::getline(ss, cell, ' ');
            std::stringstream numeric_value(cell);
            numeric_value >> val;            
	    vi_input.push_back(val);
	    n_row++;
        }
        e_file.close();

        std::cout << "Integer vector of " << n_row << " elements loaded." << std::endl;
    }
    else
    {
        std::cout << "Problem opening the map file" << std::endl;
    }
}
