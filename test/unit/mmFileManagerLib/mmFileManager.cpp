#include "mmFileManager.h"

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
	throw std::exception();
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

Waypoint getWaypoint(std::string s_path_file)
{
    std::string line;
    std::ifstream e_file(s_path_file.c_str(), std::ios::in);

    Waypoint w_current;

    std::getline(e_file, line);
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
    w_current.position[2] = val;

    e_file.close();
    return w_current;
}

void savePath(std::vector<base::Waypoint> *roverPath, std::string s_path_file)
{
    std::ofstream pathFile;
    pathFile.open(s_path_file);
    for (int j = 0; j < roverPath->size(); j++)
    {
        pathFile << roverPath->at(j).position[0] << " "
                 << roverPath->at(j).position[1] << " "
                 << roverPath->at(j).position[2] << " "
                 << roverPath->at(j).heading << "\n";
    }
    pathFile.close();
}

void saveProfile(std::vector<std::vector<double>> *pvvd_arm_motion_profile, std::string s_path_file)
{
    std::ofstream f_arm_motion;
    f_arm_motion.open(s_path_file);
    for (int j = 0; j < pvvd_arm_motion_profile->size(); j++)
    {
        for (int i = 0; i < (*pvvd_arm_motion_profile)[0].size(); i++)
        {
            f_arm_motion << (*pvvd_arm_motion_profile)[j][i] << " ";
        }
        f_arm_motion << "\n";
    }
    f_arm_motion.close();

}

void saveVolume(std::vector<std::vector<std::vector<double>>> * pvvvd_volume, std::string s_path_file)
{
    std::ofstream f_volume;
    f_volume.open(s_path_file);

    f_volume << (*pvvvd_volume).size() << " " << (*pvvvd_volume)[0].size() << " " << (*pvvvd_volume)[0][0].size() << "\n";
    for (int j = 0; j < pvvvd_volume->size(); j++)
    {
        for (int i = 0; i < (*pvvvd_volume)[0].size(); i++)
        {
            for (int k = 0; k < (*pvvvd_volume)[0][0].size(); k++)
            {
                f_volume << (*pvvvd_volume)[j][i][k] << " ";
            }
        }
        f_volume << "\n";
    }

    f_volume.close();

}
