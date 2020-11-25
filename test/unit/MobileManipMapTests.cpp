#include "MMStatus.h"
#include "MobileManipMap.h"
#include "mmFileManager.h"
#include <fstream>
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

TEST(MMMapTest, set_config_test)
{
    MobileManipMap mmmap(false);
    mmmap.setThresholdValues( 0.0,-1.0,-1.0,-1.0);
    // Do the same with config values
}


TEST(MMMapTest, nominal_working_test_spacehall)
{
    // Input Elevation Matrix is read
    std::vector<std::vector<double>> vvd_elevation_data, vvd_validity_data;
    
    // A dummy Rover Guidance based DEM is created
    ASSERT_NO_THROW(readMatrixFileCommas("test/unit/data/input/MMMapTest/RgDem.p_heightData_m.csv",
                   vvd_elevation_data)) << "Input DEM file is missing";
    
    ASSERT_NO_THROW(readMatrixFileCommas("test/unit/data/input/MMMapTest/RgDem.p_pointValidityFlag.csv",
                   vvd_validity_data)) << "Input DEM file is missing";
    std::cout << "Matrix size is " << vvd_elevation_data[0].size() << "x" << vvd_elevation_data.size() << std::endl; 
    std::cout << "Matrix size is " << vvd_validity_data[0].size() << "x" << vvd_validity_data.size() << std::endl; 
    double res = 0.1; // meters
    RoverGuidance_Dem *prgd_dummy_dem = new RoverGuidance_Dem;
    double dummyArray[vvd_elevation_data.size() * vvd_elevation_data[0].size()];
    int8_t validityArray[vvd_elevation_data.size() * vvd_elevation_data[0].size()];
    prgd_dummy_dem->p_heightData_m = dummyArray;
    prgd_dummy_dem->p_pointValidityFlag = validityArray;
    prgd_dummy_dem->cols = vvd_elevation_data[0].size();
    prgd_dummy_dem->rows = vvd_elevation_data.size();
    prgd_dummy_dem->nodeSize_m = res;
    prgd_dummy_dem->mapOrigin_m_Mlg[0] = -11.05;
    prgd_dummy_dem->mapOrigin_m_Mlg[1] = -6.05;
    prgd_dummy_dem->mapOrigin_m_Mlg[2] = 0.0;
    for (uint j = 0; j < vvd_elevation_data.size(); j++)
    {
        for (uint i = 0; i < vvd_elevation_data[0].size(); i++)
        {
            prgd_dummy_dem->p_heightData_m[i + j * vvd_elevation_data[0].size()]
                = vvd_elevation_data[j][i];
            prgd_dummy_dem->p_pointValidityFlag[i + j * vvd_elevation_data[0].size()] = vvd_validity_data[j][i];
        }
    }

    base::Waypoint samplePos;
    samplePos.position[0] = 3.0;
    samplePos.position[1] = 14.0;

    MobileManipMap dummyMap(true);
    dummyMap.setThresholdValues( -1.0,-1.0,0.01,-1.0);
    ASSERT_EQ(dummyMap.loadDEM((*prgd_dummy_dem)),0);

    ASSERT_EQ(dummyMap.computeFACE(samplePos, 0.5, 1.0, 1.3),0);
    
    std::vector<std::vector<double>> costMap, elevationMap, slopeMap, sdMap;
    std::vector<std::vector<int>> traversabilityMap;
    std::vector<std::vector<int8_t>> validityMap;
    costMap.resize(prgd_dummy_dem->rows);
    validityMap.resize(prgd_dummy_dem->rows);
    for (uint i = 0; i < prgd_dummy_dem->rows; i++)
    {
        costMap[i].resize(prgd_dummy_dem->cols);
        validityMap[i].resize(prgd_dummy_dem->cols);
    }
    dummyMap.getCostMap(costMap);
    dummyMap.getTraversabilityMap(traversabilityMap);
    dummyMap.getValidityMap(validityMap);
    dummyMap.getElevationMap(elevationMap);
    dummyMap.getSlopeMap(slopeMap);
    dummyMap.getSDMap(sdMap);

    writeMatrixFile("test/unit/data/results/MMMapTest/spacehall_validityMap.txt", validityMap);    
    writeMatrixFile("test/unit/data/results/MMMapTest/spacehall_elevationMap.txt", elevationMap);    
    writeMatrixFile("test/unit/data/results/MMMapTest/spacehall_slopeMap.txt", slopeMap);    
    writeMatrixFile("test/unit/data/results/MMMapTest/spacehall_sdMap.txt", sdMap);    
    writeMatrixFile("test/unit/data/results/MMMapTest/spacehall_costMap.txt", costMap);    
    writeMatrixFile("test/unit/data/results/MMMapTest/spacehall_traversabilityMap.txt", traversabilityMap);    

}



TEST(MMMapTest, nominal_working_test_galopprennbahnwest)
{
    // Input Elevation Matrix is read
    std::vector<std::vector<double>> vvd_elevation_data, vvd_validity_data;
    
    // A dummy Rover Guidance based DEM is created
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMapTest/GalopprennbahnWest_Zone01blurred_10cmDEM.csv",
                   vvd_elevation_data)) << "Input DEM file is missing";
    double res = 0.1; // meters
    RoverGuidance_Dem *prgd_dummy_dem = new RoverGuidance_Dem;
    double dummyArray[vvd_elevation_data.size() * vvd_elevation_data[0].size()];
    int8_t validityArray[vvd_elevation_data.size() * vvd_elevation_data[0].size()];
    prgd_dummy_dem->p_heightData_m = dummyArray;
    prgd_dummy_dem->p_pointValidityFlag = validityArray;
    prgd_dummy_dem->cols = vvd_elevation_data[0].size();
    prgd_dummy_dem->rows = vvd_elevation_data.size();
    prgd_dummy_dem->nodeSize_m = res;
    prgd_dummy_dem->mapOrigin_m_Mlg[0] = 0.0;
    prgd_dummy_dem->mapOrigin_m_Mlg[1] = 0.0;
    prgd_dummy_dem->mapOrigin_m_Mlg[2] = 0.0;
    for (uint j = 0; j < vvd_elevation_data.size(); j++)
    {
        for (uint i = 0; i < vvd_elevation_data[0].size(); i++)
        {
            prgd_dummy_dem->p_heightData_m[i + j * vvd_elevation_data[0].size()]
                = vvd_elevation_data[j][i];
            prgd_dummy_dem->p_pointValidityFlag[i + j * vvd_elevation_data[0].size()] = 1;
        }
    }

    MobileManipMap dummyMap(false);
    ASSERT_EQ(dummyMap.loadDEM((*prgd_dummy_dem)),0);

    base::Waypoint samplePos;
    samplePos.position[0] = 17.885;
    samplePos.position[1] = 12.405;

    ASSERT_EQ(dummyMap.computeFACE(samplePos, 1.0, 1.344, 1.584),0);
    std::cout << "Cost map is computed" << std::endl; 
    double d_elevation_min = dummyMap.getMinElevation();

    std::vector<std::vector<double>> costMap, slopeMap, sdMap;
    std::vector<std::vector<int>> traversabilityMap;
    std::vector<std::vector<int8_t>> validityMap;
    costMap.resize(prgd_dummy_dem->rows);
    traversabilityMap.resize(prgd_dummy_dem->rows);
    validityMap.resize(prgd_dummy_dem->rows);
    for (uint i = 0; i < prgd_dummy_dem->rows; i++)
    {
        costMap[i].resize(prgd_dummy_dem->cols);
        validityMap[i].resize(prgd_dummy_dem->cols);
    }
    dummyMap.getCostMap(costMap);
    dummyMap.getValidityMap(validityMap);
    dummyMap.getSlopeMap(slopeMap);
    dummyMap.getSDMap(sdMap);
    dummyMap.getTraversabilityMap(traversabilityMap);

    writeMatrixFile("test/unit/data/results/MMMapTest/gbwest_validityMap.txt", validityMap);    
    writeMatrixFile("test/unit/data/results/MMMapTest/gbwest_costMap.txt", costMap);    
    writeMatrixFile("test/unit/data/results/MMMapTest/gbwest_slopeMap.txt", slopeMap);    
    writeMatrixFile("test/unit/data/results/MMMapTest/gbwest_sdMap.txt", sdMap);    
    writeMatrixFile("test/unit/data/results/MMMapTest/gbwest_traversabilityMap.txt", traversabilityMap);    
}

TEST(MMMapTest, nominal_working_test_exrColmenar)
{
    // Checking EXR DEM sent by GMV
    cv::FileStorage opencv_file("test/unit/data/input/MMMapTest/1212.1628281354.363863_colmenar_subdem_id_1.ext", cv::FileStorage::READ);
    if (!opencv_file.isOpened())
    {
        std::cout << "Did not open" << std::endl;
    }
    cv::Mat exrDEM;
    opencv_file["mapMatrix"] >> exrDEM;
    std::cout << "Size is " << exrDEM.size() << std::endl; 
    //cv::Mat mGlobalDEM = cv::imread("test/unit/data/input/MMMapTest/1212.1628281354.363863_colmenar_subdem_id_1ag.ext", cv::IMREAD_ANYDEPTH); 
    cv::imshow("exr", exrDEM);
    waitKey(0);
    
    opencv_file.release();
    std::ofstream demFile;
    demFile.open("test/unit/data/results/MMMapTest/exrColmenar.txt");
    for (int j = 0; j < 301; j++)
    {
        for (int i = 0; i < 301; i++)
        {
            demFile << exrDEM.at<float>(j, i) << " ";
        }
        demFile << "\n";
    }

    
    double d_global_res = 0.1; // meters
    RoverGuidance_Dem *prgd_dummy_dem = new RoverGuidance_Dem;
    unsigned int ui_cols, ui_rows;
    ui_cols = 301;
    ui_rows = 301;
    double dummyArray[ui_cols * ui_rows];
    int8_t validityArray[ui_cols * ui_rows];
    prgd_dummy_dem->p_heightData_m = dummyArray;
    prgd_dummy_dem->p_pointValidityFlag = validityArray;
    prgd_dummy_dem->cols = ui_cols;
    prgd_dummy_dem->rows = ui_rows;
    prgd_dummy_dem->nodeSize_m = d_global_res;
    prgd_dummy_dem->mapOrigin_m_Mlg[0] = -15.0;
    prgd_dummy_dem->mapOrigin_m_Mlg[1] = -15.0;
    prgd_dummy_dem->mapOrigin_m_Mlg[2] = 0.0;
    for (uint j = 0; j < ui_rows; j++)
    {
        for (uint i = 0; i < ui_cols; i++)
        {
            prgd_dummy_dem->p_heightData_m[i + j * ui_cols]
                = exrDEM.at<float>(j, i);
            prgd_dummy_dem->p_pointValidityFlag[i + j * ui_cols] = 1;
        }
    }

    base::Waypoint samplePos;
    samplePos.position[0] = 3.0;
    samplePos.position[1] = 3.0;

    MobileManipMap dummyMap(false);
    ASSERT_EQ(dummyMap.loadDEM((*prgd_dummy_dem)),0);
    std::cout << "DEM is loaded" << std::endl; 
    std::ofstream validityMapFile, costMapFile, slopeMapFile, sdMapFile, traversabilityMapFile;
    
    std::vector<std::vector<double>> costMap, slopeMap, sdMap;
    std::vector<std::vector<int>> traversabilityMap;
    std::vector<std::vector<int8_t>> validityMap;
    dummyMap.getValidityMap(validityMap);
    dummyMap.getSlopeMap(slopeMap);
    dummyMap.getSDMap(sdMap);

    writeMatrixFile("test/unit/data/results/MMMapTest/exrcolmenar_validityMap.txt", validityMap);    
    writeMatrixFile("test/unit/data/results/MMMapTest/exrcolmenar_slopeMap.txt", slopeMap);    
    writeMatrixFile("test/unit/data/results/MMMapTest/exrcolmenar_sdMap.txt", sdMap);    

    ASSERT_EQ(dummyMap.computeFACE(samplePos, 1.0, 1.344, 1.584),0);
    std::cout << "Cost map is computed" << std::endl; 

    costMap.resize(prgd_dummy_dem->rows);
    traversabilityMap.resize(prgd_dummy_dem->rows);
    validityMap.resize(prgd_dummy_dem->rows);
    for (uint i = 0; i < prgd_dummy_dem->rows; i++)
    {
        costMap[i].resize(prgd_dummy_dem->cols);
        validityMap[i].resize(prgd_dummy_dem->cols);
    }
    dummyMap.getCostMap(costMap);

    writeMatrixFile("test/unit/data/results/MMMapTest/exrcolmenar_costMap.txt", costMap);    
    writeMatrixFile("test/unit/data/results/MMMapTest/exrcolmenar_traversabilityMap.txt", traversabilityMap);     
}




TEST(MMMapTest, nominal_working_test)
{
    // Input Elevation Matrix is read
    std::vector<std::vector<double>> vvd_elevation_data, vvd_validity_data;
    
/*
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMapTest/ColmenarRocks_Nominal_10cmDEM.csv",
                   vvd_elevation_data)) << "Input DEM file is missing";
  */
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMapTest/GalopprennbahnWest_Zone01blurred_10cmDEM.csv",
                   vvd_elevation_data)) << "Input DEM file is missing";

/*
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMapTest/RH1_Zone1_10cmDEM.csv",
                   vvd_elevation_data)) << "Input DEM file is missing";
 */ 
/*
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMapTest/RG_Colmenar_10cmDEM.csv",
                   vvd_elevation_data)) << "Input DEM file is missing";
    
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMapTest/RG_Colmenar_10cmValidity.csv",
                   vvd_validity_data)) << "Input DEM file is missing";
  */  
    std::cout << "Input Matrix is successfully read" << std::endl; 
    std::cout << "Matrix size is " << vvd_elevation_data[0].size() << "x" << vvd_elevation_data.size() << std::endl; 
    double res = 0.1; // meters

    // A dummy Rover Guidance based DEM is created
    RoverGuidance_Dem *prgd_dummy_dem = new RoverGuidance_Dem;
    double dummyArray[vvd_elevation_data.size() * vvd_elevation_data[0].size()];
    int8_t validityArray[vvd_elevation_data.size() * vvd_elevation_data[0].size()];
    prgd_dummy_dem->p_heightData_m = dummyArray;
    prgd_dummy_dem->p_pointValidityFlag = validityArray;
    prgd_dummy_dem->cols = vvd_elevation_data[0].size();
    prgd_dummy_dem->rows = vvd_elevation_data.size();
    prgd_dummy_dem->nodeSize_m = res;
    prgd_dummy_dem->mapOrigin_m_Mlg[0] = 0.0;
    prgd_dummy_dem->mapOrigin_m_Mlg[1] = 0.0;
    prgd_dummy_dem->mapOrigin_m_Mlg[2] = 0.0;
    for (uint j = 0; j < vvd_elevation_data.size(); j++)
    {
        for (uint i = 0; i < vvd_elevation_data[0].size(); i++)
        {
            prgd_dummy_dem->p_heightData_m[i + j * vvd_elevation_data[0].size()]
                = vvd_elevation_data[j][i];
            prgd_dummy_dem->p_pointValidityFlag[i + j * vvd_elevation_data[0].size()] = 1;
            //prgd_dummy_dem->p_pointValidityFlag[i + j * vvd_elevation_data[0].size()] = (uint8_t)vvd_validity_data[j][i];
        }
    }

    std::cout << "Rover Guidance DEM is successfully created" << std::endl; 
    std::cout << vvd_elevation_data[0][0] << std::endl; 

    base::Waypoint samplePos;
    samplePos.position[0] = 17.885;//5.6;
    samplePos.position[1] = 12.405;
    //ASSERT_NO_THROW(samplePos = getWaypoint("test/unit/data/input/MMMapTest/sample_pos.txt")) << "Input Waypoint file is missing";

    MobileManipMap dummyMap(false);
    ASSERT_EQ(dummyMap.loadDEM((*prgd_dummy_dem)),0);
    std::cout << "DEM is loaded" << std::endl; 
    ASSERT_EQ(dummyMap.computeFACE(samplePos, 1.0, 1.344, 1.584),0);
    std::cout << "Cost map is computed" << std::endl; 
    double d_elevation_min = dummyMap.getMinElevation();
    //ASSERT_LT(d_elevation_min, 1008.55);
    //ASSERT_GT(d_elevation_min, 1008.53);

    std::vector<std::vector<double>> costMap, slopeMap, sdMap;
    std::vector<std::vector<int>> traversabilityMap;
    std::vector<std::vector<int8_t>> validityMap;
    costMap.resize(prgd_dummy_dem->rows);
    traversabilityMap.resize(prgd_dummy_dem->rows);
    validityMap.resize(prgd_dummy_dem->rows);
    for (uint i = 0; i < prgd_dummy_dem->rows; i++)
    {
        costMap[i].resize(prgd_dummy_dem->cols);
        validityMap[i].resize(prgd_dummy_dem->cols);
    }
    dummyMap.getCostMap(costMap);
    dummyMap.getValidityMap(validityMap);
    dummyMap.getSlopeMap(slopeMap);
    dummyMap.getSDMap(sdMap);
    /*ASSERT_EQ(costMap.size(), 80);
    ASSERT_EQ(costMap[0].size(), 80);
*/
    std::ofstream validityMapFile, costMapFile, slopeMapFile, sdMapFile, traversabilityMapFile;

    validityMapFile.open("test/unit/data/results/MMMapTest/validityMap.txt");

    for (int j = 0; j < validityMap.size(); j++)
    {
        for (int i = 0; i < validityMap[0].size(); i++)
        {
            validityMapFile << (double)validityMap[j][i] << " ";
        }
        validityMapFile << "\n";
    }

    validityMapFile.close();

    costMapFile.open("test/unit/data/results/MMMapTest/costMap.txt");

    for (int j = 0; j < costMap.size(); j++)
    {
        for (int i = 0; i < costMap[0].size(); i++)
        {
            costMapFile << costMap[j][i] << " ";
        }
        costMapFile << "\n";
    }

    costMapFile.close();

    slopeMapFile.open("test/unit/data/results/MMMapTest/slopeMap.txt");

    for (int j = 0; j < slopeMap.size(); j++)
    {
        for (int i = 0; i < slopeMap[0].size(); i++)
        {
            slopeMapFile << slopeMap[j][i] << " ";
        }
        slopeMapFile << "\n";
    }

    slopeMapFile.close();

    sdMapFile.open("test/unit/data/results/MMMapTest/sdMap.txt");

    for (int j = 0; j < sdMap.size(); j++)
    {
        for (int i = 0; i < sdMap[0].size(); i++)
        {
            sdMapFile << sdMap[j][i] << " ";
        }
        sdMapFile << "\n";
    }

    sdMapFile.close();

    traversabilityMapFile.open("test/unit/data/results/MMMapTest/traversabilityMap.txt");
    dummyMap.getTraversabilityMap(traversabilityMap);
    for (int j = 0; j < traversabilityMap.size(); j++)
    {
        for (int i = 0; i < traversabilityMap[0].size(); i++)
        {
            traversabilityMapFile << traversabilityMap[j][i] << " ";
        }
        traversabilityMapFile << "\n";
    }

    traversabilityMapFile.close();

}


TEST(MMMapTest, nominal_working_test_MagLocCam)
{
    cv::Mat mGlobalDEM = cv::imread("test/unit/data/input/MMMapTest/MagDEMs/00600.exr", cv::IMREAD_ANYDEPTH); 
    cv::Mat mLocalDEM = cv::imread("test/unit/data/input/MMMapTest/MagLocCam/00600.exr", cv::IMREAD_ANYDEPTH);
    cv::imshow("exr", mLocalDEM);
    waitKey(0);
    std::ofstream demFile;
    demFile.open("test/unit/data/results/MMMapTest/magLocCam.txt");
    for (int j = 0; j < 400; j++)
    {
        for (int i = 0; i < 400; i++)
        {
            demFile << mLocalDEM.at<float>(j, i) * 0.001 << " ";
        }
        demFile << "\n";
    }

    double d_global_res = 0.1; // meters
    double d_local_res = 0.04; // meters
    RoverGuidance_Dem *prgd_dummy_dem = new RoverGuidance_Dem;
    unsigned int ui_cols, ui_rows;
    ui_cols = 160;
    ui_rows = 160;
    double dummyArray[ui_cols * ui_rows];
    int8_t validityArray[ui_cols * ui_rows];
    prgd_dummy_dem->p_heightData_m = dummyArray;
    prgd_dummy_dem->p_pointValidityFlag = validityArray;
    prgd_dummy_dem->cols = ui_cols;
    prgd_dummy_dem->rows = ui_rows;
    prgd_dummy_dem->nodeSize_m = d_global_res;
    prgd_dummy_dem->mapOrigin_m_Mlg[0] = 0.0;
    prgd_dummy_dem->mapOrigin_m_Mlg[1] = 0.0;
    prgd_dummy_dem->mapOrigin_m_Mlg[2] = 0.0;
    for (uint j = 0; j < ui_rows; j++)
    {
        for (uint i = 0; i < ui_cols; i++)
        {
            prgd_dummy_dem->p_heightData_m[i + j * ui_cols]
                = mGlobalDEM.at<float>((int)(2.5*j), (int)(2.5*i)) * 0.001;
            if (mGlobalDEM.at<float>((int)(2.5*j), (int)(2.5*i)) == -1500)
	    {
	        prgd_dummy_dem->p_pointValidityFlag[i + j * ui_cols] = 0;
	    }
	    else
	    {
	        prgd_dummy_dem->p_pointValidityFlag[i + j * ui_cols] = 1;
	    }
        }
    }

    std::ofstream globaldemFile, validFile;
    globaldemFile.open("test/unit/data/results/MMMapTest/magGlobalDEM.txt");
    validFile.open("test/unit/data/results/MMMapTest/magGlobalDEMvalidity.txt");
    for (int j = 0; j < ui_rows; j++)
    {
        for (int i = 0; i < ui_cols; i++)
        {
            globaldemFile << prgd_dummy_dem->p_heightData_m[i + j * ui_cols] << " ";
            validFile << (double)prgd_dummy_dem->p_pointValidityFlag[i + j * ui_cols] << " ";
        }
        globaldemFile << "\n";
        validFile << "\n";
    }

    globaldemFile.close(); 
    validFile.close(); 

    base::Waypoint samplePos;
    samplePos.position[0] = 7.5;//5.6;
    samplePos.position[1] = 10.0;
    //ASSERT_NO_THROW(samplePos = getWaypoint("test/unit/data/input/MMMapTest/sample_pos.txt")) << "Input Waypoint file is missing";

    MobileManipMap dummyMap(false);
    ASSERT_EQ(dummyMap.loadDEM((*prgd_dummy_dem)),0);
    std::cout << "DEM is loaded" << std::endl; 
    ASSERT_EQ(dummyMap.computeFACE(samplePos, 1.0, 1.344, 1.584),0);
    std::cout << "Cost map is computed" << std::endl; 
    double d_elevation_min = dummyMap.getMinElevation();
    //ASSERT_LT(d_elevation_min, 1008.55);
    //ASSERT_GT(d_elevation_min, 1008.53);

    std::vector<std::vector<double>> costMap, slopeMap, sdMap;
    std::vector<std::vector<int>> traversabilityMap;
    std::vector<std::vector<int8_t>> validityMap;
    costMap.resize(prgd_dummy_dem->rows);
    traversabilityMap.resize(prgd_dummy_dem->rows);
    validityMap.resize(prgd_dummy_dem->rows);
    for (uint i = 0; i < prgd_dummy_dem->rows; i++)
    {
        costMap[i].resize(prgd_dummy_dem->cols);
        validityMap[i].resize(prgd_dummy_dem->cols);
    }
    dummyMap.getCostMap(costMap);
    dummyMap.getValidityMap(validityMap);
    dummyMap.getSlopeMap(slopeMap);
    dummyMap.getSDMap(sdMap);

    std::ofstream validityMapFile, costMapFile, slopeMapFile, sdMapFile, traversabilityMapFile;
    validityMapFile.open("test/unit/data/results/MMMapTest/MAGGlobalvalidityMap.txt");

    for (int j = 0; j < validityMap.size(); j++)
    {
        for (int i = 0; i < validityMap[0].size(); i++)
        {
            validityMapFile << (double)validityMap[j][i] << " ";
        }
        validityMapFile << "\n";
    }

    validityMapFile.close();

    costMapFile.open("test/unit/data/results/MMMapTest/MAGGlobalcostMap.txt");

    for (int j = 0; j < costMap.size(); j++)
    {
        for (int i = 0; i < costMap[0].size(); i++)
        {
            costMapFile << costMap[j][i] << " ";
        }
        costMapFile << "\n";
    }

    costMapFile.close();

    slopeMapFile.open("test/unit/data/results/MMMapTest/MAGGlobalslopeMap.txt");

    for (int j = 0; j < slopeMap.size(); j++)
    {
        for (int i = 0; i < slopeMap[0].size(); i++)
        {
            slopeMapFile << slopeMap[j][i] << " ";
        }
        slopeMapFile << "\n";
    }

    slopeMapFile.close();

    sdMapFile.open("test/unit/data/results/MMMapTest/MAGGlobalsdMap.txt");

    for (int j = 0; j < sdMap.size(); j++)
    {
        for (int i = 0; i < sdMap[0].size(); i++)
        {
            sdMapFile << sdMap[j][i] << " ";
        }
        sdMapFile << "\n";
    }

    sdMapFile.close();

    traversabilityMapFile.open("test/unit/data/results/MMMapTest/MAGGlobaltraversabilityMap.txt");
    dummyMap.getTraversabilityMap(traversabilityMap);
    for (int j = 0; j < traversabilityMap.size(); j++)
    {
        for (int i = 0; i < traversabilityMap[0].size(); i++)
        {
            traversabilityMapFile << traversabilityMap[j][i] << " ";
        }
        traversabilityMapFile << "\n";
    }

    traversabilityMapFile.close();


}

TEST(MMMapTest, nominal_working_test_MagDEMs)
{
    cv::Mat dem01 = cv::imread("test/unit/data/input/MMMapTest/MagDEMs/00600.exr", cv::IMREAD_ANYDEPTH);
    cv::imshow("exr", dem01);
    waitKey(0);
    double res = 0.04; // meters

    // A dummy Rover Guidance based DEM is created
    RoverGuidance_Dem *prgd_dummy_dem = new RoverGuidance_Dem;
    double dummyArray[400 * 400];
    int8_t validityArray[400 * 400];
    prgd_dummy_dem->p_heightData_m = dummyArray;
    prgd_dummy_dem->p_pointValidityFlag = validityArray;
    prgd_dummy_dem->cols = 400;
    prgd_dummy_dem->rows = 400;
    prgd_dummy_dem->nodeSize_m = res;
    prgd_dummy_dem->mapOrigin_m_Mlg[0] = 0.0;
    prgd_dummy_dem->mapOrigin_m_Mlg[1] = 0.0;
    prgd_dummy_dem->mapOrigin_m_Mlg[2] = 0.0;
    for (uint j = 0; j < 400; j++)
    {
        for (uint i = 0; i < 400; i++)
        {
            prgd_dummy_dem->p_heightData_m[i + j * 400]
                = dem01.at<float>(j, i) * 0.001;
            if (dem01.at<float>(j,i) == -1500)
	    {
	        prgd_dummy_dem->p_pointValidityFlag[i + j * 400] = 0;
	    }
	    else
	    {
	        prgd_dummy_dem->p_pointValidityFlag[i + j * 400] = 1;
	    }
        }
    }

    std::ofstream demFile, validFile;
    demFile.open("test/unit/data/results/MMMapTest/magDEM.txt");
    validFile.open("test/unit/data/results/MMMapTest/magDEMvalidity.txt");
    for (int j = 0; j < 400; j++)
    {
        for (int i = 0; i < 400; i++)
        {
            demFile << prgd_dummy_dem->p_heightData_m[i + j * 400] << " ";
            validFile << (double)prgd_dummy_dem->p_pointValidityFlag[i + j * 400] << " ";
        }
        demFile << "\n";
        validFile << "\n";
    }

    demFile.close(); 
    validFile.close(); 

    base::Waypoint samplePos;
    samplePos.position[0] = 14.0;//5.6;
    samplePos.position[1] = 6.5;
    //ASSERT_NO_THROW(samplePos = getWaypoint("test/unit/data/input/MMMapTest/sample_pos.txt")) << "Input Waypoint file is missing";

    MobileManipMap dummyMap(true);
    ASSERT_EQ(dummyMap.loadDEM((*prgd_dummy_dem)),0);
    std::cout << "DEM is loaded" << std::endl; 
    ASSERT_EQ(dummyMap.computeFACE(samplePos, 1.0, 1.344, 1.584),0);
    std::cout << "Cost map is computed" << std::endl; 
    double d_elevation_min = dummyMap.getMinElevation();
    //ASSERT_LT(d_elevation_min, 1008.55);
    //ASSERT_GT(d_elevation_min, 1008.53);

    std::vector<std::vector<double>> costMap, slopeMap, sdMap;
    std::vector<std::vector<int>> traversabilityMap;
    std::vector<std::vector<int8_t>> validityMap;
    costMap.resize(prgd_dummy_dem->rows);
    traversabilityMap.resize(prgd_dummy_dem->rows);
    validityMap.resize(prgd_dummy_dem->rows);
    for (uint i = 0; i < prgd_dummy_dem->rows; i++)
    {
        costMap[i].resize(prgd_dummy_dem->cols);
        validityMap[i].resize(prgd_dummy_dem->cols);
    }
    dummyMap.getCostMap(costMap);
    dummyMap.getValidityMap(validityMap);
    dummyMap.getSlopeMap(slopeMap);
    dummyMap.getSDMap(sdMap);

    std::ofstream validityMapFile, costMapFile, slopeMapFile, sdMapFile, traversabilityMapFile;
    validityMapFile.open("test/unit/data/results/MMMapTest/MAGvalidityMap.txt");

    for (int j = 0; j < validityMap.size(); j++)
    {
        for (int i = 0; i < validityMap[0].size(); i++)
        {
            validityMapFile << (double)validityMap[j][i] << " ";
        }
        validityMapFile << "\n";
    }

    validityMapFile.close();

    costMapFile.open("test/unit/data/results/MMMapTest/MAGcostMap.txt");

    for (int j = 0; j < costMap.size(); j++)
    {
        for (int i = 0; i < costMap[0].size(); i++)
        {
            costMapFile << costMap[j][i] << " ";
        }
        costMapFile << "\n";
    }

    costMapFile.close();

    slopeMapFile.open("test/unit/data/results/MMMapTest/MAGslopeMap.txt");

    for (int j = 0; j < slopeMap.size(); j++)
    {
        for (int i = 0; i < slopeMap[0].size(); i++)
        {
            slopeMapFile << slopeMap[j][i] << " ";
        }
        slopeMapFile << "\n";
    }

    slopeMapFile.close();

    sdMapFile.open("test/unit/data/results/MMMapTest/MAGsdMap.txt");

    for (int j = 0; j < sdMap.size(); j++)
    {
        for (int i = 0; i < sdMap[0].size(); i++)
        {
            sdMapFile << sdMap[j][i] << " ";
        }
        sdMapFile << "\n";
    }

    sdMapFile.close();

    traversabilityMapFile.open("test/unit/data/results/MMMapTest/MAGtraversabilityMap.txt");
    dummyMap.getTraversabilityMap(traversabilityMap);
    for (int j = 0; j < traversabilityMap.size(); j++)
    {
        for (int i = 0; i < traversabilityMap[0].size(); i++)
        {
            traversabilityMapFile << traversabilityMap[j][i] << " ";
        }
        traversabilityMapFile << "\n";
    }

    traversabilityMapFile.close();


}



TEST(MMMapTest, nominal_working_test_2)
{
    // Input Elevation Matrix is read
    std::vector<std::vector<double>> vvd_elevation_data, vvd_validity_data;
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMapTest/ColmenarRocks_splitted_10cmDEM.csv",
                   vvd_elevation_data)) << "Input DEM file is missing";
    double res = 0.1; // meters

    // A dummy Rover Guidance based DEM is created
    RoverGuidance_Dem *prgd_dummy_dem = new RoverGuidance_Dem;
    double dummyArray[vvd_elevation_data.size() * vvd_elevation_data[0].size()];
    int8_t validityArray[vvd_elevation_data.size() * vvd_elevation_data[0].size()];
    prgd_dummy_dem->p_heightData_m = dummyArray;
    prgd_dummy_dem->p_pointValidityFlag = validityArray;
    prgd_dummy_dem->cols = vvd_elevation_data[0].size();
    prgd_dummy_dem->rows = vvd_elevation_data.size();
    prgd_dummy_dem->nodeSize_m = res;
    for (uint j = 0; j < vvd_elevation_data.size(); j++)
    {
        for (uint i = 0; i < vvd_elevation_data[0].size(); i++)
        {
            prgd_dummy_dem->p_heightData_m[i + j * vvd_elevation_data[0].size()]
                = vvd_elevation_data[j][i];
            prgd_dummy_dem->p_pointValidityFlag[i + j * vvd_elevation_data[0].size()] = 1;
        }
    }

    std::vector<std::vector<double>> costMap;
    costMap.resize(prgd_dummy_dem->rows);
    for (uint i = 0; i < prgd_dummy_dem->rows; i++)
    {
        costMap[i].resize(prgd_dummy_dem->cols);
    }
    std::ofstream costMapFile;
    MobileManipMap mmmap_no_shadowing;
    mmmap_no_shadowing.loadDEM((*prgd_dummy_dem));
    costMapFile.open("test/unit/data/results/MMMapTest/costMap_splittedMap.txt");
    mmmap_no_shadowing.getCostMap(costMap);
    for (int j = 0; j < costMap.size(); j++)
    {
        for (int i = 0; i < costMap[0].size(); i++)
        {
            costMapFile << costMap[j][i] << " ";
        }
        costMapFile << "\n";
    }

    costMapFile.close();

}

// TODO - FIX this with new constructor
/*TEST(MMMapTest, dem_format_error_test)
{
    // Input Elevation Matrix is read
    std::vector<std::vector<double>> vvd_elevation_data;
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMapTest/ColmenarRocks_smaller_10cmDEM.csv",
                   vvd_elevation_data));
    double res = 0.1; // meters

    // A dummy Rover Guidance based DEM is created
    RoverGuidance_Dem *prgd_dummy_dem = new RoverGuidance_Dem;
    double dummyArray[vvd_elevation_data.size() * vvd_elevation_data[0].size()];
    prgd_dummy_dem->p_heightData_m = dummyArray;
    prgd_dummy_dem->cols = vvd_elevation_data[0].size();
    prgd_dummy_dem->rows = vvd_elevation_data.size();
    prgd_dummy_dem->nodeSize_m = res;
    for (uint j = 0; j < vvd_elevation_data.size(); j++)
    {
        for (uint i = 0; i < vvd_elevation_data[0].size(); i++)
        {
            prgd_dummy_dem->p_heightData_m[i + j * vvd_elevation_data[0].size()]
                = vvd_elevation_data[j][i];
        }
    }

    base::Waypoint samplePos;
    ASSERT_NO_THROW(samplePos = getWaypoint("test/unit/data/input/MMMapTest/sample_pos.txt")) << "Input Waypoint file is missing";

    // Error with resolution
    std::cout
        << "\033[32m[----------]\033[0m [INFO] Testing exception with resolution value"
        << std::endl;
    prgd_dummy_dem->nodeSize_m = 0;

    ASSERT_THROW(MobileManipMap dummyMap1((*prgd_dummy_dem)),
                 std::exception);
    prgd_dummy_dem->nodeSize_m = -0.1;
    ASSERT_THROW(MobileManipMap dummyMap2((*prgd_dummy_dem)),
                 std::exception);
    prgd_dummy_dem->nodeSize_m = res;
    // Error with number of cols
    std::cout << "\033[32m[----------]\033[0m [INFO] Testing exception with number of "
                 "columns"
              << std::endl;
    prgd_dummy_dem->cols = 0;
    ASSERT_THROW(MobileManipMap dummyMap3((*prgd_dummy_dem)),
                 std::exception);
    prgd_dummy_dem->cols = vvd_elevation_data[0].size();
    // Error with number of rows
    std::cout
        << "\033[32m[----------]\033[0m [INFO] Testing exception with number of rows"
        << std::endl;
    prgd_dummy_dem->rows = 0;
    ASSERT_THROW(MobileManipMap dummyMap4((*prgd_dummy_dem)),
                 std::exception);
    prgd_dummy_dem->rows = vvd_elevation_data.size();
}*/

TEST(MMMapTest, sample_pos_error_test)
{
    // Input Elevation Matrix is read
    std::vector<std::vector<double>> vvd_elevation_data, vvd_validity_data;
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMapTest/RG_Colmenar_10cmDEM.csv",
                   vvd_elevation_data));
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMapTest/RG_Colmenar_10cmValidity.csv",
                   vvd_validity_data)) << "Input DEM file is missing";
    double res = 0.1; // meters

    // A dummy Rover Guidance based DEM is created
    RoverGuidance_Dem *prgd_dummy_dem = new RoverGuidance_Dem;
    double dummyArray[vvd_elevation_data.size() * vvd_elevation_data[0].size()];
    int8_t validityArray[vvd_validity_data.size() * vvd_validity_data[0].size()];
    prgd_dummy_dem->p_heightData_m = dummyArray;
    prgd_dummy_dem->p_pointValidityFlag = validityArray;
    prgd_dummy_dem->cols = vvd_elevation_data[0].size();
    prgd_dummy_dem->rows = vvd_elevation_data.size();
    prgd_dummy_dem->nodeSize_m = res;
    for (uint j = 0; j < vvd_elevation_data.size(); j++)
    {
        for (uint i = 0; i < vvd_elevation_data[0].size(); i++)
        {
            prgd_dummy_dem->p_heightData_m[i + j * vvd_elevation_data[0].size()]
                = vvd_elevation_data[j][i];
            prgd_dummy_dem->p_pointValidityFlag[i + j * vvd_elevation_data[0].size()] = (uint8_t)vvd_validity_data[j][i];
        }
    }

    base::Waypoint w_sample_one, w_sample_two, w_sample_three, w_sample_four;
    w_sample_one.position[0] = -1.0;
    w_sample_one.position[1] = 5.6;
    w_sample_two.position[0] = 100;
    w_sample_two.position[1] = 5.6;
    w_sample_three.position[0] = 5.3;
    w_sample_three.position[1] = -1.0;
    w_sample_four.position[0] = 5.3;
    w_sample_four.position[1] = 100;
    
    MobileManipMap dummyMap;
    dummyMap.loadDEM((*prgd_dummy_dem));
    unsigned int ui_error_code = 0;
    // Error with the sample waypoints
    std::cout << "\033[32m[----------]\033[0m [INFO] Testing error with waypoint "
                 "out of range - First Sample"
              << std::endl;
    ASSERT_EQ(dummyMap.computeFACE(w_sample_one, 1.0, 0.94, 1.54),
                 2);
    std::cout << "\033[32m[----------]\033[0m [INFO] Testing error with waypoint "
                 "out of range - Second Sample"
              << std::endl;
    ASSERT_EQ(dummyMap.computeFACE(w_sample_two, 1.0, 0.94, 1.54),
                 2);
    std::cout << "\033[32m[----------]\033[0m [INFO] Testing error with waypoint "
                 "out of range - Third Sample"
              << std::endl;
    ASSERT_EQ(dummyMap.computeFACE(w_sample_three, 1.0, 0.94, 1.54),
                 2);
    std::cout << "\033[32m[----------]\033[0m [INFO] Testing error with waypoint "
                 "out of range - Fourth Sample"
              << std::endl;
    ASSERT_EQ(dummyMap.computeFACE(w_sample_four, 1.0, 0.94, 1.54),
                 2);
     
}
