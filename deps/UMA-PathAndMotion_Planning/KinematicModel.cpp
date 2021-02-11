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


#include "KinematicModel.h"
#include <iostream>
#include <math.h>
#define pi 3.14159265359

using namespace KinematicModel_lib;

std::vector<std::vector<double>> KinematicModel_lib::dot(
    std::vector<std::vector<double>> A,
    std::vector<std::vector<double>> B)
{
    // This function performs matrixes product
    int n = A.size();
    int m = B[0].size();
    std::vector<std::vector<double>> C(n, std::vector<double>(m));

    for (int i = 0; i < n; i++)
        for (int j = 0; j < m; j++)
        {
            C[i][j] = 0;
            for (int k = 0; k < B.size(); k++)
                C[i][j] += A[i][k] * B[k][j];
        }

    return C;
}

std::vector<double> KinematicModel_lib::dot(std::vector<std::vector<double>> A,
                                            std::vector<double> b)
{
    // This function performs matrix and vector product
    int n = A.size();
    int m = b.size();
    std::vector<double> c(n, 0);

    for (int i = 0; i < n; i++)
        for (int j = 0; j < m; j++)
        {
            c[i] += A[i][j] * b[j];
        }

    return c;
}

std::vector<double> KinematicModel_lib::dot(double n, std::vector<double> a)
{
    // This function performs scalar product
    int m = a.size();
    std::vector<double> c(m);

    for (int i = 0; i < m; i++)
        c[i] = n * a[i];

    return c;
}

std::vector<std::vector<double>> KinematicModel_lib::getTraslation(
    std::vector<double> position)
{
    std::vector<std::vector<double>> T{{1, 0, 0, position[0]},
                                       {0, 1, 0, position[1]},
                                       {0, 0, 1, position[2]},
                                       {0, 0, 0, 1}};
    return T;
}

std::vector<std::vector<double>> KinematicModel_lib::getXrot(double angle)
{
    double s = sin(angle);
    double c = cos(angle);

    if (abs(s) < 0.000000001) s = 0;
    if (abs(c) < 0.000000001) c = 0;

    std::vector<std::vector<double>> T{
        {1, 0, 0, 0}, {0, c, -s, 0}, {0, s, c, 0}, {0, 0, 0, 1}};
    return T;
}

std::vector<std::vector<double>> KinematicModel_lib::getYrot(double angle)
{
    double s = sin(angle);
    double c = cos(angle);

    if (abs(s) < 0.000000001) s = 0;
    if (abs(c) < 0.000000001) c = 0;

    std::vector<std::vector<double>> T{
        {c, 0, s, 0}, {0, 1, 0, 0}, {-s, 0, c, 0}, {0, 0, 0, 1}};
    return T;
}

std::vector<std::vector<double>> KinematicModel_lib::getZrot(double angle)
{
    double s = sin(angle);
    double c = cos(angle);

    if (abs(s) < 0.000000001) s = 0;
    if (abs(c) < 0.000000001) c = 0;

    std::vector<std::vector<double>> T{
        {c, -s, 0, 0}, {s, c, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    return T;
}

double KinematicModel_lib::getDeterminant(
    const std::vector<std::vector<double>> *A)
{
    int n = A->size();

    if (n == 2)
    {
        return (*A)[0][0] * (*A)[1][1] - (*A)[1][0] * (*A)[0][1];
    }
    else
    {
        double d;
        int c, i, j, subi, subj;
        std::vector<std::vector<double>> subA;

        for (c = 0; c < n; c++)
        {
            subA = getCofactor(A, 0, c);
            d = d + (pow(-1, c) * (*A)[0][c] * getDeterminant(&subA));
        }
        if (abs(d) < 0.0000001) d = 0;
        return d;
    }
}

std::vector<std::vector<double>> KinematicModel_lib::getCofactor(
    const std::vector<std::vector<double>> *A,
    int row,
    int col)
{
    int n = A->size();
    int subi = 0, subj = 0;
    std::vector<std::vector<double>> subA(n - 1, std::vector<double>(n - 1));
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (i != row && j != col)
            {
                subA[subi][subj] = (*A)[i][j];
                subj++;
                if (subj == n - 1)
                {
                    subj = 0;
                    subi++;
                }
            }
        }
    }

    return subA;
}

std::vector<std::vector<double>> KinematicModel_lib::getAdjoint(
    const std::vector<std::vector<double>> *A)
{
    int n = A->size();

    if (n == 1)
        return std::vector<std::vector<double>>(1, std::vector<double>(1, 1));

    int sign = 1;
    std::vector<std::vector<double>> adj(n, std::vector<double>(n));

    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
        {
            sign = ((i + j) % 2 == 0) ? 1 : -1;
            std::vector<std::vector<double>> cof = getCofactor(A, i, j);
            adj[j][i] = sign * getDeterminant(&cof);
        }

    return adj;
}

std::vector<std::vector<double>> KinematicModel_lib::getInverse(
    const std::vector<std::vector<double>> *A)
{
    double det = getDeterminant(A);
    if (det == 0)
    {
        std::cout
            << "\033[1;31mERROR [KinematicModel_lib::getInverse]: Singular "
               "matrix, can't find its inverse\033[0m\n";
        return std::vector<std::vector<double>>(1, std::vector<double>(1, 0));
    }

    std::vector<std::vector<double>> adj = getAdjoint(A);

    int n = A->size();
    std::vector<std::vector<double>> inverse(n, std::vector<double>(n));

    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            inverse[i][j] = adj[i][j] / det;

    return inverse;
}

std::vector<double> KinematicModel_lib::getCrossProduct(std::vector<double> a,
                                                        std::vector<double> b)
{
    std::vector<double> c(3);

    c[0] = a[1] * b[2] - b[1] * a[2];
    c[1] = b[0] * a[2] - a[0] * b[2];
    c[2] = a[0] * b[1] - b[0] * a[1];

    return c;
}

std::vector<double> KinematicModel_lib::getSum(std::vector<double> a,
                                               std::vector<double> b)
{
    std::vector<double> c;
    if (a.size() != b.size())
    {
        std::cout << "\033[1;31mERROR [KinematicModel_lib::getSum]: Vector "
                     "sizes don't match\033[0m\n";
        return std::vector<double>(1, 0);
    }

    for (int i = 0; i < a.size(); i++)
    {
        c.push_back(a[i] + b[i]);
    }

    return c;
}

std::vector<double> KinematicModel_lib::getDifference(std::vector<double> a,
                                                      std::vector<double> b)
{
    std::vector<double> c;
    if (a.size() != b.size())
    {
        std::cout << "\033[1;31mERROR [KinematicModel_lib::getDifference]: "
                     "Vector sizes don't match\033[0m\n";
        return std::vector<double>(1, 0);
    }

    for (int i = 0; i < a.size(); i++)
    {
        c.push_back(a[i] - b[i]);
    }

    return c;
}

double KinematicModel_lib::getNorm(std::vector<double> a)
{
    double sum = 0;

    for (int i = 0; i < a.size(); i++)
        sum += pow(a[i], 2);

    return sqrt(sum);
}

Manipulator::Manipulator(std::string _s_data_path_m)
{
    s_data_path_m = _s_data_path_m;
    reachabilityMap_Atomic = new std::vector<std::vector<std::vector<double>>>;
    reachabilityMap_Coupled = new std::vector<std::vector<std::vector<double>>>;
    reachabilityDistances_Atomic = new std::vector<std::vector<std::vector<double>>>;
    reachabilityDistances_Coupled = new std::vector<std::vector<std::vector<double>>>;
    resolutions = new std::vector<double>;
    minValues = new std::vector<double>;
    maxValues = new std::vector<double>;

    readReachabilityMap(s_data_path_m + "/reachabilityMap_Coupled.txt",
                        reachabilityMap_Coupled,
                        resolutions,
                        minValues);
    readReachabilityMap(s_data_path_m + "/reachabilityMap_Atomic.txt",
                        reachabilityMap_Atomic,
                        resolutions,
                        minValues);
    readReachabilityMap(s_data_path_m + "/reachabilityDistances_Coupled.txt",
                        reachabilityDistances_Coupled,
                        resolutions,
                        minValues);
    readReachabilityMap(s_data_path_m + "/reachabilityDistances_Atomic.txt",
                        reachabilityDistances_Atomic,
                        resolutions,
                        minValues);

    maxValues->resize(3);
    (*maxValues)[0]
        = (*minValues)[0] + reachabilityMap_Atomic->size() * (*resolutions)[0];
    (*maxValues)[1]
        = (*minValues)[1] + (*reachabilityMap_Atomic)[0].size() * (*resolutions)[1];
    (*maxValues)[2]
        = (*minValues)[2] + (*reachabilityMap_Atomic)[0][0].size() * (*resolutions)[2];
}

Manipulator::~Manipulator()
{
    ;
}

std::vector<std::vector<double>> Manipulator::getEETransform(
    std::vector<double> manipulatorJoints)
{
    // This function uses the direct kinematics model to compute the position
    // and orientation of the end effector given a certain configuration of the
    // manipulator

    double theta1 = manipulatorJoints[0];
    double theta2 = manipulatorJoints[1];
    double theta3 = manipulatorJoints[2] + pi / 2;
    double theta4 = manipulatorJoints[3];
    double theta5 = manipulatorJoints[4];
    double theta6 = manipulatorJoints[5];

    std::vector<double> dB0{0, 0, d0};
    std::vector<std::vector<double>> TB0 = getTraslation(dB0);

    std::vector<double> a01{a1, 0, 0};
    std::vector<std::vector<double>> T01
        = dot(getZrot(theta1), dot(getTraslation(a01), getXrot(-pi / 2)));

    std::vector<double> a12{a2, c2, 0};
    std::vector<std::vector<double>> T12
        = dot(getZrot(theta2), getTraslation(a12));

    std::vector<double> a23{-a3, 0, 0};
    std::vector<std::vector<double>> T23
        = dot(getZrot(theta3), dot(getTraslation(a23), getXrot(pi / 2)));

    std::vector<double> d34{0, 0, d4};
    std::vector<std::vector<double>> T34
        = dot(getTraslation(d34), dot(getZrot(theta4), getXrot(-pi / 2)));

    std::vector<std::vector<double>> T45
        = dot(getZrot(theta5), getXrot(pi / 2));

    std::vector<double> d56{0, 0, d6};
    std::vector<std::vector<double>> T56
        = dot(getTraslation(d56), getZrot(theta6));

    std::vector<std::vector<double>> TB6
        = dot(TB0, dot(T01, dot(T12, dot(T23, dot(T34, dot(T45, T56))))));
    return TB6;
}

std::vector<std::vector<double>> Manipulator::getWristTransform(
    std::vector<double> manipulatorJoints)
{
    // This function uses the direct kinematics model to compute the position
    // and orientation of the wrist given a certain configuration of the
    // manipulator

    double theta1 = manipulatorJoints[0];
    double theta2 = manipulatorJoints[1];
    double theta3 = manipulatorJoints[2] + pi / 2;

    std::vector<double> dB0{0, 0, d0};
    std::vector<std::vector<double>> TB0 = getTraslation(dB0);

    std::vector<double> a01{a1, 0, 0};
    std::vector<std::vector<double>> T01
        = dot(getZrot(theta1), dot(getTraslation(a01), getXrot(-pi / 2)));

    std::vector<double> a12{a2, c2, 0};
    std::vector<std::vector<double>> T12
        = dot(getZrot(theta2), getTraslation(a12));

    std::vector<double> a23{-a3, 0, 0};
    std::vector<std::vector<double>> T23
        = dot(getZrot(theta3), dot(getTraslation(a23), getXrot(pi / 2)));

    std::vector<double> d34{0, 0, d4};
    std::vector<std::vector<double>> T34 = getTraslation(d34);

    std::vector<std::vector<double>> TB4
        = dot(TB0, dot(T01, dot(T12, dot(T23, T34))));
    return TB4;
}

std::vector<double> Manipulator::getManipJoints(std::vector<double> position,
                                                std::vector<double> orientation,
                                                int shoulder = 1,
                                                int elbow = 1)
{
    // This function uses a geometric Inverse Kinematics Model to obtain the
    // needed configuration of the arm to reach a certain cartesian position and
    // orientation.

    std::vector<double> q(6, 0);

    double x = position[0];
    double y = position[1];
    double z = position[2];

    double roll = orientation[0];
    double pitch = orientation[1];
    double yaw = orientation[2];

    if (abs(shoulder) != 1)
    {
        std::cout
            << "\033[1;31mWARNING [Manipulator::getManipJoints]: shoulder "
               "value has to be 1 or -1, shoulder "
               "will be considered as 1\033[0m\n";
        shoulder = 1;
    }

    if (abs(elbow) != 1)
    {
        std::cout
            << "\033[1;31mWARNING [Manipulator::getManipJoints]: elbow value "
               "has to be 1 or -1, it will be "
               "considered as 1\033[0m\n";
        elbow = 1;
    }

    std::vector<std::vector<double>> Torientation
        = dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll)));

    double xm = x - d6 * Torientation[0][2];
    double ym = y - d6 * Torientation[1][2];
    double zm = z - d6 * Torientation[2][2];

    q[0] = atan2(ym, xm) - pi / 2 + shoulder * pi / 2;

    double r = sqrt(pow(xm, 2) + pow(ym, 2)) - shoulder * a1;

    double alpha = atan2(zm - d0, r);
    double d = sqrt(pow(zm - d0, 2) + pow(r, 2));
    double l1 = sqrt(pow(c2, 2) + pow(a2, 2));
    double l2 = sqrt(pow(a3, 2) + pow(d4, 2));

    if (d > l1 + l2)
    {
        /*std::cout << "\033[1;31mERROR [Manipulator::getManipJoints]: Wrist "
                     "position is too far, unreachable position "
                     "and orientation\033[0m\n";*/
        throw std::exception();
        // return std::vector<double>(1, 0);
    }
    else if (d < l1 - l2)
    {
        /*std::cout << "\033[1;31mERROR [Manipulator::getManipJoints]: Wrist "
                     "position is too close, unreachable position "
                     "and orientation\033[0m\n";*/
        throw std::exception();
        // return std::vector<double>(1, 0);
    }

    double beta = acos((pow(d, 2) + pow(l1, 2) - pow(l2, 2)) / (2 * d * l1));
    double gamma = acos((pow(l1, 2) + pow(l2, 2) - pow(d, 2)) / (2 * l1 * l2));

    double theta2ini = atan2(c2, a2);
    double theta3ini = atan2(a3, d4);

    q[1] = 3 * pi / 2 + shoulder * pi / 2 - shoulder * alpha - elbow * beta
           - theta2ini;
    q[2] = pi - elbow * gamma + (theta2ini + theta3ini);

    std::vector<double> a01{a1, 0, 0};
    std::vector<std::vector<double>> T01
        = dot(getZrot(q[0]), dot(getTraslation(a01), getXrot(-pi / 2)));

    std::vector<double> a12{a2, c2, 0};
    std::vector<std::vector<double>> T12
        = dot(getZrot(q[1]), getTraslation(a12));

    std::vector<double> a23{0, -a3, 0};
    std::vector<std::vector<double>> T23
        = dot(getZrot(q[2]),
              dot(getTraslation(a23), dot(getXrot(pi / 2), getYrot(pi / 2))));

    std::vector<std::vector<double>> T03 = dot(T01, dot(T12, T23));

    std::vector<std::vector<double>> T36 = dot(getInverse(&T03), Torientation);

    double c5 = T36[2][2];
    double s5 = sqrt(pow(T36[0][2], 2) + pow(T36[1][2], 2));

    if (abs(s5) >= 1e-4)
    {
        double c4 = T36[0][2] / s5;
        double s4 = T36[1][2] / s5;

        double c6 = -T36[2][0] / s5;
        double s6 = T36[2][1] / s5;

        q[3] = atan2(s4, c4);
        q[4] = atan2(s5, c5);
        q[5] = atan2(s6, c6);
    }
    else
    {
        if (c5 > 0)
        {
            double s46 = T36[1][0];
            double c46 = T36[1][1];

            q[3] = 0;
            q[4] = atan2(s5, c5);
            q[5] = atan2(s46, c46);
        }
        else
        {
            double s64 = T36[1][0];
            double c64 = T36[1][1];

            q[3] = 0;
            q[4] = atan2(s5, c5);
            q[5] = atan2(s64, c64);
        }
    }

    for (int i = 0; i < 6; i++)
    {
        if (q[i] > pi) q[i] -= 2 * pi;
        if (q[i] < -pi) q[i] += 2 * pi;
        if (abs(q[i]) < 1e-4) q[i] = 0;
    }

    return q;
}

std::vector<double> Manipulator::getPositionJoints(std::vector<double> position,
                                                   int shoulder = 1,
                                                   int elbow = 1,
						   double d_error_margin)
{
    // This function uses a geometric Inverse Kinematics Model to obtain the
    // needed configuration of the arm to reach a certain cartesian position and
    // orientation.

    std::vector<double> q(3, 0);

    double xm = position[0];
    double ym = position[1];
    double zm = position[2];

    if (abs(shoulder) != 1)
    {
        std::cout
            << "\033[1;31mWARNING [Manipulator::getManipJoints]: shoulder "
               "value has to be 1 or -1, shoulder "
               "will be considered as 1\033[0m\n";
        shoulder = 1;
    }

    if (abs(elbow) != 1)
    {
        std::cout
            << "\033[1;31mWARNING [Manipulator::getManipJoints]: elbow value "
               "has to be 1 or -1, it will be "
               "considered as 1\033[0m\n";
        elbow = 1;
    }

    q[0] = atan2(ym, xm) - pi / 2 + shoulder * pi / 2;

    double r = sqrt(pow(xm, 2) + pow(ym, 2)) - shoulder * a1;

    double alpha = atan2(zm - d0, r);
    double d = sqrt(pow(zm - d0, 2) + pow(r, 2));
    double l1 = sqrt(pow(c2, 2) + pow(a2, 2));
    double l2 = sqrt(pow(a3, 2) + pow(d4, 2));

    // d_error_margin is introduced because although two near nodes can be valid, a
    // convex curved path passing through them may touch the forbidden volume and produce later
    // an exception

    if (d + d_error_margin > l1 + l2)
    {
        /*std::cout << "\033[1;31mERROR [Manipulator::getManipJoints]: Wrist "
                     "position is too far, unreachable position "
                     "and orientation\033[0m\n";
	std::cout << "d = " << d << std::endl;
	std::cout << "l1 = " << l1 << std::endl;
	std::cout << "l2 = " << l2 << std::endl;*/
        throw std::exception();
        // return std::vector<double>(1, 0);
    }
    else if (d - d_error_margin < l1 - l2)
    {
        /*std::cout << "\033[1;31mERROR [Manipulator::getManipJoints]: Wrist "
                     "position is too close, unreachable position "
                     "and orientation\033[0m\n";
       	std::cout << "d = " << d << std::endl;
	std::cout << "l1 = " << l1 << std::endl;
	std::cout << "l2 = " << l2 << std::endl;*/
        throw std::exception();
        // return std::vector<double>(1, 0);
    }

    double beta = acos((pow(d, 2) + pow(l1, 2) - pow(l2, 2)) / (2 * d * l1));
    double gamma = acos((pow(l1, 2) + pow(l2, 2) - pow(d, 2)) / (2 * l1 * l2));

    double theta2ini = atan2(c2, a2);
    double theta3ini = atan2(a3, d4);

    q[1] = 3 * pi / 2 + shoulder * pi / 2 - shoulder * alpha - elbow * beta
           - theta2ini;
    q[2] = pi - elbow * gamma + (theta2ini + theta3ini);

    for (int i = 0; i < 3; i++)
    {
        if (q[i] > pi) q[i] -= 2 * pi;
        if (q[i] < -pi) q[i] += 2 * pi;
        if (abs(q[i]) < 1e-4) q[i] = 0;
    }

    return q;
}

std::vector<double> Manipulator::getWristJoints(
    std::vector<double> positionJoints,
    std::vector<double> orientation)
{

    std::vector<double> q(3, 0);

    double roll = orientation[0];
    double pitch = orientation[1];
    double yaw = orientation[2];

    std::vector<std::vector<double>> Torientation
        = dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll)));

    std::vector<double> a01{a1, 0, 0};
    std::vector<std::vector<double>> T01 = dot(
        getZrot(positionJoints[0]), dot(getTraslation(a01), getXrot(-pi / 2)));

    std::vector<double> a12{a2, c2, 0};
    std::vector<std::vector<double>> T12
        = dot(getZrot(positionJoints[1]), getTraslation(a12));

    std::vector<double> a23{0, -a3, 0};
    std::vector<std::vector<double>> T23
        = dot(getZrot(positionJoints[2]),
              dot(getTraslation(a23), dot(getXrot(pi / 2), getYrot(pi / 2))));

    std::vector<std::vector<double>> T03 = dot(T01, dot(T12, T23));

    std::vector<std::vector<double>> T36 = dot(getInverse(&T03), Torientation);

    double c5 = T36[2][2];
    double s5 = sqrt(pow(T36[0][2], 2) + pow(T36[1][2], 2));

    if (abs(s5) >= 1e-4)
    {
        double c4 = T36[0][2] / s5;
        double s4 = T36[1][2] / s5;

        double c6 = -T36[2][0] / s5;
        double s6 = T36[2][1] / s5;

        q[0] = atan2(s4, c4);
        q[1] = atan2(s5, c5);
        q[2] = atan2(s6, c6);
    }
    else
    {
        if (c5 > 0)
        {
            double s46 = T36[1][0];
            double c46 = T36[1][1];

            q[0] = 0;
            q[1] = atan2(s5, c5);
            q[2] = atan2(s46, c46);
        }
        else
        {
            double s64 = T36[1][0];
            double c64 = T36[1][1];

            q[0] = 0;
            q[1] = atan2(s5, c5);
            q[2] = atan2(s64, c64);
        }
    }

    for (int i = 0; i < 3; i++)
    {
        if (q[i] > pi) q[i] -= 2 * pi;
        if (q[i] < -pi) q[i] += 2 * pi;
        if (abs(q[i]) < 1e-4) q[i] = 0;
    }

    return q;
}

std::vector<double> Manipulator::getManipJoints(
    std::vector<double> position,
    std::vector<double> orientation,
    std::vector<double> previousConfig)
{
    // This function uses a Closed Loop Inverse Kinematics Model based on
    // quaternions to obtain the closer configuration of the arm to reach a
    // certain cartesian position and orientation, given the previous one.
    clock_t init = clock();
    std::vector<double> q = previousConfig;

    double x = position[0];
    double y = position[1];
    double z = position[2];

    double roll = orientation[0];
    double pitch = orientation[1];
    double yaw = orientation[2];

    std::vector<double> Pd{x, y, z};
    std::vector<std::vector<double>> Rd
        = dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll)));

    double nd;
    std::vector<double> ed(3);

    nd = sqrt(Rd[0][0] + Rd[1][1] + Rd[2][2] + 1) / 2;

    int s0 = 1;
    if ((Rd[2][1] - Rd[1][2]) < 0) s0 = -1;
    ed[0] = s0 * sqrt(Rd[0][0] - Rd[1][1] - Rd[2][2] + 1) / 2;

    int s1 = 1;
    if ((Rd[0][2] - Rd[2][0]) < 0) s1 = -1;
    ed[1] = s1 * sqrt(Rd[1][1] - Rd[2][2] - Rd[0][0] + 1) / 2;

    int s2 = 1;
    if ((Rd[1][0] - Rd[0][1]) < 0) s2 = -1;
    ed[2] = s2 * sqrt(Rd[2][2] - Rd[0][0] - Rd[1][1] + 1) / 2;

    /************** Closed loop ************/
    // Initialize variables
    double n;
    std::vector<double> e(3);
    std::vector<double> qp{0, 0, 0, 0, 1, 0};
    std::vector<double> ep(3, 999);
    std::vector<double> eo(3, 999);
    std::vector<double> up;
    std::vector<double> uo;
    double Kp = 1;
    double Ko = 1;

    int it = 0;
    while ((getNorm(ep) > 1e-6) || (getNorm(eo) > 1e-6))
    {
        clock_t init2 = clock();
        std::vector<std::vector<double>> R = getEETransform(q);

        std::vector<double> P{R[0][3], R[1][3], R[2][3]};

        double n;
        std::vector<double> e(3);

        n = sqrt(R[0][0] + R[1][1] + R[2][2] + 1) / 2;

        s0 = 1;
        if ((R[2][1] - R[1][2]) < 0) s0 = -1;
        e[0] = s0 * sqrt(R[0][0] - R[1][1] - R[2][2] + 1) / 2;

        s1 = 1;
        if ((R[0][2] - R[2][0]) < 0) s1 = -1;
        e[1] = s1 * sqrt(R[1][1] - R[2][2] - R[0][0] + 1) / 2;

        s2 = 1;
        if ((R[1][0] - R[0][1]) < 0) s2 = -1;
        e[2] = s2 * sqrt(R[2][2] - R[0][0] - R[1][1] + 1) / 2;

        clock_t endt2 = clock();
        double eetime2 = double(endt2 - init2) / CLOCKS_PER_SEC;
        // std::cout << "Quaternion computation time: " << eetime2 << std::endl;

        ep = getDifference(Pd, P);
        eo = getDifference(dot(n, ed),
                           getSum(dot(nd, e), getCrossProduct(ed, e)));

        for (int i = 0; i < 3; i++)
        {
            if (abs(ep[i]) < 1e-6) ep[i] = 0;
            if (abs(eo[i]) < 1e-6) eo[i] = 0;
        }

        clock_t endt3 = clock();
        double eetime3 = double(endt3 - endt2) / CLOCKS_PER_SEC;
        // std::cout << "Error computation time: " << eetime3 << std::endl;

        up = dot(Kp, ep);
        uo = dot(Ko, eo);

        clock_t endt4 = clock();
        double eetime4 = double(endt4 - endt3) / CLOCKS_PER_SEC;
        // std::cout << "Actuation time: " << eetime4 << std::endl;

        std::vector<std::vector<double>> J = getJacobianMatrix(q);

        clock_t endt5 = clock();
        double eetime5 = double(endt5 - endt4) / CLOCKS_PER_SEC;
        // std::cout << "Jacobian time: " << eetime5 << std::endl;

        clock_t endt6 = clock();
        if (getDeterminant(&J) != 0)
        {
            std::vector<std::vector<double>> iJ = getInverse(&J);

            endt6 = clock();
            double eetime6 = double(endt6 - endt5) / CLOCKS_PER_SEC;
            // std::cout << "Inverse time: " << eetime6 << std::endl;

            std::vector<double> u{up[0], up[1], up[2], uo[0], uo[1], uo[2]};
            qp = dot(iJ, u);
        }
        else
            std::cout << "\033[1;33mWARNING [KinematicModel_lib::CLIK]: "
                         "singularity encountered\033[0m\n";

        std::vector<double> sum;
        q = getSum(q, qp);

        it += 1;
        clock_t endt7 = clock();
        double eetime7 = double(endt7 - endt6) / CLOCKS_PER_SEC;
        // std::cout << "Velocities and new position time: " << eetime7
        //          << std::endl;
    }

    for (int i = 0; i < 6; i++)
    {
        while (q[i] > pi)
            q[i] = q[i] - 2 * pi;
        while (q[i] < -pi)
            q[i] = q[i] + 2 * pi;
        if (abs(q[i]) < 1e-6) q[i] = 0;
    }

    clock_t endt = clock();
    double eetime = double(endt - init) / CLOCKS_PER_SEC;
    std::cout << "CLIK finished in " << it
              << " iterations, elapsed exec time: " << eetime << std::endl;

    return q;
}

std::vector<std::vector<double>> Manipulator::getJacobianMatrix(
    std::vector<double> manipulatorJoints)
{
    double theta1 = manipulatorJoints[0];
    double theta2 = manipulatorJoints[1];
    double theta3 = manipulatorJoints[2] + pi / 2;
    double theta4 = manipulatorJoints[3];
    double theta5 = manipulatorJoints[4];
    double theta6 = manipulatorJoints[5];

    std::vector<double> dB0{0, 0, d0};
    std::vector<std::vector<double>> TB0 = getTraslation(dB0);

    std::vector<double> a01{a1, 0, 0};
    std::vector<std::vector<double>> T01
        = dot(getZrot(theta1), dot(getTraslation(a01), getXrot(-pi / 2)));

    std::vector<double> a12{a2, c2, 0};
    std::vector<std::vector<double>> T12
        = dot(getZrot(theta2), getTraslation(a12));

    std::vector<double> a23{-a3, 0, 0};
    std::vector<std::vector<double>> T23
        = dot(getZrot(theta3), dot(getTraslation(a23), getXrot(pi / 2)));

    std::vector<double> d34{0, 0, d4};
    std::vector<std::vector<double>> T34
        = dot(getTraslation(d34), dot(getZrot(theta4), getXrot(-pi / 2)));

    std::vector<std::vector<double>> T45
        = dot(getZrot(theta5), getXrot(pi / 2));

    std::vector<double> d56{0, 0, d6};
    std::vector<std::vector<double>> T56
        = dot(getTraslation(d56), getZrot(theta6));

    std::vector<std::vector<double>> TB1 = dot(TB0, T01);
    std::vector<std::vector<double>> TB2 = dot(TB1, T12);
    std::vector<std::vector<double>> TB3 = dot(TB2, T23);
    std::vector<std::vector<double>> TB4 = dot(TB3, T34);
    std::vector<std::vector<double>> TB5 = dot(TB4, T45);
    std::vector<std::vector<double>> TB6 = dot(TB5, T56);

    for (int i = 0; i < TB1.size(); i++)
        for (int j = 0; j < TB1[0].size(); j++)
        {
            if (abs(TB1[i][j]) < 1e-6) TB1[i][j] = 0;
            if (abs(TB2[i][j]) < 1e-6) TB2[i][j] = 0;
            if (abs(TB3[i][j]) < 1e-6) TB3[i][j] = 0;
            if (abs(TB4[i][j]) < 1e-6) TB4[i][j] = 0;
            if (abs(TB5[i][j]) < 1e-6) TB5[i][j] = 0;
            if (abs(TB6[i][j]) < 1e-6) TB6[i][j] = 0;
        }

    std::vector<std::vector<double>> z;
    z.push_back(std::vector<double>{TB0[0][2], TB0[1][2], TB0[2][2]});
    z.push_back(std::vector<double>{TB1[0][2], TB1[1][2], TB1[2][2]});
    z.push_back(std::vector<double>{TB2[0][2], TB2[1][2], TB2[2][2]});
    z.push_back(std::vector<double>{TB3[0][2], TB3[1][2], TB3[2][2]});
    z.push_back(std::vector<double>{TB4[0][2], TB4[1][2], TB4[2][2]});
    z.push_back(std::vector<double>{TB5[0][2], TB5[1][2], TB5[2][2]});
    z.push_back(std::vector<double>{TB6[0][2], TB6[1][2], TB6[2][2]});

    std::vector<std::vector<double>> p;
    p.push_back(std::vector<double>{TB0[0][3], TB0[1][3], TB0[2][3]});
    p.push_back(std::vector<double>{TB1[0][3], TB1[1][3], TB1[2][3]});
    p.push_back(std::vector<double>{TB2[0][3], TB2[1][3], TB2[2][3]});
    p.push_back(std::vector<double>{TB3[0][3], TB3[1][3], TB3[2][3]});
    p.push_back(std::vector<double>{TB4[0][3], TB4[1][3], TB4[2][3]});
    p.push_back(std::vector<double>{TB5[0][3], TB5[1][3], TB5[2][3]});
    p.push_back(std::vector<double>{TB6[0][3], TB6[1][3], TB6[2][3]});

    std::vector<std::vector<double>> Jp, Jo;
    for (int i = 1; i < 7; i++)
    {
        Jp.push_back(getCrossProduct(z[i - 1], getDifference(p[6], p[i - 1])));
        Jo.push_back(z[i - 1]);
    }

    std::vector<std::vector<double>> J(6, std::vector<double>(6));
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
        {
            if (j < 3)
                J[j][i] = Jp[i][j];
            else
                J[j][i] = Jo[i][j - 3];

            if (abs(J[j][i]) < 1e-6) J[j][i] = 0;
        }

    return J;
}

bool Manipulator::isFarFromMast(double joint0, double joint1, double d_z)
{
    joint0 = abs(joint0);

   /* if ((joint0 < 0.2)||(joint0 > 3.0))
    {
        return false;
    }
    if ((joint1 < -0.7)||(joint1 > -0.3))
    {
        return false;
    }
    if ((joint2 < -1.9)||(joint2 > 0.0))
    {
        return false;
    }
    return true;*/

/*
    if (d_z < 0.5)
    {
       return false;
    }
*/
    if (joint0 < 0.0)
    {
        return false;
    }

    if ((joint0 > 1.75)&&(d_z < 0.65))
    {
        return false;
    }

/*
    if ((joint0 > 0.7)&&(d_z < 0.5))
    {
        return false;
    }
*/

    if (joint0 > 1.51)
    {
/*
        if (d_z < 0.5)
        {
            return false;
        }
*/
        return true;
    }
    else
    {
        if(joint0 < 1.0)
	{
            if ((joint1 > -0.8))
	    {
                return true;
	    }
	    else
	    {
                return false;
	    }
	}
	else
	{
/*
            if (d_z < 0.5)
            {
                return false;
            }
*/
            if ((joint1 > -joint0))
	    {
                return true;
	    } 
            else
	    {
                return false;
	    }
	}	
    }
}

bool Manipulator::isFarFromLeg(double joint0, double d_z)
{
   if ((joint0 > 0.6)&&(joint0 < 3.14)&&(d_z < 0.5))
   {
        return false;
   }
   else
   {

        return true;
   } 
}

void Manipulator::computeReachabilityMap(const double resXY, const double resZ)
{
    double res4 = 30 * M_PI / 180;
    double res5 = 20 * M_PI / 180;
    double res6 = 30 * M_PI / 180;

    double maxXY = (a1 + a2 + d4) * 1.1;
    double minXY = (-a1 - a2 - d4) * 1.1;

    double maxZ = (d0 + a2 + d4) * 1.1;
    double minZ = (d0 - a2 - d4) * 1.1;

    std::vector<double> resolutions = {resXY, resXY, resZ};
    std::vector<double> minValues = {minXY, minXY, minZ};

    double resXYZ = sqrt(2*pow(resXY,2)+pow(resZ,2));

    int sizeXY = (int)((maxXY - minXY) / resXY);
    int sizeZ = (int)((maxZ - minZ) / resZ);


    std::cout << "Size xy: " << sizeXY << ", size z: " << sizeZ << std::endl;
    std::cout << "Res xy: " << resXY << ", res z: " << resZ << ", res xyz: " << resXYZ << std::endl;
    std::cout << "Min xy: " << minXY << ", min z: " << minZ << std::endl;
    std::cout << "Max xy: " << maxXY << ", max z: " << maxZ << std::endl;


    std::vector<std::vector<std::vector<double>>> reachabilityMap(
        sizeXY,
        std::vector<std::vector<double>>(sizeXY,
                                         std::vector<double>(sizeZ, 2)));
    std::vector<double> position;
    CollisionDetector *p_collision_detector
        = new CollisionDetector(s_data_path_m);

    std::cout << "Starting reachability map computation...\n";
    for (int i = 0; i < sizeXY; i++)
        for (int j = 0; j < sizeXY; j++)
            for (int k = 0; k < sizeZ; k++)
            {
                std::cout << "\rProgress: [" << 100 * i / sizeXY << "%, "
                          << 100 * j / sizeXY << "%, " << 100 * k / sizeZ
                          << "%]";
                position
                    = {minXY + i * resXY, minXY + j * resXY, minZ + k * resZ};
                try
                {
                    std::vector<double> config
                        = getPositionJoints(position, 1, 1, resXYZ*0.2);
                    config.resize(6);

		    config[3] = 0.0; 
		    config[4] = std::max(-1.51,-config[2]); 
		    config[5] = -2.7; 

                    if ((!p_collision_detector->isWristColliding(config))&&(config[0]<3.0)&&(config[0]>-3.0)&&(config[1]>-2)&&(isFarFromMast(config[0],config[1], position[2]))) //TODO - This is a workaround to avoid passing through pi/-pi
                    {
                        /*for (int l = 0; l < 6; l++)
                        {
                            config[3] = l * res4;
                            for (int m = 0; m < 12; m++)
                            {
                                config[4] = -110 * M_PI / 180 + m * res5;
                                for (int n = 0; n < 3; n++)
                                {
                                    config[5] = n * res6;
                                    // std::cout << ". Config: ["<<config[0]<<",
                                    // "<<config[1]<<", "<<config[2]<<",
                                    // "<<config[3]<<", "<<config[4]<<",
                                    // "<<config[5]<<"]";
                                    std::cout << std::flush;

                                    if (p_collision_detector->isColliding(
                                            config))
                                    {
                                        reachabilityMap[i][j][k] = 1;
                                        break;
                                    }
                                }
                                if (reachabilityMap[i][j][k] == 1) break;
                            }
                            if (reachabilityMap[i][j][k] == 1) break;
                        }*/
                    }
                    else
                    {
                        reachabilityMap[i][j][k] = 0;
                    }
                }
                catch (std::exception &e)
                {
                    reachabilityMap[i][j][k] = 0;
                }
            }

    std::cout << "...done!" << std::flush << std::endl;
    saveVolume(&reachabilityMap,
               &resolutions,
               &minValues,
               s_data_path_m + "/reachabilityMap.txt");
}

int Manipulator::isReachable(std::vector<double> position, int mode)
{
    int ix = (int)((position[0] - (*minValues)[0]) / (*resolutions)[0] + 0.5);
    int iy = (int)((position[1] - (*minValues)[1]) / (*resolutions)[1] + 0.5);
    int iz = (int)((position[2] - (*minValues)[2]) / (*resolutions)[2] + 0.5);

    /*
    std::cout << "ix = " << ix << std::endl; 
    std::cout << "iy = " << iy << std::endl; 
    std::cout << "iz = " << iz << std::endl; 
    std::cout << "Reachability Map size = " << reachabilityMap->size() << ", " << (*reachabilityMap)[0].size() << ", " << (*reachabilityMap)[0][0].size() << std::endl; 
*/

    if (mode == 0)
    {
        if (ix > 0 && iy > 0 && iz > 0 && ix < reachabilityMap_Atomic->size() - 1
        && iy < (*reachabilityMap_Atomic)[0].size() - 1
        && iz < (*reachabilityMap_Atomic)[0][0].size() - 1)
        {
            //std::cout << "isReachable? -> " << (*reachabilityMap)[ix][iy][iz] << std::endl;
            return (*reachabilityMap_Atomic)[ix][iy][iz];
        }
        else
        {
            return 0;
        }
    }
    else
    {
        if (ix > 0 && iy > 0 && iz > 0 && ix < reachabilityMap_Coupled->size() - 1
        && iy < (*reachabilityMap_Coupled)[0].size() - 1
        && iz < (*reachabilityMap_Coupled)[0][0].size() - 1)
        {
            //std::cout << "isReachable? -> " << (*reachabilityMap)[ix][iy][iz] << std::endl;
            return (*reachabilityMap_Coupled)[ix][iy][iz];
        }
        else
        {
            return 0;
        }

    }
}
std::vector<double> Manipulator::getRelativePosition(std::vector<double> position)
{
    double posx = position[0] - (*minValues)[0];
    double posy = position[1] - (*minValues)[1];
    double posz = position[2] - (*minValues)[2];

    std::vector<double> pos = {posx, posy, posz};

    return pos;
}

std::vector<double> Manipulator::getAbsolutePosition(std::vector<double> position)
{
    double posx = position[0] + (*minValues)[0];
    double posy = position[1] + (*minValues)[1];
    double posz = position[2] + (*minValues)[2];

    std::vector<double> pos = {posx, posy, posz};

    return pos;
}

double Manipulator::getDistanceToCollision(std::vector<double> position, int mode)
{
    // mode: 0 (atomic), 1 (coupled)
    int ix = (int)((position[0] - (*minValues)[0]) / (*resolutions)[0] + 0.5);
    int iy = (int)((position[1] - (*minValues)[1]) / (*resolutions)[1] + 0.5);
    int iz = (int)((position[2] - (*minValues)[2]) / (*resolutions)[2] + 0.5);

    if (mode == 0)
    {
	if (ix > 0 && iy > 0 && iz > 0 && ix < reachabilityDistances_Atomic->size() - 1
        && iy < (*reachabilityDistances_Atomic)[0].size() - 1
        && iz < (*reachabilityDistances_Atomic)[0][0].size() - 1)
	{
	    return (*reachabilityDistances_Atomic)[ix][iy][iz];
        }
        else
        {
            return 0;
        }
    }
    else
    {
	if (ix > 0 && iy > 0 && iz > 0 && ix < reachabilityDistances_Coupled->size() - 1
        && iy < (*reachabilityDistances_Coupled)[0].size() - 1
        && iz < (*reachabilityDistances_Coupled)[0][0].size() - 1)
	{
	    return (*reachabilityDistances_Coupled)[ix][iy][iz];
        }
        else
        {
            return 0;
        }

    }
}

std::vector<double> Manipulator::getReachabilityMapSize()
{
    std::vector<double> sizes = {(*maxValues)[0]-(*minValues)[0], (*maxValues)[1]-(*minValues)[1], (*maxValues)[2]-(*minValues)[2]};
    return sizes;
}
