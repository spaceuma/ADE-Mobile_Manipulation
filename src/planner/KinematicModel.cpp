#include "KinematicModel.h"
#include <iostream>
#include <math.h>
#define pi 3.14159265359

using namespace KinematicModel_lib;

std::vector<std::vector<double>> KinematicModel_lib::dot(std::vector<std::vector<double>> A,
                                                         std::vector<std::vector<double>> B)
{
    // This function performs matrixes product
    int n = A.size();
    int m = B[0].size();
    std::vector<std::vector<double>> C;
    C.resize(n, std::vector<double>(m));

    for (int i = 0; i < n; i++)
        for (int j = 0; j < m; j++)
        {
            C[i][j] = 0;
            for (int k = 0; k < B.size(); k++)
                C[i][j] += A[i][k] * B[k][j];
        }

    return C;
}

std::vector<std::vector<double>> KinematicModel_lib::getTraslation(std::vector<double> position)
{
    std::vector<std::vector<double>> T{
        {1, 0, 0, position[0]}, {0, 1, 0, position[1]}, {0, 0, 1, position[2]}, {0, 0, 0, 1}};
    return T;
}

std::vector<std::vector<double>> KinematicModel_lib::getXrot(double angle)
{
    double s = sin(angle);
    double c = cos(angle);

    if (abs(s) < 0.000000001) s = 0;
    if (abs(c) < 0.000000001) c = 0;

    std::vector<std::vector<double>> T{{1, 0, 0, 0}, {0, c, -s, 0}, {0, s, c, 0}, {0, 0, 0, 1}};
    return T;
}

std::vector<std::vector<double>> KinematicModel_lib::getYrot(double angle)
{
    double s = sin(angle);
    double c = cos(angle);

    if (abs(s) < 0.000000001) s = 0;
    if (abs(c) < 0.000000001) c = 0;

    std::vector<std::vector<double>> T{{c, 0, s, 0}, {0, 1, 0, 0}, {-s, 0, c, 0}, {0, 0, 0, 1}};
    return T;
}

std::vector<std::vector<double>> KinematicModel_lib::getZrot(double angle)
{
    double s = sin(angle);
    double c = cos(angle);

    if (abs(s) < 0.000000001) s = 0;
    if (abs(c) < 0.000000001) c = 0;

    std::vector<std::vector<double>> T{{c, -s, 0, 0}, {s, c, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    return T;
}

double KinematicModel_lib::getDeterminant(std::vector<std::vector<double>> A)
{
    int n = A.size();

    if (n == 2)
    {
        return A[0][0] * A[1][1] - A[1][0] * A[0][1];
    }
    else
    {
        double d;
        int c, i, j, subi, subj;
        std::vector<std::vector<double>> subA;

        for (c = 0; c < n; c++)
        {
            subA = getCofactor(A, 0, c);
            d = d + (pow(-1, c) * A[0][c] * getDeterminant(subA));
        }
        if (abs(d) < 0.0000001) d = 0;
        return d;
    }
}

std::vector<std::vector<double>> KinematicModel_lib::getCofactor(std::vector<std::vector<double>> A, int row, int col)
{
    int n = A.size();
    int subi = 0, subj = 0;
    std::vector<std::vector<double>> subA(n - 1, std::vector<double>(n - 1));
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (i != row && j != col)
            {
                subA[subi][subj] = A[i][j];
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

std::vector<std::vector<double>> KinematicModel_lib::getAdjoint(std::vector<std::vector<double>> A)
{
    int n = A.size();

    if (n == 1) return std::vector<std::vector<double>>(1, std::vector<double>(1, 1));

    int sign = 1;
    std::vector<std::vector<double>> adj(n, std::vector<double>(n));

    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
        {
            sign = ((i + j) % 2 == 0) ? 1 : -1;
            adj[j][i] = sign * getDeterminant(getCofactor(A, i, j));
        }

    return adj;
}

std::vector<std::vector<double>> KinematicModel_lib::getInverse(std::vector<std::vector<double>> A)
{
    double det = getDeterminant(A);
    if (det == 0)
    {
        std::cout
            << "\033[1;31mERROR [KinematicModel_lib::getInverse]: Singular matrix, can't find its inverse\033[0m\n";
        return std::vector<std::vector<double>>(1, std::vector<double>(1, 0));
    }

    std::vector<std::vector<double>> adj = getAdjoint(A);

    int n = A.size();
    std::vector<std::vector<double>> inverse(n, std::vector<double>(n));

    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            inverse[i][j] = adj[i][j] / det;

    return inverse;
}

std::vector<std::vector<double>> Manipulator::getEEtransform(std::vector<double> manipulatorJoints)
{
    // This function uses the direct kinematics model to compute the position and
    // orientation of the end effector given a certain configuration of the manipulator

    double theta1 = manipulatorJoints[0];
    double theta2 = manipulatorJoints[1];
    double theta3 = manipulatorJoints[2] + pi / 2;
    double theta4 = manipulatorJoints[3];
    double theta5 = manipulatorJoints[4];
    double theta6 = manipulatorJoints[5];

    std::vector<double> dB0{0, 0, d0};
    std::vector<std::vector<double>> TB0 = getTraslation(dB0);

    std::vector<double> a01{a1, 0, 0};
    std::vector<std::vector<double>> T01 = dot(getZrot(theta1), dot(getTraslation(a01), getXrot(-pi / 2)));

    std::vector<double> a12{a2, c2, 0};
    std::vector<std::vector<double>> T12 = dot(getZrot(theta2), getTraslation(a12));

    std::vector<double> a23{-a3, 0, 0};
    std::vector<std::vector<double>> T23 = dot(getZrot(theta3), dot(getTraslation(a23), getXrot(pi / 2)));

    std::vector<double> d34{0, 0, d4};
    std::vector<std::vector<double>> T34 = dot(getTraslation(d34), dot(getZrot(theta4), getXrot(-pi / 2)));

    std::vector<std::vector<double>> T45 = dot(getZrot(theta5), getXrot(pi / 2));

    std::vector<double> d56{0, 0, d6};
    std::vector<std::vector<double>> T56 = dot(getTraslation(d56), getZrot(theta6));

    std::vector<std::vector<double>> TB6 = dot(TB0, dot(T01, dot(T12, dot(T23, dot(T34, dot(T45, T56))))));
    return TB6;
}

std::vector<double> Manipulator::getManipJoints(std::vector<double> position,
                                                std::vector<double> orientation,
                                                int shoulder = 1,
                                                int elbow = 1)
{
    std::vector<double> q(6, 0);

    double x = position[0];
    double y = position[1];
    double z = position[2];

    double roll = orientation[0];
    double pitch = orientation[1];
    double yaw = orientation[2];

    if (abs(shoulder) != 1)
    {
        std::cout << "\033[1;31mWARNING [Manipulator::getManipJoints]: shoulder value have to be 1 or -1, shoulder "
                     "will be considered as 1\033[0m\n";
        shoulder = 1;
    }

    if (abs(elbow) != 1)
    {
        std::cout << "\033[1;31mWARNING [Manipulator::getManipJoints]: elbow value have to be 1 or -1, it will be "
                     "considered as 1\033[0m\n";
        elbow = 1;
    }

    std::vector<std::vector<double>> Torientation = dot(getZrot(yaw), dot(getYrot(pitch), getXrot(roll)));

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
        std::cout << "\033[1;31mERROR [Manipulator::getManipJoints]: Wrist position is too far, unreachable position "
                     "and orientation\033[0m\n";
        return std::vector<double>(1, 0);
    }

    double beta = acos((pow(d, 2) + pow(l1, 2) - pow(l2, 2)) / (2 * d * l1));
    double gamma = acos((pow(l1, 2) + pow(l2, 2) - pow(d, 2)) / (2 * l1 * l2));

    double theta2ini = atan2(c2, a2);
    double theta3ini = atan2(a3, d4);

    q[1] = 3 * pi / 2 + shoulder * pi / 2 - shoulder * alpha - elbow * beta - theta2ini;
    q[2] = pi - elbow * gamma + (theta2ini + theta3ini);

    std::vector<double> a01{a1, 0, 0};
    std::vector<std::vector<double>> T01 = dot(getZrot(q[0]), dot(getTraslation(a01), getXrot(-pi / 2)));

    std::vector<double> a12{a2, c2, 0};
    std::vector<std::vector<double>> T12 = dot(getZrot(q[1]), getTraslation(a12));

    std::vector<double> a23{0, -a3, 0};
    std::vector<std::vector<double>> T23
        = dot(getZrot(q[2]), dot(getTraslation(a23), dot(getXrot(pi / 2), getYrot(pi / 2))));

    std::vector<std::vector<double>> T03 = dot(T01, dot(T12, T23));

    std::vector<std::vector<double>> T36 = dot(getInverse(T03), Torientation);

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
