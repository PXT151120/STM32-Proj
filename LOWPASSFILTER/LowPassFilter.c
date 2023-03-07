
#include "LowPassFilter.h"

/*
    First Order : y[n] = a1 * y[n - 1] + b0 * x[n] + b1 * x[n - 1]

*/

double LowPassFilter(double yn_1, double xn, double xn_1, double f, double sampT)
{
    double a1 = (-1) * (sampT * Omega(f) - 2) / (sampT * Omega(f) + 2);
    double b0 = (sampT * Omega(f)) / (sampT * Omega(f) + 2);

    printf("a1 = %f\nb0 = %f", a1, b0);
    return yn_1 * a1 + b0 * xn + b0 * xn_1;
}