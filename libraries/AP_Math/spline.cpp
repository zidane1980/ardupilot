/*
This work by Ryan Muller released under the Creative Commons CC0 License
http://creativecommons.org/publicdomain/zero/1.0/
*/

/**
  Spline interpolation of a parametric function.

  INPUT: std::vector<double> x
        A list of double values that represent sampled
        points. The array index of each point will be
        taken as the parameter t by which x will be
        represented as a function.

  OUTPUT: std::vector<cv::Vec4d> P
        A list of cv::Vec4d representing polynomials. To
        interpret segment [i]:
        x(t) =  P0*a + P1*b + P2*(a^3-a)/6 + P3*(b^3-b)/6
        where a = t-i
              b = i-t+1
  */

#include "spline.h"

std::vector<std::vector<double>> splinterp(std::vector<double> x){

//    std::vector<std::vector<double>> out(5, std::vector<double>(4));
 
    // spline size
    int n=x.size();

    std::vector<std::vector<double>> out(n, std::vector<double>(4));

    // loop counter
    int i;

    // working variables
    double p;
    std::vector<double> u;

    // second derivative
    std::vector<double> z;

    u.resize(n);
    z.resize(n);

    // set the second derivative to 0 at the ends
    z[0] = u[0] = 0;
    z[n-1] = 0;

    // decomposition loop
    for(i=1;i<n-1;i++){
        p = 0.5*z[i-1] + 2.0;
        z[i] = -0.5/p;
        u[i] = x[i+1]+x[i-1]-2*x[i];
        u[i] = (3*u[i]-0.5*u[i-1])/p;
    }

    // back-substitution loop
    for(i=n-1;i>0;i--){
        z[i] = z[i] * z[i+1] + u[i];
    }

    for(i=0;i<n-1;i++){
        out[i][0] = x[i+1];
        out[i][1] = x[i];
        out[i][2] = z[i+1];
        out[i][3] = z[i];
    }

    return out;

}
