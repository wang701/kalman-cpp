/**
 * @file test_kf.cpp
 * @author Yang Wang
 * @date 25 Apr 2022
 * 
 * @brief Example for the Kalman filter.
 *
 */

#include <fstream>

#include "kf.h"


int main(int argc, char** argv)
  {
    /* 
     * Log the result into a tab delimitted file, later we can open 
     * it with Matlab. Use: plot_data1.m to plot the results.
     */
    ofstream log_file;
#ifdef _WIN32
    log_file.open("..\\bin\\log_file_test.txt");
#else
    log_file.open("log_file1.txt");
#endif
   
    /*
     * Define the system
     */
    mat A(1,1), B(1,1), H(1,1), Q(1,1), R(1,1);
    colvec u(1);
    colvec x0(1);
/*
    A << 0;
    B << 1;
    Q << 2*2;
    H << 1;
    R << 2*2;
    u << 12.0;
*/
    A = {0};
    B = {1};
    Q = {2*2};
    H = {1};
    R = {2*2};
    u = {12.0};
    
    KF kalman;
    kalman.InitSystem(A, B, H, Q, R);
  
    for (int k = 0; k < 100; k++) {
      kalman.Kalmanf(u);
      
      colvec *x = kalman.GetCurrentState();
      colvec *z = kalman.GetCurrentOutput();
      colvec *x_m = kalman.GetCurrentEstimatedState();
      log_file << k << '\t' << z->at(0,0) << '\t' << x->at(0,0) << '\t' << x_m->at(0,0) << '\n';      
    }
    
    log_file.close();
    
    return 0;
  }
