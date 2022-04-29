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
    ofstream meas_file;
    ifstream meas;
#ifdef _WIN32
    log_file.open("..\\bin\\log_file_test.txt");
    meas_file.open("..\\bin\\meas_test_kf.txt");
#else
    log_file.open("log_file1.txt");
#endif
   
    /*
     * Define the system
     */
    mat A(5,5), B(5,1,fill::zeros), H(2,5), Q(5,5), R(2,2);
    
    // State transition matrix
    A << 1 << 0 << 1 << 0 << 0 << endr
      << 0 << 1 << 0 << 1 << 0 << endr
      << 0 << 0 << 1 << 0 << 0 << endr
      << 0 << 0 << 0 << 1 << 0 << endr
      << 0 << 0 << 0 << 0 << 0 << endr;
    
    // Measurement matrix
    H << 1 << 0 << 0 << 0 << 0 << endr
      << 0 << 1 << 0 << 0 << 0 << endr;
    
    // Process noise
    Q << (1/3.0) <<       0 << (1/2.0) <<       0 << 0 << endr
      <<       0 << (1/3.0) <<       0 << (1/2.0) << 0 << endr
      << (1/2.0) <<       0 <<       1 <<       0 << 0 << endr
      <<       0 << (1/2.0) <<       0 <<       1 << 0 << endr
      <<       0 <<       0 <<       0 <<       0 << 0 << endr;
      
    // Measusrement noise
    R = 10 * R.eye();
    
    // Initial state
    colvec x0(5);
    x0 << 0 << 0 << 1 << 1 << 0;

    // Control vector
    colvec u(1);
    // No inputs, system subjects only to random perturbation
    u = u.zeros(); 
    
    // Initial state covariance
    mat P0(5,5);
    P0 = 10 * P0.eye();

    // Initialize KF for generating measurements
    KF kalman;
    kalman.InitSystem(A, B, H, Q, R);
    kalman.InitSystemState(x0);
    kalman.InitStateCovariance(P0);

    // Initialize KF for doing the filtering
    KF filter;
    filter.InitSystem(A, B, H, Q, R);
    filter.InitSystemState(x0);
    filter.InitStateCovariance(P0);
  
    // Generate measurements first
    for (int k = 0; k < 100; k++) {
      kalman.Kalmanf(u);
      colvec *z = kalman.GetCurrentOutput();
      meas_file << k << ' ' << z->at(0,0) << ' ' << z->at(1,0) << '\n';

      // Put the z_measurment to the Kalman filter
      filter.Kalmanf(*z, u);
      colvec *x_m = kalman.GetCurrentEstimatedState();
      log_file << k << ' ' << x_m->at(0,0) << ' ' << x_m->at(1,0) << '\n';
    }

    meas_file.close();
    log_file.close();
    
    return 0;
  }
