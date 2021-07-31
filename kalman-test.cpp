/**
 * Test for the KalmanFilter class with 1D projectile motion.
 *
 * @author: Hayk Martirosyan
 * @date: 2014.11.15
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <Eigen/Dense>

#include "kalman.hpp"

// to remove
//************************************************************************************
#define COLUMN_WIDTH 20

int random_sign(double probability)
{
        int result = 1;

        if( (double) rand() / RAND_MAX < probability )
        {
                result *= -1;
        }

        return result;
}

// probability -> that the number will be negative
double random_float(double truth, double range, double probability=0)
{
        double result = truth + (double) rand() / RAND_MAX * range * random_sign(0.5);
        result *= random_sign(probability);

        return result;
}
//************************************************************************************

int main(int argc, char* argv[]) {

  int n = 3; // Number of states
  int m = 1; // Number of measurements

  double dt = 1.0/30; // Time step

  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd C(m, n); // Output matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  // Discrete LTI projectile motion, measuring position only
  A << 0.1, dt, 0, 0, 1, dt, 0, 0, 1;
  C << 1, 0, 0;

  // Reasonable covariance matrices
  Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
  R << 5;
  P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;

/*
  std::cout << "A: \n" << A << std::endl;
  std::cout << "C: \n" << C << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P: \n" << P << std::endl;
*/

  // Construct the filter
  KalmanFilter kf(dt,A, C, Q, R, P);

/*
  // List of noisy position measurements (y)
  std::vector<double> measurements = {
      1.04202710058, 1.10726790452, 1.2913511148, 1.48485250951, 1.72825901034,
      1.74216489744, 2.11672039768, 2.14529225112, 2.16029641405, 2.21269371128,
      2.57709350237, 2.6682215744, 2.51641839428, 2.76034056782, 2.88131780617,
      2.88373786518, 2.9448468727, 2.82866600131, 3.0006601946, 3.12920591669,
      2.858361783, 2.83808170354, 2.68975330958, 2.66533185589, 2.81613499531,
      2.81003612051, 2.88321849354, 2.69789264832, 2.4342229249, 2.23464791825,
      2.30278776224, 2.02069770395, 1.94393985809, 1.82498398739, 1.52526230354,
      1.86967808173, 1.18073207847, 1.10729605087, 0.916168349913, 0.678547664519,
      0.562381751596, 0.355468474885, -0.155607486619, -0.287198661013, -0.602973173813
  };
*/

  // Best guess of initial states
  Eigen::VectorXd x0(n);
  double t = 0;
  x0 << 20, 0, -9.81;
//  x0 << measurements[0], 0, -9.81;
  kf.init(t, x0);

  // Feed measurements into filter, output estimated states

  double initial_position = 20;
  double velocity = 1;

  Eigen::VectorXd y(m);

/*  
  std::cout << std::setw(COLUMN_WIDTH)
	    << "Time"
	    << std::setw(COLUMN_WIDTH)
	    << "Measurement"
	    << std::setw(COLUMN_WIDTH)
	    << "Estimate"
	    << std::endl;
*/

  std::cout << "Time,Measurement,Estimate" << std::endl;

  for(double time = 0; time < 20; time += 0.01)
  {
    t += dt;

    double true_position = initial_position - velocity*time;
    double measured_position = random_float(true_position, 1, 0.1);

    y << measured_position;
    kf.update(y);

/*
    std::cout << std::setw(COLUMN_WIDTH)
	      << t
	      << std::setw(COLUMN_WIDTH)

	      << y.transpose()
	      << std::setw(COLUMN_WIDTH)

	      << kf.state().transpose()[0]
	      << std::setw(COLUMN_WIDTH)

	      << y.size()
	      << std::endl;
*/
    std::cout << t
	      << ","
	      << y.transpose()
	      << ","
	      << kf.state().transpose()[0]
	      << std::endl;
  }

  return 0;
}
