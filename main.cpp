extern "C" {
  #include "solver.h"
}
#include "iostream"

Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 4
double l = 2.0;

void load_data(Params params, double v_r, double delta_r, double x_e, double y_e, double psi_e) {
  params.A[0] = 0;
  params.A[1] = 0;
  params.A[2] = -sin(delta_r) * v_r;
  params.A[3] = 0;
  params.A[4] = 0;
  params.A[5] = cos(delta_r) * v_r;
  params.A[6] = 0;
  params.A[7] = 0;
  params.A[8] = 0;

  params.B[0] = 0;
  params.B[1] = 0;
  params.B[2] = v_r * (cos(delta_r) * cos(delta_r) * l);

  params.R[0] = 1.0;

  params.Q[0] = 1.2;
  params.Q[1] = 0.0;
  params.Q[2] = 0.0;
  params.Q[3] = 0.0;
  params.Q[4] = 1.2;
  params.Q[5] = 0.0;
  params.Q[6] = 0.0;
  params.Q[7] = 0.0;
  params.Q[8] = 1.2;

  params.u_max[0] = 0.2;

  params.x_0[0] = x_e;
  params.x_0[1] = y_e;
  params.x_0[2] = psi_e;
}

struct CarStates {
  double x;
  double y;
  double psi;
}car_states;
struct StatesErr {
  double x;
  double y;
  double psi;
}states_err;

class Car {
 public:
  Car() {
    x_0 = 0.0;
    y_0 = 0.0;
    psi_0 = 0.0;
  }
  CarStates KinModelSim(double v, double delta, double delta_t);
 private:
  double x_0, y_0, psi_0;
};

CarStates Car::KinModelSim(double v, double delta, double delta_t) {
  car_states.x = x_0 + cos(delta) * v * delta_t;
  car_states.y = y_0 + sin(delta) * v * delta_t;
  car_states.psi = psi_0 + (tan(delta) / 2.2) * delta_t;
  x_0 = car_states.x;
  y_0 = car_states.y;
  psi_0 = car_states.psi;
  std::cout<<"x_0 : "<<x_0<<std::endl;
  std::cout<<"y_0 : "<<y_0<<std::endl;
  std::cout<<"psi_0 : "<<psi_0<<std::endl;
  return car_states;
}

void use_solution(Vars vars) {
  // In this function, use the optimization result.
  double *delta = vars.u_1;
}

int main(int argc, char **argv) {
  int num_iters;
  double t_sim = 10;
  double delta_t = 0.1;
  int n_sim = t_sim / delta_t;
  Car Car;
  set_defaults();  // Set basic algorithm parameters.
  setup_indexing();
  double v_r = 5.0;
  double delta = 0.0;
  states_err.x = 0.0;
  states_err.y = 0.0;
  states_err.psi = 0.0;
  double x_ref = 100.0;
  double y_ref = 5.0;
  double psi_ref = 0.3;

  for (int i = 0; i < n_sim; ++i) {
    states_err.x = car_states.x - x_ref;
    states_err.y = car_states.y - y_ref;
    states_err.psi = car_states.psi - psi_ref;
    load_data(params, v_r, *delta, states_err.x,states_err.y,states_err.psi);
    num_iters = solve();
    use_solution(vars);
    Car.KinModelSim(v_r, *delta, delta_t);
  }
  
//  for (int i = 0; i < NUMTESTS; i++) {  // Main control loop.
//    load_data(params, v_r, delta_r);
//
//    // Solve our problem at high speed!
//    num_iters = solve();
//    // Recommended: check work.converged == 1.
//
//    use_solution(vars);
//  }
}
//int main(int argc, char **argv) {
//  int num_iters;
//#if (NUMTESTS > 0)
//  int i;
//  double time;
//  double time_per;
//#endif
//  set_defaults();
//  setup_indexing();
//  load_default_data();
//  /* Solve problem instance for the record. */
//  settings.verbose = 1;
//  num_iters = solve();
//#ifndef ZERO_LIBRARY_MODE
//#if (NUMTESTS > 0)
//  /* Now solve multiple problem instances for timing purposes. */
//  settings.verbose = 0;
//  tic();
//  for (i = 0; i < NUMTESTS; i++) {
//    solve();
//  }
//  time = tocq();
//  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
//  time_per = time / NUMTESTS;
//  if (time_per > 1) {
//    printf("Actual time taken per solve: %.3g s.\n", time_per);
//  } else if (time_per > 1e-3) {
//    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
//  } else {
//    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
//  }
//#endif
//#endif
//  return 0;
//}
//void load_default_data(void) {
//  params.x_0[0] = 0.20319161029830202;
//  params.x_0[1] = 0.8325912904724193;
//  params.x_0[2] = -0.8363810443482227;
//  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
//  params.Q[0] = 1.510827605197663;
//  params.Q[3] = 0;
//  params.Q[6] = 0;
//  params.Q[1] = 0;
//  params.Q[4] = 1.8929469543476547;
//  params.Q[7] = 0;
//  params.Q[2] = 0;
//  params.Q[5] = 0;
//  params.Q[8] = 1.896293088933438;
//  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
//  params.R[0] = 1.1255853104638363;
//  params.A[0] = -1.171028487447253;
//  params.A[1] = -1.7941311867966805;
//  params.A[2] = -0.23676062539745413;
//  params.A[3] = -1.8804951564857322;
//  params.A[4] = -0.17266710242115568;
//  params.A[5] = 0.596576190459043;
//  params.A[6] = -0.8860508694080989;
//  params.A[7] = 0.7050196079205251;
//  params.A[8] = 0.3634512696654033;
//  params.B[0] = -1.9040724704913385;
//  params.B[1] = 0.23541635196352795;
//  params.B[2] = -0.9629902123701384;
//  params.u_max[0] = -0.3395952119597214;
//}