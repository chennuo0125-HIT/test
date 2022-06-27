/*
 * @Author: chennuo0125@163.com
 */

#include "utils.hpp"
#include "g2o_ba.hpp"
#include "ceres_ba.hpp"
#include <chrono>

using namespace std;

int main(int argc, char **argv) {
  SimulationData sim_data(15, 1000, 1.0);

  // test g2o ba
  chrono::steady_clock::time_point t0 = chrono::steady_clock::now();
  G2oBA g2o_ba(sim_data);
  g2o_ba.solve();
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

  // test ceres ba
  CeresBa ceres_ba(sim_data);
  ceres_ba.solve();
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();

  double g2o_cost_time =
      chrono::duration_cast<chrono::duration<double, milli>>(t1 - t0).count();
  double ceres_cost_time =
      chrono::duration_cast<chrono::duration<double, milli>>(t2 - t1).count();
  cout << endl;
  cout << "************** cost time **************" << endl;
  cout << "g2o cost time: " << g2o_cost_time << " [ms]" << endl;
  cout << "ceres cost time: " << ceres_cost_time << " [ms]" << endl;

  return 0;
}