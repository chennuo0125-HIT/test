/*
 * @Author: chennuo0125@163.com
 */

#include "utils.hpp"
#include "g2o_ba.hpp"
#include "ceres_ba.hpp"

using namespace std;

int main(int argc, char **argv) {
  SimulationData sim_data(15, 1000, 1.0);

  // test g2o ba
  G2oBA g2o_ba(sim_data);
  g2o_ba.solve();

  // test ceres ba
  CeresBa ceres_ba(sim_data);
  ceres_ba.solve();

  return 0;
}