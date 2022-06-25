# Test
This Repo mainly for test open project, open library and so on
### ceres_g2o_comparer
compare performance between [ceres-solver](https://github.com/ceres-solver/ceres-solver) and [g2o](https://github.com/RainerKuemmerle/g2o) by runing BA demo
##### compilation
```
git clone https://github.com/chennuo0125-HIT/test.git
cd test/ceres_g2o_comparer
./build_3th.sh
mkdir build && cd build
cmake ..
make
```
##### runing
```
./comparer
```