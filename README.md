# Test
This Repo mainly for test open project, open library and so on
### ceres_g2o_comparer
compare performance between [ceres-solver](https://github.com/ceres-solver/ceres-solver) and [g2o](https://github.com/RainerKuemmerle/g2o) by runing BA demo
##### dependence
[ceres-solver 2.0.0](https://github.com/ceres-solver/ceres-solver/tree/2.0.0)
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
##### result
```
************** solve ba by g2o ***************
# Using EigenSparseCholesky poseDim 6 landMarkDim 3 blockordering 1
Performing full BA:
iteration= 0	 chi2= 286855484.122269	 time= 0.00441121	 cumTime= 0.00441121	 edges= 9654	 schur= 1	 lambda= 19154.029012	 levenbergIter= 1
iteration= 1	 chi2= 19535741.702709	 time= 0.00244025	 cumTime= 0.00685145	 edges= 9654	 schur= 1	 lambda= 6706.446950	 levenbergIter= 1
iteration= 2	 chi2= 581794.275168	 time= 0.00257346	 cumTime= 0.00942492	 edges= 9654	 schur= 1	 lambda= 2235.482317	 levenbergIter= 1
iteration= 3	 chi2= 167818.764639	 time= 0.00265876	 cumTime= 0.0120837	 edges= 9654	 schur= 1	 lambda= 745.160772	 levenbergIter= 1
iteration= 4	 chi2= 128354.214987	 time= 0.00281681	 cumTime= 0.0149005	 edges= 9654	 schur= 1	 lambda= 248.386924	 levenbergIter= 1
iteration= 5	 chi2= 81637.720421	 time= 0.00242585	 cumTime= 0.0173263	 edges= 9654	 schur= 1	 lambda= 82.795641	 levenbergIter= 1
iteration= 6	 chi2= 36584.075486	 time= 0.00243203	 cumTime= 0.0197584	 edges= 9654	 schur= 1	 lambda= 27.598547	 levenbergIter= 1
iteration= 7	 chi2= 22080.426196	 time= 0.00248485	 cumTime= 0.0222432	 edges= 9654	 schur= 1	 lambda= 9.199516	 levenbergIter= 1
iteration= 8	 chi2= 20220.294763	 time= 0.00260873	 cumTime= 0.024852	 edges= 9654	 schur= 1	 lambda= 6.133010	 levenbergIter= 1
iteration= 9	 chi2= 20173.818840	 time= 0.00242993	 cumTime= 0.0272819	 edges= 9654	 schur= 1	 lambda= 4.088674	 levenbergIter= 1

point error before optimize: 3.02176
point error  after optimize: 0.453064


************** solve ba by ceres **************

Solver Summary (v 2.0.0-eigen-(3.3.7)-lapack-eigensparse-no_openmp)

                                     Original                  Reduced
Parameter blocks                          730                      728
Parameters                               2250                     2236
Effective parameters                     2235                     2223
Residual blocks                          9654                     9654
Residuals                               19308                    19308

Minimizer                        TRUST_REGION

Sparse linear algebra library    EIGEN_SPARSE
Trust region strategy     LEVENBERG_MARQUARDT

                                        Given                     Used
Linear solver                    SPARSE_SCHUR             SPARSE_SCHUR
Threads                                     1                        1
Linear solver ordering              AUTOMATIC                   715,13
Schur structure                         2,3,6                    2,3,6

Cost:
Initial                          1.442765e+09
Final                            1.215416e+09
Change                           2.273490e+08

Minimizer iterations                       18
Successful steps                            7
Unsuccessful steps                         11

Time (in seconds):
Preprocessor                         0.004087

  Residual only evaluation           0.009167 (18)
  Jacobian & residual evaluation     0.009303 (7)
  Linear solver                      0.053818 (18)
Minimizer                            0.076861

Postprocessor                        0.000090
Total                                0.081039

Termination:                      CONVERGENCE (Function tolerance reached. |cost_change|/cost: 2.334282e-07 <= 1.000000e-06)


point error before optimize: 3.02176
point error  after optimize: 1.83598

************** cost time **************
g2o cost time: 40.1469 [ms]
ceres cost time: 84.7236 [ms]
```