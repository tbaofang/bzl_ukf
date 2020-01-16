#ifndef KALMAN_FILTER_UNSCENTED_KALMAN_FILTER_H_
#define KALMAN_FILTER_UNSCENTED_KALMAN_FILTER_H_

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

// #include <gaussian_distribution.h>
#include "gaussian_distribution.h"


namespace kalman_filter
{

template <typename FloatType>
constexpr FloatType sqr(FloatType a) {
    return a * a;
}

template <typename FloatType, int N>
Eigen::Matrix<FloatType, N, N> OuterProduct(const Eigen::Matrix<FloatType, N, 1>& v) {
    return v * v.transpose();
}

// Checks if 'A' is a symmetric matrix.
template <typename FloatType, int N>
void CheckSymmetric(const Eigen::Matrix<FloatType, N, N>& A) {
  // This should be pretty much Eigen::Matrix<>::Zero() if the matrix is
  // symmetric.
  const FloatType norm = (A - A.transpose()).norm();
  CHECK(!std::isnan(norm) && std::abs(norm) < 1e-5)
      << "Symmetry check failed with norm: '" << norm << "' from matrix:\n"
      << A;
}


// Returns the matrix square root of a symmetric positive semidefinite matrix.
template <typename FloatType, int N>
Eigen::Matrix<FloatType, N, N> MatrixSqrt(const Eigen::Matrix<FloatType, N, N>& A) 
{

//   std::cout << "[MatrixSqrt]"<< std::endl;
//   for(int i = 0; i < A.rows(); ++i)
//   {
//       std::cout << "[MatrixSqrt][A:] " << A.row(i) << std::endl;
//   }

  CheckSymmetric(A);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<FloatType, N, N>>
      adjoint_eigen_solver((A + A.transpose()) / 2.);

  const auto& eigenvalues = adjoint_eigen_solver.eigenvalues();

  CHECK_GT(eigenvalues.minCoeff(), -1e-5)
      << "MatrixSqrt failed with negative eigenvalues: "
      << eigenvalues.transpose();

  return adjoint_eigen_solver.eigenvectors() *
         adjoint_eigen_solver.eigenvalues()
             .cwiseMax(Eigen::Matrix<FloatType, N, 1>::Zero())
             .cwiseSqrt()
             .asDiagonal() *
         adjoint_eigen_solver.eigenvectors().transpose();

// std::cout << "[MatrixSqrt][eigenvalues:] " << eigenvalues.transpose() << std::endl;

}





template<typename FloatType, int N>
class UnscentedKalmanFilter {
public:
    using StateType = Eigen::Matrix<FloatType, N, 1>;
    using StateCovarianceType = Eigen::Matrix<FloatType, N, N>;

    explicit UnscentedKalmanFilter(
      const GaussianDistribution<FloatType, N>& initial_belief,
      std::function<StateType(const StateType& state, const StateType& delta)>
          add_delta = [](const StateType& state, const StateType& delta) { return state + delta; },
      std::function<StateType(const StateType& origin, const StateType& target)>
          compute_delta = [](const StateType& origin, const StateType& target) { return target - origin; })
      : belief_(initial_belief),
        add_delta_(add_delta),
        compute_delta_(compute_delta) 
    {

    }


// Does the control/prediction step for the filter. The control must be
  // implicitly added by the function g which also does the state transition.
  // 'epsilon' is the additive combination of control and model noise.
    void Predict(std::function<StateType(const StateType&)> g,
               const GaussianDistribution<FloatType, N>& epsilon) 
    {
        std::cout << "[UKF::Predict]" << std::endl;

        CheckSymmetric(epsilon.GetCovariance());

        // Get the state mean and matrix root of its covariance.
        const StateType& mu = belief_.GetMean();
        const StateCovarianceType sqrt_sigma = MatrixSqrt(belief_.GetCovariance());

        std::vector<StateType> Y;
        Y.reserve(2 * N + 1);
        Y.emplace_back(g(mu));

        std::cout << "[UKF::Predict][N:] " << N << std::endl;
        std::cout << "[UKF::Predict][mu:] " << mu << std::endl;
        std::cout << "[UKF::Predict][g(mu):] " << g(mu) << std::endl;
        std::cout << "[UKF::Predict][Y.size():] " << Y.size() << std::endl;
    // const FloatType kSqrtNPlusLambda = std::sqrt(N + kLambda);
    // for (int i = 0; i < N; ++i) {
    //   // Order does not matter here as all have the same weights in the
    //   // summation later on anyways.
    //   Y.emplace_back(g(add_delta_(mu, kSqrtNPlusLambda * sqrt_sigma.col(i))));
    //   Y.emplace_back(g(add_delta_(mu, -kSqrtNPlusLambda * sqrt_sigma.col(i))));
    // }
    // const StateType new_mu = ComputeMean(Y);

    // StateCovarianceType new_sigma =
    //     kCovWeight0 * OuterProduct<FloatType, N>(compute_delta_(new_mu, Y[0]));
    // for (int i = 0; i < N; ++i) {
    //   new_sigma += kCovWeightI * OuterProduct<FloatType, N>(
    //                                  compute_delta_(new_mu, Y[2 * i + 1]));
    //   new_sigma += kCovWeightI * OuterProduct<FloatType, N>(
    //                                  compute_delta_(new_mu, Y[2 * i + 2]));
    // }
    // CheckSymmetric(new_sigma);

    // belief_ = GaussianDistribution<FloatType, N>(new_mu, new_sigma) + epsilon;
  }


  const GaussianDistribution<FloatType, N>& GetBelief() const 
  {
    return belief_;
  }
private:
    GaussianDistribution<FloatType, N> belief_;
    const std::function<StateType(const StateType& state, const StateType& delta)> add_delta_;
    const std::function<StateType(const StateType& origin, const StateType& target)> compute_delta_;

};



}







#endif
