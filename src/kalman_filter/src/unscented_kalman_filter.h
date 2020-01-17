#ifndef KALMAN_FILTER_UNSCENTED_KALMAN_FILTER_H_
#define KALMAN_FILTER_UNSCENTED_KALMAN_FILTER_H_

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>



#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

// #include <gaussian_distribution.h>
#include "gaussian_distribution.h"
#include <glog/logging.h>

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
      : belief_(initial_belief), add_delta_(add_delta), compute_delta_(compute_delta) 
    {
      std::cout << "[UnscentedKalmanFilter]" << std::endl;
    }


// Does the control/prediction step for the filter. The control must be
  // implicitly added by the function g which also does the state transition.
  // 'epsilon' is the additive combination of control and model noise.
  void Predict(std::function<StateType(const StateType&)> g, const GaussianDistribution<FloatType, N>& epsilon) 
  {
    std::cout << "[UKF::Predict]" << std::endl;

    CheckSymmetric(epsilon.GetCovariance());
    // Get the state mean and matrix root of its covariance.
    const StateType& mu = belief_.GetMean();
    const StateCovarianceType sqrt_sigma = MatrixSqrt(belief_.GetCovariance());
    std::vector<StateType> Y;
    Y.reserve(2 * N + 1);
    Y.emplace_back(g(mu));
    // std::cout << "[UKF::Predict][N:] " << N << std::endl;
    // std::cout << "[UKF::Predict][mu:] " << mu << std::endl;
    // std::cout << "[UKF::Predict][g(mu):] " << g(mu) << std::endl;
    // std::cout << "[UKF::Predict][Y.size():] " << Y.size() << std::endl;


    const FloatType kSqrtNPlusLambda = std::sqrt(N + kLambda);
    for (int i = 0; i < N; ++i) 
    {
    //   StateType add_delta_tmp = add_delta_(mu, kSqrtNPlusLambda * sqrt_sigma.col(i));
    //   StateType g_tmp = g(add_delta);

    //   std::cout << "[UKF::Predict][add_delta_:] " << add_delta_tmp.transpose() << std::endl;
    //   std::cout << "[UKF::Predict][g_tmp:] " << g_tmp.transpose() << std::endl;
    //   std::cout << "[][:] add_delta_(mu, kSqrtNPlusLambda * sqrt_sigma.col(i))" << add_delta_(mu, kSqrtNPlusLambda * sqrt_sigma.col(i)) << std::endl;
      // Order does not matter here as all have the same weights in the summation later on anyways.

      Y.emplace_back(g(add_delta_(mu, kSqrtNPlusLambda * sqrt_sigma.col(i))));
      Y.emplace_back(g(add_delta_(mu, -kSqrtNPlusLambda * sqrt_sigma.col(i))));
    }
    const StateType new_mu = ComputeMean(Y);

    double k_cov_weight_0 = kCovWeight0; //-999996
    double k_cov_weight_i = kCovWeightI; //55555.6

    std::cout << "[UKF::Predict][k_cov_weight_0:] " << k_cov_weight_0 << std::endl;
    std::cout << "[UKF::Predict][k_cov_weight_i:] " << k_cov_weight_i << std::endl;

    // StateCovarianceType new_sigma = kCovWeight0 * OuterProduct<FloatType, N>(compute_delta_(new_mu, Y[0]));
    StateCovarianceType new_sigma = k_cov_weight_0 * OuterProduct<FloatType, N>(compute_delta_(new_mu, Y[0]));
    for (int i = 0; i < N; ++i) {
    //   new_sigma += kCovWeightI * OuterProduct<FloatType, N>( compute_delta_(new_mu, Y[2 * i + 1]));
    //   new_sigma += kCovWeightI * OuterProduct<FloatType, N>( compute_delta_(new_mu, Y[2 * i + 2]));
      new_sigma += k_cov_weight_i * OuterProduct<FloatType, N>( compute_delta_(new_mu, Y[2 * i + 1]));
      new_sigma += k_cov_weight_i * OuterProduct<FloatType, N>( compute_delta_(new_mu, Y[2 * i + 2]));
    }

    CheckSymmetric(new_sigma);

    belief_ = GaussianDistribution<FloatType, N>(new_mu, new_sigma) + epsilon;
  }


  // The observation step of the Kalman filter. 'h' transfers the state
  // into an observation that should be zero, i.e., the sensor readings should
  // be included in this function already. 'delta' is the measurement noise and
  // must have zero mean.
  template <int K>
  void Observe(std::function<Eigen::Matrix<FloatType, K, 1>(const StateType&)> h,
               const GaussianDistribution<FloatType, K>& delta) 
  {
    CheckSymmetric(delta.GetCovariance());
    // We expect zero mean delta.
    CHECK_NEAR(delta.GetMean().norm(), 0., 1e-9);

    // Get the state mean and matrix root of its covariance.
    const StateType& mu = belief_.GetMean();
    const StateCovarianceType sqrt_sigma = MatrixSqrt(belief_.GetCovariance());

    // As in Kraft's paper, we compute W containing the zero-mean sigma points,
    // since this is all we need.
    std::vector<StateType> W;
    W.reserve(2 * N + 1);
    W.emplace_back(StateType::Zero());

    std::vector<Eigen::Matrix<FloatType, K, 1>> Z;
    Z.reserve(2 * N + 1);
    Z.emplace_back(h(mu));

    double k_mean_weight_0 = kMeanWeight0;
    double k_mean_weight_i = kMeanWeightI;
    double k_cov_weight_0 = kCovWeight0;
    double k_cov_weight_i = kCovWeightI;
    // Eigen::Matrix<FloatType, K, 1> z_hat = kMeanWeight0 * Z[0];
    Eigen::Matrix<FloatType, K, 1> z_hat = k_mean_weight_0 * Z[0];
    const FloatType kSqrtNPlusLambda = std::sqrt(N + kLambda);

    std::cout << "[UKF::Observe][z_hat.transpose():] " << z_hat.transpose() << std::endl;
    std::cout << "[UKF::Observe][kSqrtNPlusLambda:] " << kSqrtNPlusLambda << std::endl;

    for (int i = 0; i < N; ++i) {
      // Order does not matter here as all have the same weights in the
      // summation later on anyways.
      W.emplace_back(kSqrtNPlusLambda * sqrt_sigma.col(i));
      Z.emplace_back(h(add_delta_(mu, W.back())));

      W.emplace_back(-kSqrtNPlusLambda * sqrt_sigma.col(i));
      Z.emplace_back(h(add_delta_(mu, W.back())));

    //   z_hat += kMeanWeightI * Z[2 * i + 1];
    //   z_hat += kMeanWeightI * Z[2 * i + 2];
      z_hat += k_mean_weight_i * Z[2 * i + 1];
      z_hat += k_mean_weight_i * Z[2 * i + 2];
    }

    std::cout << "[UKF::Observe][K:] " << K << std::endl;

    Eigen::Matrix<FloatType, K, K> S = k_cov_weight_0 * OuterProduct<FloatType, K>(Z[0] - z_hat);
    for (int i = 0; i < N; ++i) 
    {
      S += k_cov_weight_i * OuterProduct<FloatType, K>(Z[2 * i + 1] - z_hat);
      S += k_cov_weight_i * OuterProduct<FloatType, K>(Z[2 * i + 2] - z_hat);
    }
    CheckSymmetric(S);
    S += delta.GetCovariance();

    Eigen::Matrix<FloatType, N, K> sigma_bar_xz = k_cov_weight_0 * W[0] * (Z[0] - z_hat).transpose();
    for (int i = 0; i < N; ++i) 
    {
      sigma_bar_xz += static_cast<double>(kCovWeightI) * W[2 * i + 1] * (Z[2 * i + 1] - z_hat).transpose();
      sigma_bar_xz += static_cast<double>(kCovWeightI) * W[2 * i + 2] * (Z[2 * i + 2] - z_hat).transpose();
    }

    const Eigen::Matrix<FloatType, N, K> kalman_gain = sigma_bar_xz * S.inverse();
    const StateCovarianceType new_sigma = belief_.GetCovariance() - kalman_gain * S * kalman_gain.transpose();
    CheckSymmetric(new_sigma);

    belief_ = GaussianDistribution<FloatType, N>(add_delta_(mu, kalman_gain * -z_hat), new_sigma);
  }


  const GaussianDistribution<FloatType, N>& GetBelief() const 
  {
    return belief_;
  }
private:

  StateType ComputeWeightedError(const StateType& mean_estimate, const std::vector<StateType>& states) 
  {

    double aa0 = kMeanWeight0;  
    double aai = kMeanWeightI;

    //  std::cout << "[UKF::ComputeWeightedError][states[0].size():] " << states[0].size() << std::endl; 
    // std::cout << "[UKF::ComputeWeightedError][states[0]:] " << states[0] << std::endl;
    // std::cout << "[UKF::ComputeWeightedError][states[0].transpose():] " << states[0].transpose() << std::endl;
    // std::cout << "[UKF::ComputeWeightedError][kMeanWeight0:] " << kMeanWeight0 << std::endl;
    // std::cout << "[UKF::ComputeWeightedError][kMeanWeightI:] " << kMeanWeightI << std::endl;
    // std::cout << "[UKF::ComputeWeightedError][aa:] " << aa0 << std::endl;
    // std::cout << "[UKF::ComputeWeightedError][mean_estimate:] " << mean_estimate.transpose() << std::endl;
    // std::cout << "[UKF::ComputeWeightedError][states[0]:] " << states[0].transpose() << std::endl;
    // std::cout << "[UKF::ComputeWeightedError][kMeanWeight0 * compute_delta_(mean_estimate, states[0]):] " << 
    // (aa0 * compute_delta_(mean_estimate, states[0])).transpose() << std::endl;

    // StateType weighted_error =  kMeanWeight0 * compute_delta_(mean_estimate, states[0]);
    StateType weighted_error =  aa0 * compute_delta_(mean_estimate, states[0]);
    for (int i = 1; i != 2 * N + 1; ++i) {
    //   weighted_error += kMeanWeightI * compute_delta_(mean_estimate, states[i]);
      weighted_error += aai * compute_delta_(mean_estimate, states[i]);
    }

    std::cout << "[UKF::ComputeWeightedError][weighted_error.transpose():] " << weighted_error.transpose() << std::endl;

    return weighted_error;
  }




  // Algorithm for computing the mean of non-additive states taken from Kraft's
  // Section 3.4, adapted to our implementation.
  StateType ComputeMean(const std::vector<StateType>& states) 
  {
    std::cout << "[UKF::ComputeMean][states.size():] " << states.size() << std::endl;  
    CHECK_EQ(states.size(), 2 * N + 1);
    StateType current_estimate = states[0];
    StateType weighted_error = ComputeWeightedError(current_estimate, states);
    int iterations = 0;

    std::cout << "[UKF::ComputeMean][weighted_error.norm():] " << weighted_error.norm() << std::endl;
    while (weighted_error.norm() > 1e-9) {
      double step_size = 1.;
      while (true) {
        const StateType next_estimate = add_delta_(current_estimate, step_size * weighted_error);
        const StateType next_error = ComputeWeightedError(next_estimate, states);
        if (next_error.norm() < weighted_error.norm()) 
        {
          current_estimate = next_estimate;
          weighted_error = next_error;
          break;
        }
        step_size *= 0.5;
        CHECK_GT(step_size, 1e-3) << "Step size too small, line search failed.";
      }
      ++iterations;
      CHECK_LT(iterations, 20) << "Too many iterations.";
    }
    return current_estimate;
  }



  // According to Wikipedia these are the normal values. Thrun does not mention those.
  constexpr static FloatType kAlpha = 1e-3;
  constexpr static FloatType kKappa = 0.;
  constexpr static FloatType kBeta = 2.;
  constexpr static FloatType kLambda = sqr(kAlpha) * (N + kKappa) - N;
  constexpr static FloatType kMeanWeight0 = kLambda / (N + kLambda);
  constexpr static FloatType kCovWeight0 = kLambda / (N + kLambda) + (1. - sqr(kAlpha) + kBeta);
  constexpr static FloatType kMeanWeightI = 1. / (2. * (N + kLambda));
  constexpr static FloatType kCovWeightI = kMeanWeightI;

  GaussianDistribution<FloatType, N> belief_;
  const std::function<StateType(const StateType& state, const StateType& delta)> add_delta_;
  const std::function<StateType(const StateType& origin, const StateType& target)> compute_delta_;

};


template <typename FloatType, int N>
constexpr FloatType UnscentedKalmanFilter<FloatType, N>::kAlpha;
template <typename FloatType, int N>
constexpr FloatType UnscentedKalmanFilter<FloatType, N>::kKappa;
template <typename FloatType, int N>
constexpr FloatType UnscentedKalmanFilter<FloatType, N>::kBeta;
template <typename FloatType, int N>
constexpr FloatType UnscentedKalmanFilter<FloatType, N>::kLambda;
template <typename FloatType, int N>
constexpr FloatType UnscentedKalmanFilter<FloatType, N>::kMeanWeight0;
template <typename FloatType, int N>
constexpr FloatType UnscentedKalmanFilter<FloatType, N>::kCovWeight0;
template <typename FloatType, int N>
constexpr FloatType UnscentedKalmanFilter<FloatType, N>::kMeanWeightI;
template <typename FloatType, int N>
constexpr FloatType UnscentedKalmanFilter<FloatType, N>::kCovWeightI;



}







#endif
