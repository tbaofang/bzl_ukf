#ifndef KALMAN_FILTER_GAUSSIAN_DISTRIBUTION_H_
#define KALMAN_FILTER_GAUSSIAN_DISTRIBUTION_H_

#include "Eigen/Core"

namespace kalman_filter{

template <typename T, int N>
class GaussianDistribution
{
    public:
    GaussianDistribution(const Eigen::Matrix<T, N, 1>& mean, const Eigen::Matrix<T, N, N>& covariance)
    : mean_(mean), covariance_(covariance){}

    const Eigen::Matrix<T, N, 1>& GetMean() const {return mean_;}
    const Eigen::Matrix<T, N, N>& GetCovariance() const {return covariance_;}

    private:
    Eigen::Matrix<T, N, 1> mean_;
    Eigen::Matrix<T, N, N> covariance_;
};

template <typename T, int N>
GaussianDistribution<T, N> operator+(const GaussianDistribution<T, N>& lhs,
                                     const GaussianDistribution<T, N>& rhs) {
  return GaussianDistribution<T, N>(lhs.GetMean() + rhs.GetMean(),
                                    lhs.GetCovariance() + rhs.GetCovariance());
}

template <typename T, int N, int M>
GaussianDistribution<T, N> operator*(const Eigen::Matrix<T, N, M>& lhs,
                                     const GaussianDistribution<T, M>& rhs) {
  return GaussianDistribution<T, N>(
      lhs * rhs.GetMean(), lhs * rhs.GetCovariance() * lhs.transpose());
}


}

#endif