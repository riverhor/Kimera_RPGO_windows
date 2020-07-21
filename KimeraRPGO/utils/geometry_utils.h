#ifndef KIMERARPGO_UTILS_GEOMETRY_UTILS_H_
#define KIMERARPGO_UTILS_GEOMETRY_UTILS_H_

#define SLOW_BUT_CORRECT_BETWEENFACTOR

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "KimeraRPGO/logger.h"

#include <map>
#include <vector>


namespace KimeraRPGO {

template <class T>
static const size_t getRotationDim() {
  BOOST_CONCEPT_ASSERT((gtsam::IsLieGroup<T>));
  T sample_object;
  return sample_object.rotation().dimension;
}

template <class T>
static const size_t getTranslationDim() {
  BOOST_CONCEPT_ASSERT((gtsam::IsLieGroup<T>));
  T sample_object;
  return sample_object.translation().dimension;
}

template <class T>
static const size_t getDim() {
  BOOST_CONCEPT_ASSERT((gtsam::IsLieGroup<T>));
  T sample_object;
  return sample_object.dimension;
}
template <class T>
struct PoseWithCovariance {
  T pose;
  gtsam::Matrix covariance_matrix;
  bool rotation_info = true;
  PoseWithCovariance() {
    pose = T();
    const size_t dim = getDim<T>();
    gtsam::Matrix covar =
        Eigen::MatrixXd::Zero(dim, dim);
    covariance_matrix = covar;
  }
  PoseWithCovariance(T pose_in, gtsam::Matrix matrix_in) {
    pose = pose_in;
    covariance_matrix = matrix_in;
  }
  explicit PoseWithCovariance(const gtsam::PriorFactor<T>& prior_factor) {
    T value = prior_factor.prior();
    const size_t dim = getDim<T>();
    gtsam::Matrix covar =
        Eigen::MatrixXd::Zero(dim, dim);

    pose = value;
    covariance_matrix = covar;
  }

  explicit PoseWithCovariance(const gtsam::BetweenFactor<T>& between_factor) {
    pose = between_factor.measured();
    gtsam::Matrix covar =
        boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(
            between_factor.get_noiseModel())
            ->covariance();

    const int dim = getDim<T>();
    const int r_dim = getRotationDim<T>();
    const int t_dim = getTranslationDim<T>();
    rotation_info = true;
    if (std::isnan(covar.block(0, 0, r_dim, r_dim).trace())) {
      rotation_info = false;
  
      Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(
          dim, dim); 
      temp.block(r_dim, r_dim, t_dim, t_dim) =
          covar.block(r_dim, r_dim, t_dim, t_dim);
      covar = temp;
    }
    covariance_matrix = covar;
  }

  PoseWithCovariance compose(const PoseWithCovariance& other) const {
    PoseWithCovariance<T> out;
    gtsam::Matrix Ha, Hb;

    out.pose = pose.compose(other.pose, Ha, Hb);
    out.covariance_matrix = Ha * covariance_matrix * Ha.transpose() +
                            Hb * other.covariance_matrix * Hb.transpose();

    if (!rotation_info || !other.rotation_info) out.rotation_info = false;
    return out;
  }

  PoseWithCovariance inverse() const {
    PoseWithCovariance<T> out;
    out.pose = pose.inverse();
    out.covariance_matrix = covariance_matrix;
    if (!rotation_info) out.rotation_info = false;
    return out;
  }


  PoseWithCovariance between(const PoseWithCovariance& other) const {
    PoseWithCovariance<T> out;
    gtsam::Matrix Ha, Hb;
    out.pose = pose.between(other.pose, Ha, Hb); 

    out.covariance_matrix =
        other.covariance_matrix - Ha * covariance_matrix * Ha.transpose();
    bool pos_semi_def = true;
 
    Eigen::LLT<Eigen::MatrixXd> lltCovar1(out.covariance_matrix);
    if (lltCovar1.info() == Eigen::NumericalIssue) {
      pos_semi_def = false;
    }

    if (!pos_semi_def) {
      other.pose.between(pose, Ha, Hb);  
      out.covariance_matrix =
          covariance_matrix - Ha * other.covariance_matrix * Ha.transpose();

      Eigen::LLT<Eigen::MatrixXd> lltCovar2(out.covariance_matrix);

    }
    if (!rotation_info || !other.rotation_info) out.rotation_info = false;
    return out;
  }

  double mahalanobis_norm() const {

    gtsam::Vector log = T::Logmap(pose);
    if (!rotation_info) {

      int t_dim = getTranslationDim<T>();
      int r_dim = getRotationDim<T>();
      Eigen::MatrixXd cov_block =
          covariance_matrix.block(r_dim, r_dim, t_dim, t_dim);
      return std::sqrt(log.tail(t_dim).transpose() * gtsam::inverse(cov_block) *
                       log.tail(t_dim));
    }

    return std::sqrt(log.transpose() * gtsam::inverse(covariance_matrix) * log);
  }
};

template <class T>
struct PoseWithNode {

  T pose;                     
  int node;                  
  bool rotation_info = true;  

  PoseWithNode() {
    pose = T();
    node = 0;
  }

  PoseWithNode(T pose_in, int node_in) {
    pose = pose_in;
    node = node_in;
  }

  explicit PoseWithNode(const gtsam::PriorFactor<T>& prior_factor) {
    T value = prior_factor.prior();
    pose = value;
    node = 0;
  }

  explicit PoseWithNode(const gtsam::BetweenFactor<T>& between_factor) {
    pose = between_factor.measured();
    gtsam::Matrix covar =
        boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(
            between_factor.get_noiseModel())
            ->covariance();

    const int dim = getDim<T>();
    const int r_dim = getRotationDim<T>();
    const int t_dim = getTranslationDim<T>();
    rotation_info = true;
    if (std::isnan(covar.block(0, 0, r_dim, r_dim).trace())) {
      rotation_info = false;
    }
    node = 1;
  }

  PoseWithNode compose(const PoseWithNode& other) const {
    PoseWithNode<T> out;

    out.pose = pose.compose(other.pose);
    out.node = node + other.node;

    if (!rotation_info || !other.rotation_info) out.rotation_info = false;
    return out;
  }


  PoseWithNode inverse() const {
    PoseWithNode<T> out;

    out.pose = pose.inverse();
    out.node = node;

    if (!rotation_info) out.rotation_info = false;
    return out;
  }


  PoseWithNode between(const PoseWithNode& other) const {
    PoseWithNode<T> out;
    out.pose = pose.between(other.pose);  

    out.node = abs(other.node - node);

    if (!rotation_info || !other.rotation_info) out.rotation_info = false;
    return out;
  }

  double avg_trans_norm() const {
   
    gtsam::Vector log = T::Logmap(pose);
    const int t_dim = getTranslationDim<T>();
    return std::sqrt(log.tail(t_dim).transpose() * log.tail(t_dim)) / node;
  }

  double avg_rot_norm() const {

    if (!rotation_info) return 0;
    gtsam::Vector log = T::Logmap(pose);
    const int r_dim = getRotationDim<T>();
    return std::sqrt(log.head(r_dim).transpose() * log.head(r_dim)) / node;
  }
};

} 

#endif 
