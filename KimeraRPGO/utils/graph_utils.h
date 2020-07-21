#ifndef KIMERARPGO_UTILS_GRAPH_UTILS_H_
#define KIMERARPGO_UTILS_GRAPH_UTILS_H_

#include <map>
#include <unordered_map>
#include <vector>

#include <gtsam/inference/Symbol.h>
#include <Eigen/Dense>

namespace KimeraRPGO {

int findMaxClique(const Eigen::MatrixXd adjMatrix,
                  std::vector<int>& max_clique);

int findMaxCliqueHeu(const Eigen::MatrixXd adjMatrix,
                     std::vector<int>& max_clique);

template <class poseT, template <class> class T>
struct Trajectory {
  std::unordered_map<gtsam::Key, T<poseT>> poses;

  T<poseT> getBetween(const gtsam::Key& key_a, const gtsam::Key& key_b) {
    gtsam::Symbol symb_key_a(key_a);
    gtsam::Symbol symb_key_b(key_b);
    if (symb_key_a.chr() == symb_key_b.chr()) {
      
      return poses[key_a].between(poses[key_b]);
    } else {
      char prefix_a = symb_key_a.chr();
      char prefix_b = symb_key_b.chr();
  
      gtsam::Key a0 = gtsam::Symbol(prefix_a, 0);
      gtsam::Key b0 = gtsam::Symbol(prefix_b, 0);
      T<poseT> pose_a = poses[a0].between(poses[key_a]);
      T<poseT> pose_b = poses[b0].between(poses[key_b]);
      T<poseT> pose_a0b0 = poses[a0].between(poses[b0]);

      T<poseT> result = pose_a.inverse().compose(pose_a0b0);
      result = result.compose(pose_b);
      return result;
    }
  }
};

} 

#endif  
