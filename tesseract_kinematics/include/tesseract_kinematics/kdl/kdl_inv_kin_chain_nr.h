/**
 * @file kdl_fwd_kin_chain_nr.h
 * @brief Tesseract KDL inverse kinematics chain Newton-Raphson implementation.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_KINEMATICS_KDL_INV_KIN_CHAIN_NR_H
#define TESSERACT_KINEMATICS_KDL_INV_KIN_CHAIN_NR_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <unordered_map>
#include <console_bridge/console.h>

#include <tesseract_scene_graph/graph.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/kdl/kdl_utils.h>

#ifdef SWIG
%shared_ptr(tesseract_kinematics::KDLInvKinChainNR)
#endif  // SWIG

namespace tesseract_kinematics
{
static const std::string KDL_INV_KIN_CHAIN_NR_SOLVER_NAME = "KDLInvKinChainNR";

/**
 * @brief KDL Inverse kinematic chain implementation.
 */
class KDLInvKinChainNR : public InverseKinematics
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<KDLInvKinChainNR>;
  using ConstPtr = std::shared_ptr<const KDLInvKinChainNR>;
  using UPtr = std::unique_ptr<KDLInvKinChainNR>;
  using ConstUPtr = std::unique_ptr<const KDLInvKinChainNR>;

  ~KDLInvKinChainNR() override final = default;
  KDLInvKinChainNR(const KDLInvKinChainNR& other);
  KDLInvKinChainNR& operator=(const KDLInvKinChainNR& other);
  KDLInvKinChainNR(KDLInvKinChainNR&&) = default;
  KDLInvKinChainNR& operator=(KDLInvKinChainNR&&) = default;

  /**
   * @brief Construct KDL Forward Kinematics
   * Creates KDL::Chain from tesseract scene graph
   * @param name The name of the kinematic chain
   * @param scene_graph The Tesseract Scene Graph
   * @param base_link The name of the base link for the kinematic chain
   * @param tip_link The name of the tip link for the kinematic chain
   */
  KDLInvKinChainNR(const std::string& name,
                   const tesseract_scene_graph::SceneGraph& scene_graph,
                   const std::string& base_link,
                   const std::string& tip_link,
                   std::string solver_name = KDL_INV_KIN_CHAIN_NR_SOLVER_NAME);

  /**
   * @brief Construct Inverse Kinematics as chain
   * Creates a inverse kinematic chain object from sequential chains
   * @param scene_graph The Tesseract Scene Graph
   * @param chains A vector of kinematics chains <base_link, tip_link> that get concatenated
   * @param name The name of the kinematic chain
   */
  KDLInvKinChainNR(std::string name,
                   const tesseract_scene_graph::SceneGraph& scene_graph,
                   const std::vector<std::pair<std::string, std::string> >& chains,
                   std::string solver_name = KDL_INV_KIN_CHAIN_NR_SOLVER_NAME);

  IKSolutions calcInvKin(const IKInput& tip_link_poses,
                         const Eigen::Ref<const Eigen::VectorXd>& seed) const override final;

  std::vector<std::string> getJointNames() const override final;
  Eigen::Index numJoints() const override final;
  std::string getBaseLinkName() const override final;
  std::string getWorkingFrame() const override final;
  std::vector<std::string> getTipLinkNames() const override final;
  std::string getName() const override final;
  std::string getSolverName() const override final;
  InverseKinematics::UPtr clone() const override final;

private:
  KDLChainData kdl_data_;                                       /**< @brief KDL data parsed from Scene Graph */
  std::string name_;                                            /**< @brief Name of the kinematic chain */
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;  /**< @brief KDL Forward Kinematic Solver */
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;   /**< @brief KDL Inverse kinematic velocity solver */
  std::unique_ptr<KDL::ChainIkSolverPos_NR> ik_solver_;         /**< @brief KDL Inverse kinematic solver */
  std::string solver_name_{ KDL_INV_KIN_CHAIN_NR_SOLVER_NAME }; /**< @brief Name of this solver */

  /** @brief calcFwdKin helper function */
  IKSolutions calcInvKinHelper(const Eigen::Isometry3d& pose,
                               const Eigen::Ref<const Eigen::VectorXd>& seed,
                               int segment_num = -1) const;
};

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_KDL_INV_KIN_CHAIN_NR_H
