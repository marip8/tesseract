/**
 * @file continuous_contact_manager.h
 * @brief This is the continuous contact manager base class
 *
 * It should be used to perform continuous contact checking.
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
#ifndef TESSERACT_COLLISION_CONTINUOUS_CONTACT_MANAGER_H
#define TESSERACT_COLLISION_CONTINUOUS_CONTACT_MANAGER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/contact_manager.h>

namespace tesseract_collision
{
class ContinuousContactManager : public ContactManager
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<ContinuousContactManager>;
  using ConstPtr = std::shared_ptr<const ContinuousContactManager>;
  using UPtr = std::unique_ptr<ContinuousContactManager>;
  using ConstUPtr = std::unique_ptr<const ContinuousContactManager>;

  ContinuousContactManager() = default;
  virtual ~ContinuousContactManager() = default;
  ContinuousContactManager(const ContinuousContactManager&) = delete;
  ContinuousContactManager& operator=(const ContinuousContactManager&) = delete;
  ContinuousContactManager(ContinuousContactManager&&) = delete;
  ContinuousContactManager& operator=(ContinuousContactManager&&) = delete;

  virtual ContinuousContactManager::UPtr cloneContinuous() const = 0;
  ContactManager::UPtr clone() const override final { return cloneContinuous(); }

  using ContactManager::setCollisionObjectsTransform;

  /**
   * @brief Set a single cast(moving) collision object's tansforms
   *
   * This should only be used for moving objects. Use the base
   * class methods for static objects.
   *
   * @param name The name of the object
   * @param pose1 The start tranformation in world
   * @param pose2 The end tranformation in world
   */
  virtual void setCollisionObjectsTransform(const std::string& name,
                                            const Eigen::Isometry3d& pose1,
                                            const Eigen::Isometry3d& pose2) = 0;

  /**
   * @brief Set a series of cast(moving) collision object's tranforms
   *
   * This should only be used for moving objects. Use the base
   * class methods for static objects.
   *
   * @param names The name of the object
   * @param pose1 The start tranformations in world
   * @param pose2 The end tranformations in world
   */
  virtual void setCollisionObjectsTransform(const std::vector<std::string>& names,
                                            const tesseract_common::VectorIsometry3d& pose1,
                                            const tesseract_common::VectorIsometry3d& pose2) = 0;

  /**
   * @brief Set a series of cast(moving) collision object's tranforms
   *
   * This should only be used for moving objects. Use the base
   * class methods for static objects.
   *
   * @param pose1 A start transform map <name, pose>
   * @param pose2 A end transform map <name, pose>
   */
  virtual void setCollisionObjectsTransform(const tesseract_common::TransformMap& pose1,
                                            const tesseract_common::TransformMap& pose2) = 0;
};

}  // namespace tesseract_collision

#endif  // TESSERACT_COLLISION_CONTINUOUS_CONTACT_MANAGER_H
