/**
 * \file foraging_perception_subsystem.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * This file is part of FORDYCA.
 *
 * FORDYCA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * FORDYCA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * FORDYCA.  If not, see <http://www.gnu.org/licenses/
 */

#ifndef INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_FORAGING_PERCEPTION_SUBSYSTEM_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_FORAGING_PERCEPTION_SUBSYSTEM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/subsystem/perception/base_perception_subsystem.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/repr/forager_los.hpp"
#include "fordyca/ds/access_known_objects.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive);

class oracular_info_receptor;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class foraging_perception_subsystem
 * \ingroup controller cognitive
 *
 * \brief Base class for robot perception common to all foraging controllers,
 * which is just the \ref dpo_store of objects.
 */
class foraging_perception_subsystem
    : public csperception::base_perception_subsystem<repr::forager_los> {
 public:
  explicit foraging_perception_subsystem(
      const cspconfig::perception_config* const pconfig)
      : base_perception_subsystem(pconfig) {}

  ~foraging_perception_subsystem(void) override = default;

  /**
   * \brief Update the internal data structure/repr of the
   * environment/arena, after the LOS has been updated.
   *
   * \param receptor Handle to \ref oracular_info_receptor containing perfect
   * information about the environment. If non-NULL, bypasses LOS updating (why
   * use LOS when you have perception information?).
   */
  virtual void update(oracular_info_receptor* receptor) = 0;

  /**
   * \brief Get access to all objects known to the robot, independent of
   * perception model.
   */
  virtual const ds::access_known_objects* known_objects(void) const = 0;
  virtual ds::access_known_objects* known_objects(void) = 0;

  template<typename TModelType>
  const TModelType* model(void) const {
    return static_cast<const TModelType*>(base_perception_subsystem::model());
  }
  template<typename TModelType>
  TModelType* model(void) {
    return static_cast<TModelType*>(base_perception_subsystem::model());
  }
};

NS_END(cognitive, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_FORAGING_PERCEPTION_SUBSYSTEM_HPP_ */
