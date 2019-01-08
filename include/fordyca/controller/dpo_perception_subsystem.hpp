/**
 * @file dpo_perception_subsystem.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DPO_PERCEPTION_SUBSYSTEM_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DPO_PERCEPTION_SUBSYSTEM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/controller/base_perception_subsystem.hpp"
#include "rcppsw/common/common.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace ds {
class dpo_store;
}
namespace params { namespace perception {
struct perception_params;
}} // namespace params::perception

NS_START(controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class dpo_perception_subsystem
 * @ingroup controller
 *
 * @brief Translates the sensor readings of the robot (i.e. \ref line_of_sight),
 * into a useful internal representation: a \ref dpo_store.
 */
class dpo_perception_subsystem
    : public rcppsw::er::client<dpo_perception_subsystem>,
      public base_perception_subsystem {
 public:
  explicit dpo_perception_subsystem(
      const struct params::perception::perception_params* params);
  ~dpo_perception_subsystem(void) override;

  /**
   * @brief Update the robot's perception of the environment, passing it its
   * current line of sight.
   *
   * @param los The current line of sight.
   */
  void update(void) override;

  /**
   * @brief Reset the robot's perception of the environment to an initial state
   */
  void reset(void) override;

  const ds::dpo_store* store(void) const { return m_store.get(); }
  ds::dpo_store* store(void) { return m_store.get(); }

 private:
  /*
   * @brief Update the perceived arena map with the current line-of-sight,
   * update the relevance of information (density) within it (blocks and
   * caches).
   *
   * @param c_los The LOS to process.
   */
  void process_los(const representation::line_of_sight* const c_los);

  void process_los_blocks(const representation::line_of_sight* const c_los);
  void process_los_caches(const representation::line_of_sight* const c_los);

  /**
   * @brief The processing of the current LOS after processing (i.e. does the
   * PAM now accurately reflect what was in the LOS)?
   *
   * @param c_los Current LOS.
   */
  void processed_los_verify(
      const representation::line_of_sight* const c_los) const;

 private:
  // clang-format off
  std::unique_ptr<ds::dpo_store>                 m_store;
  // clang-format on
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DPO_PERCEPTION_SUBSYSTEM_HPP_ */
