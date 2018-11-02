/**
 * @file base_perception_subsystem.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_BASE_PERCEPTION_SUBSYSTEM_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_BASE_PERCEPTION_SUBSYSTEM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <string>

#include "fordyca/ds/perceived_arena_map.hpp"
#include "fordyca/metrics/world_model_metrics.hpp"
#include "fordyca/params/perception_params.hpp"
#include "rcppsw/common/common.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation {
class line_of_sight;
} // namespace representation

namespace ds {
class perceived_arena_map;
} // namespace ds

NS_START(controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class base_perception_subsystem
 * @ingroup controller
 *
 * @brief Manages all robot perception of the environment; vis a vie, how to
 * take what the sensors read and turn it into a useful internal
 * representation.
 */
class base_perception_subsystem
    : public rcppsw::er::client<base_perception_subsystem>,
      public metrics::world_model_metrics {
 public:
  base_perception_subsystem(const params::perception_params* const params,
                            const std::string& id);

  /* world model metrics */
  uint cell_state_inaccuracies(uint state) const override {
    return m_cell_stats[state];
  }
  void reset_metrics(void) override;
  double known_percentage(void) const override;
  double unknown_percentage(void) const override;

  /**
   * @brief Update the robot's perception of the environment, passing it its
   * current line of sight.
   *
   * @param los The current line of sight.
   */
  void update(const representation::line_of_sight* const los);

  /**
   * @brief Reset the robot's perception of the environment to an initial state
   */
  void reset(void);

  const ds::perceived_arena_map* map(void) const { return m_map.get(); }
  ds::perceived_arena_map* map(void) { return m_map.get(); }

 protected:
  /*
   * @brief Update the perceived arena map with the current line-of-sight,
   * update the relevance of information (density) within it, and fix any blocks
   * that should be hidden from our awareness.
   *
   * @param c_los The LOS to process.
   */
  virtual void process_los(const representation::line_of_sight* const c_los);

  /**
   * @brief The processing of the current LOS after processing (i.e. does the
   * PAM now accurately reflect what was in the LOS)?
   *
   * @param c_los Current LOS.
   */
  virtual void processed_los_verify(
      const representation::line_of_sight* const c_los) const;

 private:
  /**
   * @brief Update the aggregate stats on inaccuracies in the robot's perceived
   * arena map for this timestep.
   *
   * @param los The current LOS
   */
  void update_cell_stats(const representation::line_of_sight* const los);

  // clang-format off
  std::vector<uint>                        m_cell_stats;
  std::unique_ptr<ds::perceived_arena_map> m_map;
  // clang-format on
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_BASE_PERCEPTION_SUBSYSTEM_HPP_ */
