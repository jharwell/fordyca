/**
 * \file mdpo_perception_subsystem.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>
#include <vector>

#include "rcppsw/er/client.hpp"

#include "fordyca/subsystem/perception/config/perception_config.hpp"
#include "fordyca/subsystem/perception/foraging_perception_subsystem.hpp"
#include "fordyca/fordyca.hpp"
#include "fordyca/metrics/perception/mdpo_metrics.hpp"
#include "fordyca/repr/forager_los.hpp"
#include "fordyca/subsystem/perception/perception_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, subsystem, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class mdpo_perception_subsystem
 * \ingroup subsystem perception
 *
 * \brief Translates the sensor readings of the robot (i.e. \ref forager_los),
 * into a useful internal repr: a \ref dpo_semantic_map.
 */
class mdpo_perception_subsystem final
    : public rer::client<mdpo_perception_subsystem>,
      public foraging_perception_subsystem,
      public metrics::perception::mdpo_metrics {
 public:
  explicit mdpo_perception_subsystem(const config::perception_config* config);
  ~mdpo_perception_subsystem(void) override;

  /* world model metrics */
  uint cell_state_inaccuracies(uint state) const override {
    return m_cell_stats[state];
  }
  void reset_metrics(void) override;
  double known_percentage(void) const override RCPPSW_PURE;
  double unknown_percentage(void) const override RCPPSW_PURE;

  /* foraging_perception_subsystem overrides */
  const known_objects_accessor* known_objects(void) const override RCPPSW_PURE;
  void update(oracular_info_receptor* receptor) override;

  /**
   * \brief Reset the robot's perception of the environment to an initial state
   */
  void reset(void) override;

 private:
  /*
   * \brief Update the perceived arena map with the current line-of-sight,
   * update the relevance of information (density) within it, and fix any blocks
   * that should be hidden from our awareness.
   *
   * \param c_los The LOS to process.
   */
  void process_los(const repr::forager_los* c_los,
                   oracular_info_receptor* receptor);
  void process_los_blocks(const repr::forager_los* c_los);
  void process_los_caches(const repr::forager_los* c_los);

  /**
   * \brief Update the aggregate stats on inaccuracies in the robot's perceived
   * arena map for this timestep.
   *
   * \param c_los The current LOS
   */
  void update_cell_stats(const repr::forager_los* c_los);

  const ds::dpo_semantic_map* map(void) const RCPPSW_PURE;
  ds::dpo_semantic_map* map(void) RCPPSW_PURE;

  /* clang-format off */
  std::vector<size_t>                   m_cell_stats;
  std::unique_ptr<repr::forager_los>    m_los;
  /* clang-format on */
};

NS_END(perception, subsystem, fordyca);

