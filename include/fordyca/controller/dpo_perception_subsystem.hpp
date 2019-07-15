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
#include "fordyca/metrics/perception/dpo_perception_metrics.hpp"
#include "fordyca/nsalias.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace ds {
class dpo_store;
}
namespace config { namespace perception {
struct perception_config;
}} // namespace config::perception

NS_START(controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class dpo_perception_subsystem
 * @ingroup fordyca controller
 *
 * @brief Translates the sensor readings of the robot (i.e. \ref line_of_sight),
 * into a useful internal repr: a \ref dpo_store.
 */
class dpo_perception_subsystem final
    : public rer::client<dpo_perception_subsystem>,
      public base_perception_subsystem,
      public metrics::perception::dpo_perception_metrics {
 public:
  explicit dpo_perception_subsystem(
      const config::perception::perception_config* config);
  ~dpo_perception_subsystem(void) override;

  /* DPO perception metrics */
  uint n_known_blocks(void) const override RCSW_PURE;
  uint n_known_caches(void) const override RCSW_PURE;
  rswarm::pheromone_density avg_block_density(void) const override;
  rswarm::pheromone_density avg_cache_density(void) const override;

  /**
   * @brief Update the robot's perception of the environment, passing it its
   * current line of sight.
   */
  void update(oracular_info_receptor* receptor) override;

  /**
   * @brief Reset the robot's perception of the environment to an initial state
   */
  void reset(void) override;

  const ds::dpo_store* dpo_store(void) const override { return m_store.get(); }
  ds::dpo_store* dpo_store(void) override { return m_store.get(); }

 private:
  /*
   * @brief Update the perceived arena map with the current line-of-sight,
   * update the relevance of information (density) within it (blocks and
   * caches).
   *
   * @param c_los The LOS to process.
   */
  void process_los(const repr::line_of_sight* c_los,
                   oracular_info_receptor* receptor);

  void process_los_blocks(const repr::line_of_sight* c_los);
  void process_los_caches(const repr::line_of_sight* c_los);

  void los_tracking_sync(const repr::line_of_sight* c_los,
                         const ds::cache_list& los_caches);
  void los_tracking_sync(const repr::line_of_sight* c_los,
                         const ds::block_list& blocks);

 private:
  /* clang-format off */
  std::unique_ptr<ds::dpo_store> m_store;
  /* clang-format on */
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DPO_PERCEPTION_SUBSYSTEM_HPP_ */
