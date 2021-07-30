/**
 * \file dpo_perception_subsystem.hpp
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

#ifndef INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_DPO_PERCEPTION_SUBSYSTEM_HPP_
#define INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_DPO_PERCEPTION_SUBSYSTEM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>

#include "rcppsw/er/client.hpp"

#include "cosm/ds/entity_vector.hpp"

#include "fordyca/subsystem/perception/foraging_perception_subsystem.hpp"
#include "fordyca/fordyca.hpp"
#include "fordyca/metrics/perception/dpo_metrics.hpp"
#include "fordyca/subsystem/perception/perception_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, subsystem, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dpo_perception_subsystem
 * \ingroup subsystem perception
 *
 * \brief Translates the sensor readings of the robot (i.e. \ref forager_los),
 * into a useful internal repr: a \ref dpo_store.
 */
class dpo_perception_subsystem final
    : public rer::client<dpo_perception_subsystem>,
      public foraging_perception_subsystem,
      public metrics::perception::dpo_metrics {
 public:
  explicit dpo_perception_subsystem(const cspconfig::perception_config* config);
  ~dpo_perception_subsystem(void) override;

  /* DPO perception metrics */
  size_t n_known_blocks(void) const override RCPPSW_PURE;
  size_t n_known_caches(void) const override RCPPSW_PURE;
  crepr::pheromone_density avg_block_density(void) const override;
  crepr::pheromone_density avg_cache_density(void) const override;

  /* foraging_perception_subsystem overrides */
  const known_objects_accessor* known_objects(void) const override;
  void update(oracular_info_receptor* receptor) override;

  /**
   * \brief Reset the robot's perception of the environment to an initial state
   */
  void reset(void) override;

 private:
  /*
   * \brief Update the perceived arena map with the current line-of-sight,
   * update the relevance of information (density) within it (blocks and
   * caches).
   *
   * \param c_los The LOS to process.
   */
  void process_los(const repr::forager_los* c_los,
                   oracular_info_receptor* receptor);

  void process_los_blocks(const repr::forager_los* c_los);
  void process_los_caches(const repr::forager_los* c_los);

  void los_tracking_sync(const repr::forager_los* c_los,
                         const cads::bcache_vectorno& los_caches);
  void los_tracking_sync(const repr::forager_los* c_los,
                         const cds::block3D_vectorno& los_blocks);

  const ds::dpo_store* store(void) const;
  ds::dpo_store* store(void);
};

NS_END(cognitive, controller, fordyca);

#endif /* INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_DPO_PERCEPTION_SUBSYSTEM_HPP_ */
