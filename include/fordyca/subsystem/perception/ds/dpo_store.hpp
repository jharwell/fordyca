/**
 * \file dpo_store.hpp
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

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/subsystem/perception/config/pheromone_config.hpp"

#include "fordyca/subsystem/perception/ds/dp_block_map.hpp"
#include "fordyca/subsystem/perception/ds/dp_cache_map.hpp"
#include "fordyca/subsystem/perception/ds/dpo_map.hpp"
#include "fordyca/subsystem/perception/foraging_memory_model.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, subsystem, perception, ds);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dpo_store
 * \ingroup subsystem perception ds
 *
 * \brief Stores sets of caches and blocks the robot encounters in the arena,
 * along with the pheromone-based relevance for each. Also contains the logic
 * for correctly updating the sets of caches and blocks when updated versions of
 * an entity are encountered (e.g. the real version has vanished and the tracked
 * version is out of date).
 */
class dpo_store final : public foraging_memory_model<ds::dp_block_map,
                                                         ds::dp_cache_map>,
                        public rer::client<dpo_store> {
 public:
  template <typename T>
  using dpo_entity = repr::dpo_entity<T>;

  using base_model_type = foraging_memory_model<ds::dp_block_map,
                                                    ds::dp_cache_map>;
  using base_model_type::tracked_cache_type;
  using base_model_type::tracked_block_type;

  /**
   * \brief The maximum pheromone level if repeat deposits of pheromone are not
   * enabled.
   */
  static constexpr double kNRD_MAX_PHEROMONE = 1.0;

  explicit dpo_store(const cspconfig::pheromone_config* config);

  /* access_known_objects overrides */
  cds::block3D_vectorno known_blocks(void) const override {
    return dp_block_map::raw_values_extract<cds::block3D_vectorno>(tracked_blocks());
  }
  cads::bcache_vectorno known_caches(void) const override {
    return dp_cache_map::raw_values_extract<cads::bcache_vectorno>(tracked_caches());
  }

  /* foraging_memory_model overrides */
  bool cache_remove(carepr::base_cache* victim) override;
  bool block_remove(crepr::sim_block3D* victim) override;
  model_update_result cache_update(tracked_cache_type&& cache) override;

  /*
   * \brief Update the known blocks set with the new block.
   *
   * If the the new block's location has changed: tracked block is replaced
   * If the new blocks' location has not changed from the tracked block: don't
   * replace it, but update its pheromone level to correspond to the new block.
   *
   * \return \c TRUE if a block was added, and \c FALSE otherwise.
   */
  model_update_result block_update(tracked_block_type&& block) override;

  bool repeat_deposit(void) const { return mc_repeat_deposit; }

  /**
   * \brief Update the densities of all objects in the store (i.e. decay
   * them). Should be called when one unit of time has passed (e.g. every
   * timestep).
   */
  void decay_all(void);
  void clear_all(void);

  double pheromone_rho(void) const { return mc_pheromone_rho; }

 private:
  /* clang-format off */
  const bool   mc_repeat_deposit;
  const double mc_pheromone_rho;
  /* clang-format on */
};

NS_END(ds, perception, subsystem, fordyca);

