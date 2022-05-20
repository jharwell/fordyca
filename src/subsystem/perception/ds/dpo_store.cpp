/**
 * \file dpo_store.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/subsystem/perception/ds/dpo_store.hpp"

#include <boost/range/adaptor/map.hpp>
#include <numeric>

#include "cosm/arena/repr/base_cache.hpp"
#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, subsystem, perception, ds);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dpo_store::dpo_store(const cspconfig::pheromone_config* const config)
    : ER_CLIENT_INIT("fordyca.subsystem.perception.ds.dpo_store"),
      mc_repeat_deposit(config->repeat_deposit),
      mc_pheromone_rho(config->rho) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
model_update_result dpo_store::cache_update(tracked_cache_type&& cache) {
  model_update_result res;

  ER_TRACE("Updating cache%d@%s",
           cache.ent()->id().v(),
           cache.ent()->dcenter2D().to_str().c_str());
  /*
   * If we are currently tracking the cache, we unconditionally remove it,
   * because the # blocks in the cache could have changed since we last saw
   * it, AND its density probably changed too.
   */
  if (cache_remove(cache.ent())) {
    res.reason = model_update_status::ekCACHE_UPDATED;
  } else {
    res.reason = model_update_status::ekNEW_CACHE_ADDED;
  }
  tracked_caches().obj_add({ cache.ent()->dcenter2D(), std::move(cache) });
  return res;
} /* cache_update() */

bool dpo_store::cache_remove(carepr::base_cache* const victim) {
  auto range = tracked_caches().values_range();

  auto it = std::find_if(range.begin(), range.end(), [&](const auto& c) {
    return c.ent()->idcmp(*victim);
  });

  if (it != range.end()) {
    ER_TRACE("Removing cache%d@%s",
             it->ent()->id().v(),
             it->ent()->dcenter2D().to_str().c_str());
    if (1 == tracked_caches().size()) {
      last_cache_loc(it->ent()->rcenter2D());
    }
    tracked_caches().obj_remove(it->ent()->dcenter2D());
    return true;
  }
  return false;
} /* cache_remove() */

model_update_result dpo_store::block_update(tracked_block_type&& block_in) {
  auto range = tracked_blocks().values_range();

  auto it1 = std::find_if(range.begin(),
                          range.end(),
                          [&block_in](const dpo_entity<crepr::sim_block3D>& b) {
                            return b.ent()->idcmp(*block_in.ent());
                          });
  auto it2 = std::find_if(range.begin(),
                          range.end(),
                          [&block_in](const dpo_entity<crepr::sim_block3D>& b) {
                            return b.ent()->dloccmp(*block_in.ent()) &&
                                   !b.ent()->idcmp(*block_in.ent());
                          });

  /*
   * A different block is currently tracked where the new block was seen, and
   * so the old block needs to be removed, as it is out of date information
   * about the arena. This needs to happen regardless of whether the incoming
   * block is known (i.e. its ID matches that of a block in our current block
   * list) or not, in order to avoid transient assert() triggering during LOS
   * processing.
   */
  if (it2 != range.end()) {
    ER_TRACE("Remove old block%d@%s: new block%d found there",
             it2->ent()->id().v(),
             block_in.ent()->danchor2D().to_str().c_str(),
             block_in.ent()->id().v());
    block_remove(it2->ent());
  }

  if (range.end() != it1) { /* block is known */
    ER_TRACE("Known incoming block%d@%s",
             block_in.ent()->id().v(),
             block_in.ent()->danchor2D().to_str().c_str());
    /*
     * Unless a given block's location has changed, there is no need to update
     * the state of the world.
     */
    if (block_in.ent()->danchor2D() != it1->ent()->danchor2D()) {
      ER_TRACE("Block%d has moved: %s -> %s",
               block_in.ent()->id().v(),
               it1->ent()->danchor2D().to_str().c_str(),
               block_in.ent()->danchor2D().to_str().c_str());
      block_remove(it1->ent());

      /*
       * it1 will point to new block after this, so we need to save the old
       * location beforehand.
       */
      rmath::vector2z old_loc = it1->ent()->danchor2D();
      ER_TRACE("Add block%d@%s (n_blocks=%zu)",
               block_in.ent()->id().v(),
               block_in.ent()->danchor2D().to_str().c_str(),
               tracked_blocks().size());
      tracked_blocks().obj_add({ block_in.ent()->id(), std::move(block_in) });

      return { model_update_status::ekBLOCK_MOVED, old_loc };
    }
    /*
     * Even if the block's location has not changed, if we have seen it again we
     * need to update its density.
     */
    dp_block_map::value_type* known = tracked_blocks().find(block_in.ent()->id());
    if (nullptr != known) {
      known->density(block_in.density());
      ER_TRACE("Update density of known block%d@%s to %f",
               block_in.ent()->id().v(),
               block_in.ent()->danchor2D().to_str().c_str(),
               block_in.density().v());
      return { model_update_status::ekBLOCK_DENSITY_UPDATE, rmath::vector2z() };
    }
  } else { /* block is not known */
    ER_TRACE("Unknown incoming block%d", block_in.ent()->id().v());
    ER_TRACE("Add block%d@%s (n_blocks=%zu)",
             block_in.ent()->id().v(),
             block_in.ent()->danchor2D().to_str().c_str(),
             tracked_blocks().size());
    tracked_blocks().obj_add({ block_in.ent()->id(), std::move(block_in) });
    return { model_update_status::ekNEW_BLOCK_ADDED, rmath::vector2z() };
  }
  return { model_update_status::ekNO_CHANGE, rmath::vector2z() };
} /* block_update() */

bool dpo_store::block_remove(crepr::sim_block3D* const victim) {
  auto range = tracked_blocks().values_range();
  auto it = std::find_if(range.begin(), range.end(), [&](const auto& b) {
    return b.ent()->idcmp(*victim);
  });
  if (it != range.end()) {
    ER_TRACE("Removing block%d@%s",
             victim->id().v(),
             victim->danchor2D().to_str().c_str());
    if (1 == tracked_blocks().size()) {
      last_block_loc(it->ent()->ranchor2D());
    }
    tracked_blocks().obj_remove(it->ent()->id());
    return true;
  }
  return false;
} /* block_remove() */

void dpo_store::decay_all(void) {
  tracked_blocks().decay_all();
  tracked_caches().decay_all();
} /* decay_all() */

void dpo_store::clear_all(void) {
  tracked_blocks().clear();
  tracked_caches().clear();
} /* clear_all() */

NS_END(ds, perception, subsystem, fordyca);
