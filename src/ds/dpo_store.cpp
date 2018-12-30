/**
 * @file dpo_store.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/ds/dpo_store.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/representation/base_cache.hpp"
#include "fordyca/params/perception/pheromone_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, ds);
namespace rswarm = rcppsw::swarm;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dpo_store::dpo_store(const params::perception::pheromone_params* const params)
    : ER_CLIENT_INIT("fordyca.ds.dpo_store"),
      mc_repeat_deposit(params->repeat_deposit),
      mc_rho(params->rho) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
dpo_store::update_res_t dpo_store::cache_update(
    const dp_entity<representation::base_cache>& cache) {
  update_res_t res = {
    .status = true,
    .reason = kNoChange,
    .old_loc = rmath::vector2u()
  };

  /*
   * If we are currently tracking the cache, we unconditionally remove it,
   * because the # blocks in the cache could have changed since we last saw
   * it.
   */
  if (cache_remove(cache.ent_obj())) {
    res.reason = kCacheUpdated;
  } else {
    res.reason = kNewCacheAdded;
  }
  m_caches.obj_add(cache);
  return res;
} /* cache_update() */

bool dpo_store::cache_remove(
    const std::shared_ptr<representation::base_cache>& victim) {
  for (auto it = m_caches.begin(); it != m_caches.end(); ++it) {
    if (it->ent_obj() == victim) {
      m_caches.obj_remove(*it);
      return true;
    }
  } /* for(it..) */
  return false;
} /* cache_remove() */

dpo_store::update_res_t dpo_store::block_update(
    const dp_entity<representation::base_block>& block_in) {
  auto it1 = std::find_if(
      m_blocks.begin(),
      m_blocks.end(),
      [&block_in](const dp_entity<representation::base_block>& b) {
        return b.ent()->idcmp(*block_in.ent());
      });
  auto it2 = std::find_if(
      m_blocks.begin(),
      m_blocks.end(),
      [&block_in](const dp_entity<representation::base_block>& b) {
        return b.ent()->loccmp(*block_in.ent()) &&
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
  if (it2 != m_blocks.end()) {
    ER_TRACE("Remove old block%d@%s: new block%d found there",
             it2->ent()->id(),
             block_in.ent()->discrete_loc().to_str().c_str(),
             block_in.ent()->id());
    block_remove(it2->ent_obj());
  }

  if (m_blocks.end() != it1) { /* block is known */
    ER_TRACE("Known incoming block%d@%s",
             block_in.ent()->id(),
             block_in.ent()->discrete_loc().to_str().c_str());
    /*
     * Unless a given block's location has changed, there is no need to update
     * the state of the world.
     */
    if (block_in.ent()->discrete_loc() != it1->ent()->discrete_loc()) {
      ER_TRACE("Block%d has moved: %s -> %s",
               block_in.ent()->id(),
               it1->ent()->discrete_loc().to_str().c_str(),
               block_in.ent()->discrete_loc().to_str().c_str());
      block_remove(it1->ent_obj());

      /*
       * it1 will point to new block after this, so we need to save the old
       * location beforehand.
       */
      rmath::vector2u old_loc = it1->ent()->discrete_loc();
      m_blocks.obj_add(block_in);
      __rcsw_unused int id = block_in.ent()->id();
      ER_TRACE("Add block%d@%s (n_blocks=%zu)",
               id,
               block_in.ent()->discrete_loc().to_str().c_str(),
               m_blocks.size());
      return update_res_t{true, kBlockMoved, old_loc};
    }
    /*
     * Even if the block's location has not changed, if we have seen it again we
     * need to update its density.
     */
    auto known = m_blocks.find(block_in);
    if (nullptr != known) {
        ER_TRACE("Update density of known block%d@%s to %f",
                 block_in.ent()->id(),
                 block_in.ent()->discrete_loc().to_str().c_str(),
                 block_in.density().last_result());
        const_cast<decltype(m_blocks)::value_type*>(known)->density(block_in.density());
      }
  } else { /* block is not known */
    ER_TRACE("Unknown incoming block%d", block_in.ent()->id());
    m_blocks.obj_add(block_in);
    ER_TRACE("Add block%d@%s (n_blocks=%zu)",
             block_in.ent()->id(),
             block_in.ent()->discrete_loc().to_str().c_str(),
             m_blocks.size());
    return {true, kNewBlockAdded, rmath::vector2u()};
  }
  return {false, kNoChange, rmath::vector2u()};
} /* block_update() */

bool dpo_store::block_remove(
    const std::shared_ptr<representation::base_block>& victim) {
  for (auto it = m_blocks.begin(); it != m_blocks.end(); ++it) {
    if (it->ent_obj() == victim) {
      ER_TRACE("Removing block%d@%s",
               victim->id(),
               victim->discrete_loc().to_str().c_str());
      m_blocks.obj_remove(*it);
      return true;
    }
  } /* for(it..) */
  return false;
} /* block_remove() */

void dpo_store::decay_all(void) {
  m_blocks.decay_all();
  m_caches.decay_all();
} /* decay_all() */

void dpo_store::clear_all(void) {
  m_blocks.clear();
  m_caches.clear();
} /* clear_all() */

bool dpo_store::contains(
    const std::shared_ptr<representation::base_block>& block) const {
  return m_blocks.contains(const_dp_block_set::value_type(block,
                                                          rswarm::pheromone_density()));
} /* contains() */

bool dpo_store::contains(
    const std::shared_ptr<representation::base_cache>& cache) const {
  return m_caches.contains(const_dp_cache_set::value_type(cache,
                                                          rswarm::pheromone_density()));
} /* contains() */

const const_dp_block_set::value_type* dpo_store::find(
    const std::shared_ptr<representation::base_block>& block) const {
  return m_blocks.find(const_dp_block_set::value_type(block,
                                                      rswarm::pheromone_density()));
} /* find() */

const const_dp_cache_set::value_type* dpo_store::find(
    const std::shared_ptr<representation::base_cache>& cache) const {
  return m_caches.find(const_dp_cache_set::value_type(cache,
                                                      rswarm::pheromone_density()));
} /* find() */


NS_END(ds, fordyca);
