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
#include "fordyca/ds/dpo_store.hpp"

#include <boost/range/adaptor/map.hpp>
#include <numeric>

#include "cosm/arena/repr/base_cache.hpp"
#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, ds);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dpo_store::dpo_store(const ccontconfig::perception::pheromone_config* const config)
    : ER_CLIENT_INIT("fordyca.ds.dpo_store"),
      mc_repeat_deposit(config->repeat_deposit),
      mc_pheromone_rho(config->rho) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
dpo_store::update_res_t dpo_store::cache_update(
    dpo_entity<carepr::base_cache> cache) {
  update_res_t res = {.status = true,
                      .reason = ekNO_CHANGE,
                      .old_loc = rmath::vector2z()};
  ER_TRACE("Updating cache%d@%s",
           cache.ent()->id().v(),
           cache.ent()->dpos2D().to_str().c_str());
  /*
   * If we are currently tracking the cache, we unconditionally remove it,
   * because the # blocks in the cache could have changed since we last saw
   * it.
   */
  if (cache_remove(cache.ent())) {
    res.reason = ekCACHE_UPDATED;
  } else {
    res.reason = ekNEW_CACHE_ADDED;
  }
  m_caches.obj_add({cache.ent()->dpos2D(), std::move(cache)});
  return res;
} /* cache_update() */

bool dpo_store::cache_remove(carepr::base_cache* const victim) {
  auto range = m_caches.const_values_range();

  auto it = std::find_if(range.begin(), range.end(), [&](const auto& c) {
    return c.ent()->idcmp(*victim);
  });

  if (it != range.end()) {
    ER_TRACE("Removing cache%d@%s",
             it->ent()->id().v(),
             it->ent()->dpos2D().to_str().c_str());
    if (1 == m_caches.size()) {
      m_last_cache_loc = boost::make_optional(it->ent()->rpos2D());
    }
    m_caches.obj_remove(it->ent()->dpos2D());
    return true;
  }
  return false;
} /* cache_remove() */

dpo_store::update_res_t dpo_store::block_update(
    dpo_entity<crepr::base_block3D> block_in) {
  auto range = m_blocks.values_range();

  auto it1 = std::find_if(range.begin(),
                          range.end(),
                          [&block_in](const dpo_entity<crepr::base_block3D>& b) {
                            return b.ent()->idcmp(*block_in.ent());
                          });
  auto it2 = std::find_if(range.begin(),
                          range.end(),
                          [&block_in](const dpo_entity<crepr::base_block3D>& b) {
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
             block_in.ent()->dpos2D().to_str().c_str(),
             block_in.ent()->id().v());
    block_remove(it2->ent());
  }

  if (range.end() != it1) { /* block is known */
    ER_TRACE("Known incoming block%d@%s",
             block_in.ent()->id().v(),
             block_in.ent()->dpos2D().to_str().c_str());
    /*
     * Unless a given block's location has changed, there is no need to update
     * the state of the world.
     */
    if (block_in.ent()->dpos2D() != it1->ent()->dpos2D()) {
      ER_TRACE("Block%d has moved: %s -> %s",
               block_in.ent()->id().v(),
               it1->ent()->dpos2D().to_str().c_str(),
               block_in.ent()->dpos2D().to_str().c_str());
      block_remove(it1->ent());

      /*
       * it1 will point to new block after this, so we need to save the old
       * location beforehand.
       */
      rmath::vector2z old_loc = it1->ent()->dpos2D();
      m_blocks.obj_add({block_in.ent()->id(), std::move(block_in)});
      RCSW_UNUSED rtypes::type_uuid id = block_in.ent()->id();
      ER_TRACE("Add block%d@%s (n_blocks=%zu)",
               id.v(),
               block_in.ent()->dpos2D().to_str().c_str(),
               m_blocks.size());
      return update_res_t{true, ekBLOCK_MOVED, old_loc};
    }
    /*
     * Even if the block's location has not changed, if we have seen it again we
     * need to update its density.
     */
    dp_block_map::value_type* known = m_blocks.find(block_in.ent()->id());
    if (nullptr != known) {
      known->density(block_in.density());
      ER_TRACE("Update density of known block%d@%s to %f",
               block_in.ent()->id().v(),
               block_in.ent()->dpos2D().to_str().c_str(),
               block_in.density().v());
    }
  } else { /* block is not known */
    ER_TRACE("Unknown incoming block%d", block_in.ent()->id().v());
    m_blocks.obj_add({block_in.ent()->id(), std::move(block_in)});
    ER_TRACE("Add block%d@%s (n_blocks=%zu)",
             block_in.ent()->id().v(),
             block_in.ent()->dpos2D().to_str().c_str(),
             m_blocks.size());
    return {true, ekNEW_BLOCK_ADDED, rmath::vector2z()};
  }
  return {false, ekNO_CHANGE, rmath::vector2z()};
} /* block_update() */

bool dpo_store::block_remove(crepr::base_block3D* const victim) {
  auto range = m_blocks.const_values_range();
  auto it = std::find_if(range.begin(), range.end(), [&](const auto& b) {
    return b.ent()->idcmp(*victim);
  });
  if (it != range.end()) {
    ER_TRACE("Removing block%d@%s",
             victim->id().v(),
             victim->dpos2D().to_str().c_str());
    if (1 == m_blocks.size()) {
      m_last_block_loc = boost::make_optional(it->ent()->rpos2D());
    }
    m_blocks.obj_remove(it->ent()->id());
    return true;
  }
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

bool dpo_store::contains(const crepr::base_block3D* const block) const {
  return m_blocks.contains(block->id());
} /* contains() */

bool dpo_store::contains(const carepr::base_cache* const cache) const {
  return m_caches.contains(cache->dpos2D());
} /* contains() */

const dp_block_map::value_type* dpo_store::find(
    const crepr::base_block3D* const block) const {
  return m_blocks.find(block->id());
} /* find() */

dp_block_map::value_type* dpo_store::find(const crepr::base_block3D* const block) {
  return m_blocks.find(block->id());
} /* find() */

const dp_cache_map::value_type* dpo_store::find(
    const carepr::base_cache* const cache) const {
  return m_caches.find(cache->dpos2D());
} /* find() */

dp_cache_map::value_type* dpo_store::find(const carepr::base_cache* const cache) {
  return m_caches.find(cache->dpos2D());
} /* find() */

NS_END(ds, fordyca);
