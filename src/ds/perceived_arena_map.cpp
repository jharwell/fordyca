/**
 * @file perceived_arena_map.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/ds/perceived_arena_map.hpp"

#include "fordyca/events/cell_empty.hpp"
#include "fordyca/params/occupancy_grid_params.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/representation/base_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, ds);
using ds::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
perceived_arena_map::perceived_arena_map(
    const struct fordyca::params::occupancy_grid_params* c_params,
    const std::string& robot_id)
    : ER_CLIENT_INIT("fordyca.ds.perceived_arena_map"),
      decorator(c_params, robot_id),
      m_caches(),
      m_blocks() {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
ds::perceived_block_list perceived_arena_map::perceived_blocks(
    void) const {
  ds::perceived_block_list pblocks;

  for (auto& b : m_blocks) {
    pblocks.push_back(representation::perceived_block(
        b, decoratee().access<occupancy_grid::kPheromone>(b->discrete_loc())));
  } /* for(&b..) */
  return pblocks;
} /* blocks() */

ds::perceived_cache_list perceived_arena_map::perceived_caches(
    void) const {
  ds::perceived_cache_list pcaches;

  for (auto& c : m_caches) {
    pcaches.push_back(representation::perceived_cache(
        c, decoratee().access<occupancy_grid::kPheromone>(c->discrete_loc())));
  } /* for(c..) */
  return pcaches;
} /* caches() */

void perceived_arena_map::cache_add(
    const std::shared_ptr<representation::base_cache>& cache) {
  cache_remove(cache);
  m_caches.push_back(cache);
} /* cache_add() */

void perceived_arena_map::cache_remove(
    const std::shared_ptr<representation::base_cache>& victim) {
  for (auto it = m_caches.begin(); it != m_caches.end(); ++it) {
    if (*(*it) == *victim) {
      events::cell_empty op(victim->discrete_loc());
      decoratee().access<occupancy_grid::kCell>(victim->discrete_loc()).accept(op);
      m_caches.erase(it);
      return;
    }
  } /* for(it..) */
} /* cache_remove() */

bool perceived_arena_map::block_add(
    const std::shared_ptr<representation::base_block>& block_in) {
  auto it1 = std::find_if(
      m_blocks.begin(),
      m_blocks.end(),
      [&block_in](const std::shared_ptr<representation::base_block>& b) {
        return b->id() == block_in->id();
      });
  auto it2 = std::find_if(
      m_blocks.begin(),
      m_blocks.end(),
      [&block_in](const std::shared_ptr<representation::base_block>& b) {
        return b->discrete_loc() == block_in->discrete_loc() &&
               b->id() != block_in->id();
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
             (*it2)->id(),
             block_in->discrete_loc().to_str().c_str(),
             block_in->id());
    block_remove(*it2);
  }

  if (m_blocks.end() != it1) { /* block is known */
    ER_TRACE("Known incoming block%d@%s",
             block_in->id(),
             block_in->discrete_loc().to_str().c_str());
    /*
     * Unless a given block's location has changed, there is no need to update
     * the state of the world.
     */
    if (block_in->discrete_loc() != (*it1)->discrete_loc()) {
      ER_TRACE("Block%d has moved: %s - %s",
               block_in->id(),
               (*it1)->discrete_loc().to_str().c_str(),
               block_in->discrete_loc().to_str().c_str());
      block_remove(*it1);
      m_blocks.push_back(block_in);
      __rcsw_unused int id = block_in->id();
      ER_TRACE("Add block%d@%s (n_blocks=%zu)",
               id,
               block_in->discrete_loc().to_str().c_str(),
               m_blocks.size());
      return true;
    }
  } else { /* block is not known */
    ER_TRACE("Unknown incoming block%d", block_in->id());
    m_blocks.push_back(block_in);
    ER_TRACE("Add block%d@%s (n_blocks=%zu)",
             block_in->id(),
             block_in->discrete_loc().to_str().c_str(),
             m_blocks.size());
    return true;
  }
  return false;
} /* block_add() */

bool perceived_arena_map::block_remove(
    const std::shared_ptr<representation::base_block>& victim) {
  for (auto it = m_blocks.begin(); it != m_blocks.end(); ++it) {
    if (*(*it) == *victim) {
      ER_TRACE("Removing block%d@%s",
               victim->id(),
               victim->discrete_loc().to_str().c_str());
      events::cell_empty op(victim->discrete_loc());
      access<occupancy_grid::kCell>(victim->discrete_loc()).accept(op);
      m_blocks.erase(it);
      return true;
    }
  } /* for(it..) */
  return false;
} /* block_remove() */

NS_END(ds, fordyca);
