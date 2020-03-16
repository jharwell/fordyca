/**
 * \file mdpo_perception_subsystem.cpp
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
#include "fordyca/controller/mdpo_perception_subsystem.hpp"

#include <algorithm>

#include "cosm/ds/cell2D.hpp"
#include "cosm/foraging/repr/base_cache.hpp"
#include "cosm/fsm/cell2D_state.hpp"
#include "cosm/repr/base_block2D.hpp"

#include "fordyca/controller/los_proc_verify.hpp"
#include "fordyca/controller/oracular_info_receptor.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/events/block_found.hpp"
#include "fordyca/events/cache_found.hpp"
#include "fordyca/events/cell2D_empty.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
using ds::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
mdpo_perception_subsystem::mdpo_perception_subsystem(
    const config::perception::perception_config* const config,
    const std::string& id)
    : ER_CLIENT_INIT("fordyca.controller.mdpo_perception"),
      base_perception_subsystem(config),
      m_cell_stats(cfsm::cell2D_state::ekST_MAX_STATES),
      m_los(),
      m_map(std::make_unique<ds::dpo_semantic_map>(config, id)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void mdpo_perception_subsystem::update(oracular_info_receptor* const receptor) {
  update_cell_stats(los());
  process_los(los(), receptor);
  ER_ASSERT(los_proc_verify(los())(map()), "LOS verification failed");
  m_map->decay_all();
} /* update() */

void mdpo_perception_subsystem::reset(void) { m_map->reset(); }

void mdpo_perception_subsystem::process_los(
    const repr::line_of_sight* const c_los,
    oracular_info_receptor* const receptor) {
  ER_TRACE("LOS LL=%s, LR=%s, UL=%s UR=%s",
           c_los->abs_ll().to_str().c_str(),
           c_los->abs_lr().to_str().c_str(),
           c_los->abs_ul().to_str().c_str(),
           c_los->abs_ur().to_str().c_str());

  /* If we are in an oracular controller, process the updates from the oracle */
  if (nullptr != receptor) {
    receptor->dpo_store_update(dpo_store());
  }

  /*
   * Depending on oracle configuration, we may be able to skip processing parts
   * of our LOS, as they will be a subset of the updates we get from the oracle.
   */
  if (nullptr == receptor ||
      (nullptr != receptor && !receptor->entities_blocks_enabled())) {
    process_los_blocks(c_los);
  }
  if (nullptr == receptor ||
      (nullptr != receptor && !receptor->entities_caches_enabled())) {
    process_los_caches(c_los);
  }
} /* process_los() */

void mdpo_perception_subsystem::process_los_blocks(
    const repr::line_of_sight* const c_los) {
  /*
   * Because this is computed, rather than a returned reference to a member
   * variable, we can't use separate begin()/end() calls with it, and need to
   * explicitly assign it.
   */
  cds::block2D_vectorno blocks = c_los->blocks();
  if (!blocks.empty()) {
    ER_DEBUG("Blocks in LOS: [%s]", rcppsw::to_string(blocks).c_str());
    ER_DEBUG("Blocks in DPO store: [%s]",
             rcppsw::to_string(m_map->store()->blocks()).c_str());
  }

  /*
   * If the robot thinks that a cell contains a block, because the cell had one
   * the last time it passed nearby, but when coming near the cell a second time
   * the cell does not contain a block, then someone else picked up the block
   * between then and now, and it needs to update its internal repr
   * accordingly.
   */
  for (uint i = 0; i < c_los->xsize(); ++i) {
    for (uint j = 0; j < c_los->ysize(); ++j) {
      rmath::vector2u d = c_los->cell(i, j).loc();
      if (!c_los->cell(i, j).state_has_block() &&
          m_map->access<occupancy_grid::kCell>(d).state_has_block()) {
        auto block = m_map->access<occupancy_grid::kCell>(d).block();
        ER_DEBUG("Correct block%d %s/%s discrepency",
                 block->id().v(),
                 block->rloc().to_str().c_str(),
                 block->dloc().to_str().c_str());
        m_map->block_remove(block);
      } else if (c_los->cell(i, j).state_is_known() &&
                 !m_map->access<occupancy_grid::kCell>(d).state_is_known()) {
        ER_TRACE("Cell@%s now known to be empty", d.to_str().c_str());
        events::cell2D_empty_visitor e(d);
        e.visit(*m_map);
      }
    } /* for(j..) */
  }   /* for(i..) */

  for (auto& block : c_los->blocks()) {
    ER_ASSERT(!block->is_out_of_sight(),
              "Block%d out of sight in LOS?",
              block->id().v());
    auto& cell = m_map->access<occupancy_grid::kCell>(block->dloc());
    if (!cell.state_has_block()) {
      ER_INFO("Discovered block%d@%s/%s",
              block->id().v(),
              block->rloc().to_str().c_str(),
              block->dloc().to_str().c_str());
    } else if (cell.state_has_block()) {
      ER_DEBUG("Block%d@%s/%s already known",
               block->id().v(),
               block->rloc().to_str().c_str(),
               block->dloc().to_str().c_str());
      auto range = m_map->blocks().const_values_range();
      auto it = std::find_if(range.begin(), range.end(), [&](const auto& b) {
        return b.ent()->id() == cell.block()->id();
      });
      ER_ASSERT(it != range.end(), "Known block%d not in PAM", block->id().v());
    }
    events::block_found_visitor op(block);
    op.visit(*m_map);
  } /* for(block..) */
} /* process_los_blocks() */

void mdpo_perception_subsystem::process_los_caches(
    const repr::line_of_sight* const c_los) {
  /*
   * Because this is computed, rather than a returned reference to a member
   * variable, we can't use separate begin()/end() calls with it, and need to
   * explicitly assign it.
   */
  cfds::bcache_vectorno los_caches = c_los->caches();
  if (!los_caches.empty()) {
    ER_DEBUG("Caches in LOS: [%s]", rcppsw::to_string(los_caches).c_str());
    ER_DEBUG("Caches in DPO store: [%s]",
             rcppsw::to_string(m_map->store()->caches()).c_str());
  }

  /*
   * If the robot thinks that a cell contains a cache, because the cell had one
   * the last time it passed nearby, but when coming near the cell a second time
   * the cell does not contain a cache, then the cache was depleted between then
   * and now, and it needs to update its internal repr accordingly.
   */
  for (uint i = 0; i < c_los->xsize(); ++i) {
    for (uint j = 0; j < c_los->ysize(); ++j) {
      rmath::vector2u d = c_los->cell(i, j).loc();
      if (!c_los->cell(i, j).state_has_cache() &&
          map()->access<occupancy_grid::kCell>(d).state_has_cache()) {
        auto cache = map()->access<occupancy_grid::kCell>(d).cache();
        ER_DEBUG("Correct cache%d@%s/%s discrepency",
                 cache->id().v(),
                 cache->rloc().to_str().c_str(),
                 cache->dloc().to_str().c_str());
        map()->cache_remove(cache);
      }
    } /* for(j..) */
  }   /* for(i..) */

  for (auto& cache : los_caches) {
    /*
     * The state of a cache can change between when the robot saw it last
     * (i.e. different # of blocks in it), and so you need to always process
     * caches in the LOS, even if you already know about them.
     */
    ER_DEBUG("LOS: Cache%d@%s/%s: %zu blocks",
             cache->id().v(),
             cache->rloc().to_str().c_str(),
             cache->dloc().to_str().c_str(),
             cache->n_blocks());
    auto& cell = map()->access<occupancy_grid::kCell>(cache->dloc());

    if (!cell.state_has_cache()) {
      ER_INFO("Discovered cache%d@%s/%s: %zu blocks",
              cache->id().v(),
              cache->rloc().to_str().c_str(),
              cache->dloc().to_str().c_str(),
              cache->n_blocks());
    } else if (cell.state_has_cache() &&
               cell.cache()->n_blocks() != cache->n_blocks()) {
      ER_INFO("Fixed cache%d@%s/%s block count: %zu -> %zu",
              cache->id().v(),
              cache->rloc().to_str().c_str(),
              cache->dloc().to_str().c_str(),
              cache->n_blocks(),
              cell.cache()->n_blocks());
    }
    events::cache_found_visitor op(cache);
    op.visit(*m_map);
  } /* for(cache..) */
} /* process_los_caches() */

void mdpo_perception_subsystem::update_cell_stats(
    const repr::line_of_sight* const c_los) {
  for (uint i = 0; i < c_los->xsize(); ++i) {
    for (uint j = 0; j < c_los->ysize(); ++j) {
      rmath::vector2u d = c_los->cell(i, j).loc();
      if (c_los->cell(i, j).state_is_empty() &&
          m_map->access<occupancy_grid::kCell>(d).state_is_known() &&
          !m_map->access<occupancy_grid::kCell>(d).state_is_empty()) {
        m_cell_stats[cfsm::cell2D_state::ekST_EMPTY]++;
      } else if (c_los->cell(i, j).state_has_block() &&
                 m_map->access<occupancy_grid::kCell>(d).state_is_known() &&
                 !m_map->access<occupancy_grid::kCell>(d).state_has_block()) {
        m_cell_stats[cfsm::cell2D_state::ekST_HAS_BLOCK]++;
      } else if (c_los->cell(i, j).state_has_cache() &&
                 m_map->access<occupancy_grid::kCell>(d).state_is_known() &&
                 !m_map->access<occupancy_grid::kCell>(d).state_has_cache()) {
        m_cell_stats[cfsm::cell2D_state::ekST_HAS_CACHE]++;
      }
    } /* for(j..) */
  }   /* for(i..) */
} /* update_cell_stats() */

ds::dpo_store* mdpo_perception_subsystem::dpo_store(void) {
  return m_map->store();
} /* dpo_store() */

const ds::dpo_store* mdpo_perception_subsystem::dpo_store(void) const {
  return m_map->store();
} /* dpo_store() */

/*******************************************************************************
 * MDPO Perception Metrics
 ******************************************************************************/
double mdpo_perception_subsystem::known_percentage(void) const {
  return m_map->known_cell_count() /
         static_cast<double>(m_map->xdsize() * m_map->ydsize());
} /* known_percentage() */

double mdpo_perception_subsystem::unknown_percentage(void) const {
  return 1.0 - known_percentage();
} /* unknown_percentage() */

void mdpo_perception_subsystem::reset_metrics(void) {
  m_cell_stats.assign(m_cell_stats.size(), 0);
} /* reset_metrics() */

NS_END(controller, fordyca);
