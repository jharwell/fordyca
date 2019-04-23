/**
 * @file dpo_perception_subsystem.cpp
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
#include "fordyca/controller/dpo_perception_subsystem.hpp"
#include "fordyca/controller/los_proc_verify.hpp"
#include "fordyca/ds/dpo_store.hpp"
#include "fordyca/events/block_found.hpp"
#include "fordyca/events/cache_found.hpp"
#include "fordyca/params/perception/perception_params.hpp"
#include "fordyca/repr/base_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dpo_perception_subsystem::dpo_perception_subsystem(
    const struct params::perception::perception_params* const params)
    : ER_CLIENT_INIT("fordyca.controller.dpo_perception"),
      m_store(rcppsw::make_unique<ds::dpo_store>(&params->pheromone)) {}

dpo_perception_subsystem::~dpo_perception_subsystem(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void dpo_perception_subsystem::update(void) {
  process_los(los());
  ER_ASSERT(los_proc_verify(los())(dpo_store()), "LOS verification failed");
  m_store->decay_all();
} /* update() */

void dpo_perception_subsystem::reset(void) { m_store->clear_all(); }

void dpo_perception_subsystem::process_los(
    const repr::line_of_sight* const c_los) {
  ER_TRACE("LOS LL=%s, LR=%s, UL=%s UR=%s",
           c_los->abs_ll().to_str().c_str(),
           c_los->abs_lr().to_str().c_str(),
           c_los->abs_ul().to_str().c_str(),
           c_los->abs_ur().to_str().c_str());

  process_los_blocks(c_los);
  process_los_caches(c_los);
} /* process_los() */

void dpo_perception_subsystem::process_los_caches(
    const repr::line_of_sight* const c_los) {
  ds::cache_list los_caches = c_los->caches();
  if (!los_caches.empty()) {
    ER_DEBUG("Caches in LOS: [%s]", rcppsw::to_string(los_caches).c_str());
    ER_DEBUG("Caches in DPO store: [%s]",
             rcppsw::to_string(m_store->caches()).c_str());
  }

  /* Fix our tracking of caches that no longer exist in our perception */
  los_tracking_sync(c_los, los_caches);

  for (auto& cache : los_caches) {
    if (!m_store->contains(cache)) {
      ER_INFO("Discovered Cache%d@%s/%s",
              cache->id(),
              cache->real_loc().to_str().c_str(),
              cache->discrete_loc().to_str().c_str());
    } else if (cache->n_blocks() != m_store->find(cache)->ent()->n_blocks()) {
      ER_INFO("Update cache%d@%s blocks: %zu -> %zu",
              cache->id(),
              cache->discrete_loc().to_str().c_str(),
              cache->n_blocks(),
              m_store->find(cache)->ent()->n_blocks());
    }
    /*
     * The cache we get a handle to is owned by the simulation, and we don't
     * want to just pass that into the robot's arena_map, as keeping them in
     * sync is not possible in all situations.
     *
     * For example, if a block executing the collector task picks up a block
     * and tries to compute the best cache to bring it to, only to have one or
     * more of its cache references be invalid due to other robots causing
     * caches to be created/destroyed.
     *
     * Cloning is definitely necessary here.
     */
    events::cache_found_visitor op(cache->clone());
    op.visit(*m_store);
  } /* for(cache..) */
} /* process_los_caches() */

void dpo_perception_subsystem::process_los_blocks(
    const repr::line_of_sight* const c_los) {
  /*
   * Because this is computed, rather than a returned reference to a member
   * variable, we can't use separate begin()/end() calls with it, and need to
   * explicitly assign it.
   */
  ds::block_list los_blocks = c_los->blocks();
  if (!los_blocks.empty()) {
    ER_DEBUG("Blocks in LOS: [%s]", rcppsw::to_string(los_blocks).c_str());
    ER_DEBUG("Blocks in DPO store: [%s]",
             rcppsw::to_string(m_store->blocks()).c_str());
  }

  /*
   * Fix our tracking of blocks that no longer exist in our perception/have
   * moved.
   */
  los_tracking_sync(c_los, los_blocks);

  for (auto&& block : c_los->blocks()) {
    ER_ASSERT(!block->is_out_of_sight(),
              "Block%d out of sight in LOS?",
              block->id());

    if (!m_store->contains(block)) {
      ER_INFO("Discovered block%d@%s/%s",
              block->id(),
              block->real_loc().to_str().c_str(),
              block->discrete_loc().to_str().c_str());
    } else {
      ER_DEBUG("Block%d@%s/%s already known",
               block->id(),
               block->real_loc().to_str().c_str(),
               block->discrete_loc().to_str().c_str());
    }
    events::block_found_visitor op(block->clone());
    op.visit(*m_store);
  } /* for(block..) */
} /* process_los() */

void dpo_perception_subsystem::los_tracking_sync(
    const repr::line_of_sight* const c_los,
    const ds::cache_list& los_caches) {
  /*
   * If the location of one of the caches we are tracking is in our LOS, then
   * the corresponding cache should also be in our LOS. If it is not, then our
   * tracked version is out of date and needs to be removed.
   */
  for (auto& cache : m_store->caches().const_values_range()) {
    if (c_los->contains_loc(cache.ent()->discrete_loc())) {
      auto it =
          std::find_if(los_caches.begin(), los_caches.end(), [&](const auto& c) {
            return c->loccmp(*cache.ent_obj());
          });

      if (los_caches.end() == it) {
        ER_INFO("Remove tracked cache%d@%s: not in LOS caches",
                cache.ent()->id(),
                cache.ent()->discrete_loc().to_str().c_str());
        m_store->cache_remove(cache.ent_obj());
        ER_ASSERT(nullptr == m_store->find(cache.ent_obj()),
                  "Cache%d still exists in store after removal",
                  cache.ent()->id());
      }
    }
  } /* for(&&block..) */
} /* los_tracking_sync() */

void dpo_perception_subsystem::los_tracking_sync(
    const repr::line_of_sight* const c_los,
    const ds::block_list& blocks) {
  /*
   * If the location of one of the blocks we are tracking is in our LOS, then
   * the corresponding block should also be in our LOS. If it is not, then our
   * tracked version is out of date and needs to be removed.
   *
   * This logic does NOT handle cases where the tracked block is in our LOS, but
   * has moved since we last saw it (since that is limited to at most a single
   * block, it is handled by the \ref block_found event).
   */
  for (auto&& block : m_store->blocks().const_values_range()) {
    if (c_los->contains_loc(block.ent()->discrete_loc())) {
      auto it = std::find_if(blocks.begin(), blocks.end(), [&](const auto& b) {
        return b->idcmp(*block.ent_obj());
      });
      if (blocks.end() == it) {
        /*
         * Needed for assert() to prevent last reference to shared_ptr being
         * removed and object destruction.
         */
        auto tmp = block;
        m_store->block_remove(tmp.ent_obj());
        ER_ASSERT(nullptr == m_store->find(tmp.ent_obj()),
                  "Block%d still exists in store after removal",
                  tmp.ent()->id());
      }
    }
  } /* for(&&block..) */
} /* los_tracking_sync() */

/*******************************************************************************
 * DPO Perception Metrics
 ******************************************************************************/
__rcsw_pure uint dpo_perception_subsystem::n_known_blocks(void) const {
  return m_store->blocks().size();
} /* n_known_blocks() */

__rcsw_pure uint dpo_perception_subsystem::n_known_caches(void) const {
  return m_store->caches().size();
} /* n_known_caches() */

rswarm::pheromone_density dpo_perception_subsystem::avg_block_density(void) const {
  auto range = m_store->blocks().const_values_range();
  if (m_store->blocks().empty()) {
    return rswarm::pheromone_density();
  }
  return std::accumulate(range.begin(),
                         range.end(),
                         rswarm::pheromone_density(),
                         [&](const auto& accum, const auto& block) {
                           return accum + block.density();
                         }) /
         m_store->blocks().size();
} /* avg_block_density() */

rswarm::pheromone_density dpo_perception_subsystem::avg_cache_density(void) const {
  auto range = m_store->caches().const_values_range();
  if (m_store->caches().empty()) {
    return rswarm::pheromone_density();
  }
  return std::accumulate(range.begin(),
                         range.end(),
                         rswarm::pheromone_density(),
                         [&](const auto& accum, const auto& cache) {
                           return accum + cache.density();
                         }) /
         m_store->caches().size();
} /* avg_cache_density() */

NS_END(controller, fordyca);
