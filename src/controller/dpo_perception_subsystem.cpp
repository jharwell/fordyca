/**
 * \file dpo_perception_subsystem.cpp
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
#include "fordyca/controller/dpo_perception_subsystem.hpp"

#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/arena/repr/base_cache.hpp"

#include "fordyca/controller/los_proc_verify.hpp"
#include "fordyca/controller/oracular_info_receptor.hpp"
#include "fordyca/ds/dpo_store.hpp"
#include "fordyca/events/block_found.hpp"
#include "fordyca/events/cache_found.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dpo_perception_subsystem::dpo_perception_subsystem(
    const cspconfig::perception_config* const config)
    : ER_CLIENT_INIT("fordyca.controller.dpo_perception"),
      foraging_perception_subsystem(config),
      m_store(std::make_unique<ds::dpo_store>(&config->pheromone)) {}

dpo_perception_subsystem::~dpo_perception_subsystem(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void dpo_perception_subsystem::update(oracular_info_receptor* const receptor) {
  process_los(los(), receptor);
  ER_ASSERT(los_proc_verify(los())(dpo_store()), "LOS verification failed");
  m_store->decay_all();
} /* update() */

void dpo_perception_subsystem::reset(void) { m_store->clear_all(); }

void dpo_perception_subsystem::process_los(
    const repr::forager_los* const c_los,
    oracular_info_receptor* const receptor) {
  ER_TRACE("LOS LL=%s, LR=%s, UL=%s UR=%s",
           c_los->abs_ll().to_str().c_str(),
           c_los->abs_lr().to_str().c_str(),
           c_los->abs_ul().to_str().c_str(),
           c_los->abs_ur().to_str().c_str());

  /* If we are in an oracular controller, process the updates from the oracle */
  if (nullptr != receptor) {
    receptor->dpo_store_update(m_store.get());
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

void dpo_perception_subsystem::process_los_caches(
    const repr::forager_los* const c_los) {
  cads::bcache_vectorno los_caches = c_los->caches();
  ER_DEBUG("Caches in DPO store: [%s]",
           rcppsw::to_string(m_store->caches()).c_str());
  if (!los_caches.empty()) {
    ER_DEBUG("Caches in LOS: [%s]", rcppsw::to_string(los_caches).c_str());
  }

  /* Fix our tracking of caches that no longer exist in our perception */
  los_tracking_sync(c_los, los_caches);

  for (auto& cache : los_caches) {
    if (!m_store->contains(cache)) {
      ER_INFO("Discovered Cache%d@%s/%s",
              cache->id().v(),
              rcppsw::to_string(cache->rcenter2D()).c_str(),
              rcppsw::to_string(cache->dcenter2D()).c_str());
    } else if (cache->n_blocks() != m_store->find(cache)->ent()->n_blocks()) {
      ER_INFO("Update cache%d@%s blocks: %zu -> %zu",
              cache->id().v(),
              rcppsw::to_string(cache->dcenter2D()).c_str(),
              m_store->find(cache)->ent()->n_blocks(),
              cache->n_blocks());
    }
    events::cache_found_visitor op(cache);
    op.visit(*m_store);
  } /* for(cache..) */
} /* process_los_caches() */

void dpo_perception_subsystem::process_los_blocks(
    const repr::forager_los* const c_los) {
  /*
   * Because this is computed, rather than a returned reference to a member
   * variable, we can't use separate begin()/end() calls with it, and need to
   * explicitly assign it.
   */
  auto los_blocks = c_los->blocks();
  if (!los_blocks.empty()) {
    ER_DEBUG("Blocks in DPO store: [%s]",
             rcppsw::to_string(m_store->blocks()).c_str());
    auto accum =
        std::accumulate(los_blocks.begin(),
                        los_blocks.end(),
                        std::string(),
                        [&](const std::string& a, const auto& b) {
                          return a + "b" + rcppsw::to_string(b->id()) + ",";
                        });
  }

  /*
   * Fix our tracking of blocks that no longer exist in our perception/have
   * moved.
   */
  los_tracking_sync(c_los, los_blocks);

  for (auto* block : c_los->blocks()) {
    ER_ASSERT(!block->is_out_of_sight(),
              "Block%d@%s/%s out of sight in LOS?",
              block->id().v(),
              rcppsw::to_string(block->ranchor2D()).c_str(),
              rcppsw::to_string(block->danchor2D()).c_str());

    if (!m_store->contains(block)) {
      ER_INFO("Discovered block%d@%s/%s",
              block->id().v(),
              rcppsw::to_string(block->ranchor2D()).c_str(),
              rcppsw::to_string(block->danchor2D()).c_str());
    } else {
      ER_DEBUG("Block%d@%s/%s already known",
               block->id().v(),
               rcppsw::to_string(block->ranchor2D()).c_str(),
               rcppsw::to_string(block->danchor2D()).c_str());
    }
    events::block_found_visitor op(block);
    op.visit(*m_store);
  } /* for(block..) */
} /* process_los() */

void dpo_perception_subsystem::los_tracking_sync(
    const repr::forager_los* const c_los,
    const cads::bcache_vectorno& los_caches) {
  /*
   * If the location of one of the caches we are tracking is in our LOS, then
   * the corresponding cache should also be in our LOS. If it is not, then our
   * tracked version is out of date and needs to be removed.
   */
  auto range = m_store->caches().values_range();
  auto it = range.begin();

  while (it != range.end()) {
    ER_TRACE("Check tracked DPO cache%d@%s/%s xspan=%s,yspan=%s w/LOS xspan=%s,yspan=%s",
             it->ent()->id().v(),
             rcppsw::to_string(it->ent()->rcenter2D()).c_str(),
             rcppsw::to_string(it->ent()->dcenter2D()).c_str(),
             rcppsw::to_string(it->ent()->xrspan()).c_str(),
             rcppsw::to_string(it->ent()->yrspan()).c_str(),
             rcppsw::to_string(c_los->xspan()).c_str(),
             rcppsw::to_string(c_los->yspan()).c_str());

    /*
     * We can't just check if the cache host cell is in our LOS, because for
     * bigger arenas, it almost never is.
     */
    bool should_be_in_los = it->ent()->yrspan().overlaps_with(c_los->yspan()) ||
                            it->ent()->xrspan().overlaps_with(c_los->xspan());

    bool in_los =
        los_caches.end() !=
        std::find_if(los_caches.begin(), los_caches.end(), [&](const auto& c) {
          return c->dloccmp(*it->ent());
        });

    if (should_be_in_los && !in_los) {
      ER_INFO("Remove tracked DPO cache%d@%s/%s xspan=%s,yspan=%s: not in LOS xspan=%s,yspan=%s",
              it->ent()->id().v(),
              rcppsw::to_string(it->ent()->rcenter2D()).c_str(),
              rcppsw::to_string(it->ent()->dcenter2D()).c_str(),
              rcppsw::to_string(it->ent()->xrspan()).c_str(),
              rcppsw::to_string(it->ent()->yrspan()).c_str(),
              rcppsw::to_string(c_los->xspan()).c_str(),
              rcppsw::to_string(c_los->yspan()).c_str());

      /*
       * Copy iterator object + iterator increment MUST be before removal to
       * avoid iterator invalidation and undefined behavior (I've seen both a
       * segfault and infinite loop). See FORDYCA#589.
       */
      carepr::base_cache* tmp = (*it).ent();
      ++it;
      m_store->cache_remove(tmp);
      ER_ASSERT(nullptr == m_store->find(tmp),
                "Cache%d still exists in store after removal",
                tmp->id().v());
    } else {
      ++it;
    }
  } /* while(it..) */
} /* los_tracking_sync() */

void dpo_perception_subsystem::los_tracking_sync(
    const repr::forager_los* const c_los,
    const cds::block3D_vectorno& los_blocks) {
  /*
   * If the location of one of the blocks we are tracking is in our LOS, then
   * the corresponding block should also be in our LOS. If it is not, then our
   * tracked version is out of date and needs to be removed.
   *
   * This logic does NOT handle cases where the tracked block is in our LOS, but
   * has moved since we last saw it (since that is limited to at most a single
   * block, it is handled by the \ref block_found event).
   */
  auto range = m_store->blocks().values_range();
  auto it = range.begin();

  while (it != range.end()) {
    ER_TRACE("Examining block%d@%s/%s",
             it->ent()->id().v(),
             rcppsw::to_string(it->ent()->ranchor2D()).c_str(),
             rcppsw::to_string(it->ent()->danchor2D()).c_str());

    if (!c_los->contains_abs(it->ent()->danchor2D())) {
      ++it;
      continue;
    }
    /*
     * static_cast is safe, because we verified we are only dealing with 2D
     * blocks earlier in the update chain.
     */
    auto exists_in_los =
        los_blocks.end() !=
        std::find_if(los_blocks.begin(), los_blocks.end(), [&](const auto& b) {
          return static_cast<crepr::base_block3D*>(b)->idcmp(*(it->ent()));
        });
    ER_TRACE("Block%d location in LOS", it->ent()->id().v());
    if (!exists_in_los) {
      ER_INFO("Remove tracked block%d@%s/%s: not in LOS blocks",
              it->ent()->id().v(),
              rcppsw::to_string(it->ent()->ranchor2D()).c_str(),
              rcppsw::to_string(it->ent()->danchor2D()).c_str());
      /*
       * Copy iterator object + iterator increment MUST be before removal to
       * avoid iterator invalidation and undefined behavior (I've seen both a
       * segfault and infinite loop). See FORDYCA#589.
       */
      crepr::base_block3D* tmp = (*it).ent();
      ++it;
      m_store->block_remove(tmp);
    } else {
      ++it;
    }
  } /* while(it..) */
} /* los_tracking_sync() */

/*******************************************************************************
 * DPO Perception Metrics
 ******************************************************************************/
uint dpo_perception_subsystem::n_known_blocks(void) const {
  return m_store->blocks().size();
} /* n_known_blocks() */

uint dpo_perception_subsystem::n_known_caches(void) const {
  return m_store->caches().size();
} /* n_known_caches() */

crepr::pheromone_density dpo_perception_subsystem::avg_block_density(void) const {
  auto range = m_store->blocks().const_values_range();
  if (m_store->blocks().empty()) {
    return crepr::pheromone_density();
  }
  return std::accumulate(range.begin(),
                         range.end(),
                         crepr::pheromone_density(),
                         [&](const auto& accum, const auto& block) {
                           return accum + block.density();
                         }) /
         m_store->blocks().size();
} /* avg_block_density() */

crepr::pheromone_density dpo_perception_subsystem::avg_cache_density(void) const {
  auto range = m_store->caches().const_values_range();
  if (m_store->caches().empty()) {
    return crepr::pheromone_density();
  }
  return std::accumulate(range.begin(),
                         range.end(),
                         crepr::pheromone_density(),
                         [&](const auto& accum, const auto& cache) {
                           return accum + cache.density();
                         }) /
         m_store->caches().size();
} /* avg_cache_density() */

NS_END(controller, fordyca);
