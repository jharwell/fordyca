/**
 * \file ntimestep_perception_subsystem.cpp
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
#include "fordyca/subsystem/perception/ntimestep_perception_subsystem.hpp"

#include "cosm/arena/repr/base_cache.hpp"
#include "cosm/repr/block_metadata.hpp" // To get block timestep pick up and UUID

#include "fordyca/subsystem/perception/los_proc_verify.hpp"
#include "fordyca/subsystem/perception/oracular_info_receptor.hpp"
#include "fordyca/subsystem/perception/ds/nb_store.hpp"
#include "fordyca/events/block_found.hpp"
#include "fordyca/events/cache_found.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, subsystem, perception);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/

ntimestep_perception_subsystem::ntimestep_perception_subsystem(
    const cspconfig::perception_config* const config)
    : ER_CLIENT_INIT("fordyca.subsystem.perception.ntimestep"),
      foraging_perception_subsystem(config, 
                                    std::make_unique<ds::nb_store>(&config->ntimestep.N)) {}  //TODO: replace unique pointer here to point towards nb_store

ntimestep_perception_subsystem::~ntimestep_perception_subsystem(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void ntimestep_perception_subsystem::update(oracular_info_receptor* const receptor, uint timestep) {
  c_timestep = timestep;
  process_los(los(), receptor);
  ER_ASSERT(los_proc_verify(los()), "LOS verification failed");
  store()->update_all(); //TODO: here update all the objects from our memory model into the "tracked_blocks" variable
} /* update() */

void ntimestep_perception_subsystem::reset(void) { store()->clear_all(); }

void ntimestep_perception_subsystem::process_los(
    const repr::forager_los* const c_los,
    oracular_info_receptor* const receptor) {
  ER_TRACE("LOS LL=%s, LR=%s, UL=%s UR=%s",
           c_los->abs_ll().to_str().c_str(),
           c_los->abs_lr().to_str().c_str(),
           c_los->abs_ul().to_str().c_str(),
           c_los->abs_ur().to_str().c_str());

  /* If we are in an oracular controller, process the updates from the oracle */
  if (nullptr != receptor) { //TODO: in oracular controller need to update the seen blocks into our ntimestep ds -- aka our Map
    receptor->store_update(store(), c_timestep); 
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

void ntimestep_perception_subsystem::process_los_caches(
    const repr::forager_los* const c_los) {
  cads::bcache_vectorno los_caches = c_los->caches();
  if (!los_caches.empty()) {
    ER_DEBUG("Caches in LOS: [%s]", rcppsw::to_string(los_caches).c_str());
  }

  /* Fix our tracking of caches that no longer exist in our perception */
  los_tracking_sync(c_los, los_caches);

  for (auto& cache : los_caches) {
    if (!store()->contains(cache)) {
      ER_INFO("Discovered Cache%d@%s/%s",
              cache->id().v(),
              rcppsw::to_string(cache->rcenter2D()).c_str(),
              rcppsw::to_string(cache->dcenter2D()).c_str());
    } else if (cache->n_blocks() != store()->find(cache)->ent()->n_blocks()) {
      ER_INFO("Update cache%d@%s blocks: %zu -> %zu",
              cache->id().v(),
              rcppsw::to_string(cache->dcenter2D()).c_str(),
              store->find(cache)->ent()->n_blocks(),
              cache->n_blocks());
    }
    events::cache_found_visitor op(cache);
    op.visit(*store());
  } /* for(cache..) */
} /* process_los_caches() */

void ntimestep_perception_subsystem::process_los_blocks( //TODO: processing blocks here so update our known blocks 
    const repr::forager_los* const c_los) {
  /*
   * Because this is computed, rather than a returned reference to a member
   * variable, we can't use separate begin()/end() calls with it, and need to
   * explicitly assign it.
   */
  auto los_blocks = c_los->blocks();
  ER_DEBUG("Blocks in DPO store: [%s]", //TODO: do a debug of the blocks in our Map?
           rcppsw::to_string(store->blocks()).c_str());
  if (!los_blocks.empty()) {
    ER_DEBUG("Blocks in LOS: [%s]", rcppsw::to_string(los_blocks).c_str());
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

    ER_CHECKI(!store->contains(block),
              "Discovered block%d@%s/%s",
              block->id().v(),
              rcppsw::to_string(block->ranchor2D()).c_str(),
              rcppsw::to_string(block->danchor2D()).c_str());
    ER_CHECKD(store->contains(block),
              "Block%d@%s/%s already known",
              block->id().v(),
              rcppsw::to_string(block->ranchor2D()).c_str(),
              rcppsw::to_string(block->danchor2D()).c_str());
    events::block_found_visitor op(block);
    op.visit(*store());
  } /* for(block..) */
} /* process_los() */

void ntimestep_perception_subsystem::los_tracking_sync( //TODO: replace dpo store with ntimestep ds
    const repr::forager_los* const c_los,
    const cads::bcache_vectorno& los_caches) {
  /*
   * If the location of one of the caches we are tracking is in our LOS, then
   * the corresponding cache should also be in our LOS. If it is not, then our
   * tracked version is out of date and needs to be removed.
   */
  auto range = store()->known_caches();
  auto it = range.begin();

  while (it != range.end()) {
    ER_TRACE("Check tracked DPO cache%d@%s/%s xspan=%s,yspan=%s w/LOS "
             "xspan=%s,yspan=%s",
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
    bool should_be_in_los = (*it)->yrspan().overlaps_with(c_los->yspan()) ||
                            (*it)->xrspan().overlaps_with(c_los->xspan());

    bool in_los =
        los_caches.end() !=
        std::find_if(los_caches.begin(), los_caches.end(), [&](const auto& c) {
          return c->dloccmp(*it->ent());
        });

    if (should_be_in_los && !in_los) {
      ER_INFO("Remove tracked DPO cache%d@%s/%s xspan=%s,yspan=%s: not in LOS "
              "xspan=%s,yspan=%s",
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
      carepr::base_cache* tmp = *it;
      ++it;
      store()->cache_remove(tmp);
      ER_ASSERT(nullptr == store()->find(tmp),
                "Cache%d still exists in store after removal",
                tmp->id().v());
    } else {
      ++it;
    }
  } /* while(it..) */
} /* los_tracking_sync() */

void ntimestep_perception_subsystem::los_tracking_sync( //TODO: replace dpo store with ntimestep ds
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
  auto range = store()->known_blocks();
  auto it = range.begin();

  while (it != range.end()) {
    ER_TRACE("Examining block%d@%s/%s",
             (*it)->id().v(),
             rcppsw::to_string((*it)->ranchor2D()).c_str(),
             rcppsw::to_string((*it)->danchor2D()).c_str());

    if (!c_los->contains_abs((*it)->danchor2D())) {
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
      crepr::base_block3D* tmp = *it;
      ++it;
      store()->block_remove(tmp);
    } else {
      ++it;
    }
  } /* while(it..) */
} /* los_tracking_sync() */

/*******************************************************************************
 * NTimestep Perception Metrics
 ******************************************************************************/
uint ntimestep_perception_subsystem::n_known_blocks(void) const { //TODO: Perception metrics here to be replaced with ntimestep variant
  return store()->known_blocks().size();
} /* n_known_blocks() */

uint ntimestep_perception_subsystem::n_known_caches(void) const { 
  return store()->known_caches().size();
} /* n_known_caches() */

NS_END(cognitive, controller, fordyca);
