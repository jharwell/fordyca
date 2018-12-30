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
#include "fordyca/events/block_found.hpp"
#include "fordyca/events/cache_found.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/representation/base_cache.hpp"
#include "fordyca/ds/dpo_store.hpp"
#include "fordyca/params/perception/perception_params.hpp"

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
  processed_los_verify(los());
  m_store->decay_all();
} /* update() */

void dpo_perception_subsystem::reset(void) { m_store->clear_all(); }

void dpo_perception_subsystem::process_los(
    const representation::line_of_sight* const c_los) {
  ER_TRACE("LOS LL=%s, LR=%s, UL=%s UR=%s",
           c_los->abs_ll().to_str().c_str(),
           c_los->abs_lr().to_str().c_str(),
           c_los->abs_ul().to_str().c_str(),
           c_los->abs_ur().to_str().c_str());

  process_los_blocks(c_los);
  process_los_caches(c_los);
} /* process_los() */

void dpo_perception_subsystem::process_los_caches(
    const representation::line_of_sight* const c_los) {
  ds::cache_list caches = c_los->caches();
  if (!caches.empty()) {
    ER_DEBUG("Caches in LOS: [%s]", rcppsw::to_string(caches).c_str());
  }
  for (auto& cache : caches) {
    if (!m_store->contains(cache)) {
      ER_INFO("Discovered Cache%d@%s/%s",
              cache->id(),
              cache->real_loc().to_str().c_str(),
              cache->discrete_loc().to_str().c_str());
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
    events::cache_found op(cache->clone());
    m_store->accept(op);
  } /* for(cache..) */
} /* process_los_caches() */

void dpo_perception_subsystem::process_los_blocks(
    const representation::line_of_sight* const c_los) {
  /*
   * Because this is computed, rather than a returned reference to a member
   * variable, we can't use separate begin()/end() calls with it, and need to
   * explicitly assign it.
   */
  ds::block_list blocks = c_los->blocks();
  if (!blocks.empty()) {
    ER_DEBUG("Blocks in LOS: [%s]", rcppsw::to_string(blocks).c_str());
  }

  /*
   * If one of the blocks we are tracking should be in our LOS and actually is
   * not, then our tracked version is out of date and needs to be removed. This
   * logic does NOT handle cases where the tracked block is in our LOS, but has
   * moved since we last saw it (since that is limited to at most a single
   * block, it is handled by the \ref block_found event).
   */
  auto los_blocks = c_los->blocks();
  for (auto &&block : m_store->blocks()) {
    if (c_los->contains_loc(block.ent()->discrete_loc())) {
      auto it = std::find_if(los_blocks.begin(),
                             los_blocks.end(),
                             [&](const auto& b) {
                               return b->idcmp(*block.ent_obj());
                             });
      if (los_blocks.end() == it) {
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
    events::block_found op(block->clone());
    m_store->accept(op);
  } /* for(block..) */
} /* process_los() */

void dpo_perception_subsystem::processed_los_verify(
    const representation::line_of_sight* const c_los) const {
  /*
   * Verify that for each cell that contained a block in the LOS, that the block
   * is also contained in the store after processing.
   */
  for (auto& block : c_los->blocks()) {
    ER_ASSERT(m_store->contains(block),
              "Store does not contain block%d@%s",
              block->id(),
              block->discrete_loc().to_str().c_str());
  } /* for(&block..) */

  /*
   * Verify that for each cell that contained a cache in the LOS:
   *
   * - The corresponding cache exists in the store.
   * - The store and LOS versions of the cache have the same # of blocks
   */
  for (auto& c1 : c_los->caches()) {
    auto exists = m_store->find(c1);
    ER_ASSERT(nullptr != exists,
              "LOS Cache%d@%s does not exist in DPO store",
              c1->id(),
              c1->discrete_loc().to_str().c_str());
    ER_ASSERT(c1->n_blocks() == exists->ent()->n_blocks(),
              "LOS/DPO store disagree on # of blocks in cache%d@%s: %zu/%zu",
              c1->id(),
              c1->discrete_loc().to_str().c_str(),
              c1->n_blocks(),
              exists->ent()->n_blocks());
  } /* for(c1..) */
} /* processed_los_verify() */

NS_END(controller, fordyca);
