/**
 * \file dpo_semantic_map.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/subsystem/perception/ds/dpo_semantic_map.hpp"

#include "cosm/arena/repr/base_cache.hpp"

#include "fordyca/events/cell2D_empty.hpp"
#include "fordyca/subsystem/perception/events/block_found.hpp"
#include "fordyca/subsystem/perception/events/cache_found.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, subsystem, perception, ds);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dpo_semantic_map::dpo_semantic_map(const config::mdpo_config* c_config)
    : ER_CLIENT_INIT("fordyca.subsystem.perception.ds.dpo_semantic_map"),
      decorator(c_config),
      m_store(&c_config->pheromone) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
model_update_result dpo_semantic_map::block_update(tracked_block_type&& block) {
  auto res = store()->block_update(std::move(block));

  /*
   * ONLY if the underlying DPO store actually changed do we update what block
   * the cell points to. If we do it unconditionally, we are left
   * with dangling references as a result of mixing unique_ptr and raw ptr. See
   * FORDYCA#229.
   */
  if (model_update_status::ekBLOCK_MOVED == res.reason ||
      model_update_status::ekNEW_BLOCK_ADDED == res.reason) {
    auto* found = store()->find(block.ent());

    if (model_update_status::ekBLOCK_MOVED == res.reason) {
      ER_DEBUG("Updating cell@%s: Block%d moved %s -> %s",
               res.old_loc.to_str().c_str(),
               found->ent()->id().v(),
               res.old_loc.to_str().c_str(),
               found->ent()->danchor2D().to_str().c_str());

      /* clear old cell */
      cdops::cell2D_empty_visitor op(res.old_loc);
      op.visit(access<occupancy_grid::kCell>(res.old_loc));

      /* reset density in old cell */
      auto& density = access<occupancy_grid::kPheromone>(res.old_loc);
      density.reset();
    }

    /*
     * At this point we know that if the block was previously tracked, its old
     * host cell has been updated and the block updated in the store, so we are
     * good to update the NEW host cell to point to the block.
     */
    events::block_found_visitor e(found->ent());
    e.visit(access<occupancy_grid::kCell>(found->ent()->danchor2D()));

    /* copy density from DPO store to new cell  */
    access<occupancy_grid::kPheromone>(found->ent()->danchor2D()) =
        found->density();
  }
  return res;
} /* block_update() */

model_update_result dpo_semantic_map::cache_update(tracked_cache_type&& cache) {
  auto res = store()->cache_update(std::move(cache));
  if (model_update_status::ekCACHE_UPDATED == res.reason ||
      model_update_status::ekNEW_CACHE_ADDED == res.reason) {
    auto* found = store()->find(cache.ent());

    /* update cache host cell */
    fspevents::cache_found_visitor e(found->ent());
    e.visit(access<occupancy_grid::kCell>(found->ent()->dcenter2D()));

    /* copy density from DPO store to new cell  */
    access<occupancy_grid::kPheromone>(found->ent()->dcenter2D()) =
        found->density();
  }
  return res;
} /* cache_update() */

bool dpo_semantic_map::cache_remove(carepr::base_cache* const victim) {
  if (m_store.cache_remove(victim)) {
    ER_DEBUG("Updating cell@%s for removed cache",
             victim->dcenter2D().to_str().c_str());
    cdops::cell2D_empty_visitor op(victim->dcenter2D());
    op.visit(decoratee().access<occupancy_grid::kCell>(victim->dcenter2D()));

    /* reset density */
    decoratee().access<occupancy_grid::kPheromone>(victim->dcenter2D()).reset();
    return true;
  }
  return false;
} /* cache_remove() */

bool dpo_semantic_map::block_remove(crepr::sim_block3D* const victim) {
  if (m_store.block_remove(victim)) {
    ER_DEBUG("Updating cell@%s for removed block",
             victim->danchor2D().to_str().c_str());
    fevents::cell2D_empty_visitor op(victim->danchor2D());
    op.visit(access<occupancy_grid::kCell>(victim->danchor2D()));

    /* reset density */
    decoratee().access<occupancy_grid::kPheromone>(victim->danchor2D()).reset();
    return true;
  }
  return false;
} /* block_remove() */

void dpo_semantic_map::decay_all(void) {
  decoratee().update();
  m_store.decay_all();

  for (const auto& b : m_store.tracked_blocks().values_range()) {
    const rmath::vector2z& loc = b.ent()->danchor2D();
    crepr::pheromone_density& map_density =
        decoratee().access<occupancy_grid::kPheromone>(loc);

    ER_ASSERT(std::fabs((map_density - b.density()).v()) <=
                  std::numeric_limits<double>::epsilon(),
              "FATAL: Map density@%s and DP block%d density disagree: %f vs %f",
              loc.to_str().c_str(),
              b.ent()->id().v(),
              map_density.v(),
              b.density().v());
  } /* for(&b..) */

  for (auto&& c : m_store.tracked_caches().values_range()) {
    const rmath::vector2z& loc = c.ent()->dcenter2D();
    crepr::pheromone_density& map_density =
        decoratee().access<occupancy_grid::kPheromone>(loc);

    ER_ASSERT(std::fabs((map_density - c.density()).v()) <=
                  std::numeric_limits<double>::epsilon(),
              "FATAL: Map density@%s and DP cache%d density disagree: %f vs %f",
              loc.to_str().c_str(),
              c.ent()->id().v(),
              map_density.v(),
              c.density().v());
  } /* for(&c..) */
} /* decay_all() */

NS_END(ds, perception, subsystem, fordyca);
