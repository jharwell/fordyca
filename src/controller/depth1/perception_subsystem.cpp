/**
 * @file perception_subsystem.cpp
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
#include "fordyca/controller/depth1/perception_subsystem.hpp"
#include "fordyca/ds/cell2D.hpp"
#include "fordyca/ds/perceived_arena_map.hpp"
#include "fordyca/events/cache_found.hpp"
#include "fordyca/representation/base_cache.hpp"
#include "fordyca/representation/line_of_sight.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth1);
using ds::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
perception_subsystem::perception_subsystem(
    const params::perception_params* const params,
    const std::string& id)
    : base_perception_subsystem(params, id),
      ER_CLIENT_INIT("fordyca.controller.depth1.perception") {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void perception_subsystem::process_los(
    const representation::line_of_sight* const c_los) {
  base_perception_subsystem::process_los(c_los);
  /*
   * Because this is computed, rather than a returned reference to a member
   * variable, we can't use separate begin()/end() calls with it, and need to
   * explicitly assign it.
   */
  representation::line_of_sight::const_cache_list caches = c_los->caches();
  std::string accum;
  std::for_each(caches.begin(), caches.end(), [&](const auto& c) {
    accum += "c" + std::to_string(c->id()) + "->(" +
             std::to_string(c->discrete_loc().first) + "," +
             std::to_string(c->discrete_loc().second) + "),";
  });
  if (!caches.empty()) {
    ER_DEBUG("Caches in LOS: [%s]", accum.c_str());
  }

  /*
   * If the robot thinks that a cell contains a cache, because the cell had one
   * the last time it passed nearby, but when coming near the cell a second time
   * the cell does not contain a cache, then the cache was depleted between then
   * and now, and it needs to update its internal representation accordingly.
   */
  for (uint i = 0; i < c_los->xsize(); ++i) {
    for (uint j = 0; j < c_los->ysize(); ++j) {
      rcppsw::math::dcoord2 d = c_los->cell(i, j).loc();
      if (!c_los->cell(i, j).state_has_cache() &&
          map()->access<occupancy_grid::kCell>(d).state_has_cache()) {
        ER_DEBUG("Correct cache%d discrepency at (%u, %u)",
                 map()->access<occupancy_grid::kCell>(d).cache()->id(),
                 d.first,
                 d.second);
        map()->cache_remove(map()->access<occupancy_grid::kCell>(d).cache());
      }
    } /* for(j..) */
  }   /* for(i..) */

  for (auto cache : c_los->caches()) {
    /*
     * The state of a cache can change between when the robot saw it last
     * (i.e. different # of blocks in it), and so you need to always process
     * caches in the LOS, even if you already know about them.
     */
    ER_DEBUG("LOS: Cache%d@(%u,%u): %zu blocks",
             cache->id(),
             cache->discrete_loc().first,
             cache->discrete_loc().second,
             cache->n_blocks());
    auto& cell = map()->access<occupancy_grid::kCell>(cache->discrete_loc());
    auto& density =
        map()->access<occupancy_grid::kPheromone>(cache->discrete_loc());

    if (!cell.state_has_cache()) {
      ER_INFO("Discovered cache%d(%u, %u): %zu blocks (density=%f, state=%d)",
              cache->id(),
              cache->discrete_loc().first,
              cache->discrete_loc().second,
              cache->n_blocks(),
              density.last_result(),
              cell.fsm().current_state());
    } else if (cell.state_has_cache() &&
               cell.cache()->n_blocks() != cache->n_blocks()) {
      ER_INFO("Fixed cache%d@(%u, %u) block count: %zu blocks -> %zu blocks",
              cache->id(),
              cache->discrete_loc().first,
              cache->discrete_loc().second,
              cell.cache()->n_blocks(),
              cache->n_blocks());
    }
    /*
     * The cache we get a handle to is owned by the simulation, and we don't
     * want to just pass that into the robot's arena_map, as keeping them in
     * sync is not possible in all situations.
     *
     * For example, if a block executing the collector task picks up a block and
     * tries to compute the best cache to bring it to, only to have one or more
     * of its cache references be invalid due to other robots causing caches to
     * be created/destroyed.
     *
     * Cloning is definitely necessary here.
     */
    events::cache_found op(cache->clone());
    map()->accept(op);
  } /* for(cache..) */
} /* process_los() */

void perception_subsystem::processed_los_verify(
    const representation::line_of_sight* const c_los) const {
  base_perception_subsystem::processed_los_verify(c_los);
  /*
   * Verify that for each cell that contained a cache in the LOS, the
   * corresponding cell in the PAM also contains the same cache.
   */
  for (auto& cache : c_los->caches()) {
    auto& cell = map()->access<occupancy_grid::kCell>(cache->discrete_loc());
    ER_ASSERT(cell.state_has_cache(),
              "Cell at (%u,%u) not in HAS_CACHE state",
              cache->discrete_loc().first,
              cache->discrete_loc().second);
    ER_ASSERT(cell.cache()->id() == cache->id(),
              "Cell at (%u,%u) has wrong cache ID (%u vs %u)",
              cache->discrete_loc().first,
              cache->discrete_loc().second,
              cache->id(),
              cell.cache()->id());
  } /* for(&block..) */

  /*
   * Verify processing (cache part, other parts done by parent class). We do not
   * check the CACHE_EXTENT state because the PAM does not need that
   * information.
   */
  for (uint i = 0; i < c_los->xsize(); ++i) {
    for (uint j = 0; j < c_los->ysize(); ++j) {
      rcppsw::math::dcoord2 d = c_los->cell(i, j).loc();
      auto& cell1 = c_los->cell(i, j);
      auto& cell2 = map()->access<occupancy_grid::kCell>(d);
      if (cell1.state_has_cache()) {
        ER_ASSERT(cell1.fsm().current_state() == cell2.fsm().current_state(),
                  "LOS/PAM disagree on state of cell at (%u, %u): %d/%d",
                  d.first,
                  d.second,
                  cell1.fsm().current_state(),
                  cell2.fsm().current_state());
        ER_ASSERT(
            cell1.cache()->n_blocks() == cell2.cache()->n_blocks(),
            "LOS/PAM disagree on # of blocks in cell at (%u, %u): %zu/%zu",
            d.first,
            d.second,
            cell1.cache()->n_blocks(),
            cell2.cache()->n_blocks());
      }
    } /* for(j..) */
  }   /* for(i..) */

  for (auto& c1 : c_los->caches()) {
    for (auto& c2 : map()->caches()) {
      if (*c1 == *c2) {
        auto& cell = map()->access<occupancy_grid::kCell>(c2->discrete_loc());
        ER_ASSERT(
            c1->n_blocks() == cell.cache()->n_blocks(),
            "LOS/PAM disagree on # of blocks in cell at (%u, %u): %zu/%zu",
            cell.loc().first,
            cell.loc().second,
            c1->n_blocks(),
            cell.cache()->n_blocks());
      }
    } /* for(c2..) */
  }
} /* processed_los_verify() */

NS_END(depth1, controller, fordyca);
