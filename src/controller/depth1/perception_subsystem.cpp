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
#include "fordyca/events/cache_found.hpp"
#include "fordyca/representation/base_cache.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/cell2D.hpp"
#include "fordyca/representation/line_of_sight.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth1);
using representation::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
perception_subsystem::perception_subsystem(
    const std::shared_ptr<rcppsw::er::server>& server,
    const params::perception_params* const params,
    const std::string& id)
    : base_perception_subsystem(server, params, id) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void perception_subsystem::process_los(
    const representation::line_of_sight* const c_los) {
  base_perception_subsystem::process_los(c_los);

  /*
   * If the robot thinks that a cell contains a cache, because the cell had one
   * the last time it passed nearby, but when coming near the cell a second time
   * the cell does not contain a cache, then the cache was depleted between then
   * and now, and it needs to update its internal representation accordingly.
   */
  for (size_t i = 0; i < c_los->xsize(); ++i) {
    for (size_t j = 0; j < c_los->ysize(); ++j) {
      rcppsw::math::dcoord2 d = c_los->cell(i, j).loc();
      if (!c_los->cell(i, j).state_has_cache() &&
          map()->access<occupancy_grid::kCellLayer>(d).state_has_cache()) {
        ER_DIAG("Correct cache%d discrepency at (%zu, %zu)",
                map()->access<occupancy_grid::kCellLayer>(d).cache()->id(),
                d.first,
                d.second);
        map()->cache_remove(
            map()->access<occupancy_grid::kCellLayer>(d).cache());
      }
    } /* for(j..) */
  }   /* for(i..) */

  for (auto cache : c_los->caches()) {
    /*
     * The state of a cache can change between when the robot saw it last
     * (i.e. different # of blocks in it), and so you need to always process
     * caches in the LOS, even if you already know about them.
     */
    if (!map()
             ->access<occupancy_grid::kCellLayer>(cache->discrete_loc())
             .state_has_cache()) {
      ER_NOM("Discovered cache%d at (%zu, %zu): %u blocks",
             cache->id(),
             cache->discrete_loc().first,
             cache->discrete_loc().second,
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
    events::cache_found op(server_ref(), cache->clone());
    map()->accept(op);
  } /* for(cache..) */
} /* process_los() */

NS_END(depth1, controller, fordyca);
