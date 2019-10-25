/**
 * @file arena_map.cpp
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
#include "fordyca/ds/arena_map.hpp"

#include <argos3/plugins/simulator/media/led_medium.h>

#include "fordyca/config/arena/arena_map_config.hpp"
#include "fordyca/config/saa_xml_names.hpp"
#include "fordyca/ds/cell2D.hpp"
#include "fordyca/events/cell_cache_extent.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/repr/arena_cache.hpp"
#include "fordyca/repr/cube_block.hpp"
#include "fordyca/repr/ramp_block.hpp"
#include "fordyca/support/base_loop_functions.hpp"
#include "fordyca/support/block_manifest_processor.hpp"
#include "fordyca/support/light_type_index.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, ds);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
arena_map::arena_map(const config::arena::arena_map_config* config)
    : ER_CLIENT_INIT("fordyca.ds.arena_map"),
      decorator(config->grid.resolution,
                static_cast<uint>(config->grid.upper.x() + arena_padding()),
                static_cast<uint>(config->grid.upper.y() + arena_padding())),
      m_blocks(support::block_manifest_processor(&config->blocks.dist.manifest)
                   .create_blocks()),
      m_nest(config->nest.dims,
             config->nest.center,
             config->grid.resolution,
             support::light_type_index()[support::light_type_index::kNest]),
      m_block_dispatcher(&decoratee(),
                         config->grid.resolution,
                         &config->blocks.dist),
      m_redist_governor(&config->blocks.dist.redist_governor) {
  ER_INFO("real=(%fx%f), discrete=(%zux%zu), resolution=%f",
          xrsize(),
          yrsize(),
          xdsize(),
          ydsize(),
          grid_resolution().v());
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool arena_map::initialize(support::base_loop_functions* loop, rmath::rng* rng) {
  for (auto& l : m_nest.lights()) {
    loop->AddEntity(*l);
  } /* for(&l..) */

  return m_block_dispatcher.initialize(rng);
} /* initialize() */

void arena_map::caches_add(const cache_vector& caches,
                           support::base_loop_functions* loop) {
  auto& medium = loop->GetSimulator().GetMedium<argos::CLEDMedium>(
      config::saa_xml_names().leds_saa);

  /*
   * Add all lights of caches to the arena. Cache lights are added directly to
   * the LED medium, which is different than what is rendered to the screen, so
   * they actually are invisible.
   */
  for (auto& c : caches) {
    medium.AddEntity(*c->light());
  } /* for(&c..) */

  m_caches.insert(m_caches.end(), caches.begin(), caches.end());
  ER_INFO("Add %zu created caches, total=%zu", caches.size(), m_caches.size());
} /* caches_add() */

int arena_map::robot_on_block(const rmath::vector2d& pos) const {
  /*
   * Caches hide blocks, add even though a robot may technically be standing on
   * a block, if it is also standing in a cache, that takes priority.
   */
  if (-1 != robot_on_cache(pos)) {
    ER_TRACE("Block hidden by cache%d", robot_on_cache(pos));
    return -1;
  }
  for (auto& b : m_blocks) {
    if (b->contains_point(pos)) {
      return b->id();
    }
  } /* for(&b..) */
  return -1;
} /* robot_on_block() */

int arena_map::robot_on_cache(const rmath::vector2d& pos) const {
  for (auto& c : m_caches) {
    if (c->contains_point(pos)) {
      return c->id();
    }
  } /* for(&c..) */
  return -1;
} /* robot_on_cache() */

bool arena_map::distribute_single_block(std::shared_ptr<repr::base_block>& block) {
  /* return TRUE because the distribution of nothing is ALWAYS successful */
  if (!m_redist_governor.dist_status()) {
    return true;
  }
  /* Entities that need to be avoided during block distribution are:
   *
   * - All existing caches
   * - All existing blocks
   * - Nest
   */
  ds::const_entity_list entities;
  for (auto& cache : m_caches) {
    entities.push_back(cache.get());
  } /* for(&cache..) */

  for (auto& b : m_blocks) {
    if (b != block) {
      entities.push_back(b.get());
    }
  } /* for(&b..) */
  entities.push_back(&m_nest);
  return m_block_dispatcher.distribute_block(block, entities);
} /* disribute_single_block() */

void arena_map::distribute_all_blocks(void) {
  // Reset all the cells to clear old references to blocks
  decoratee().reset();

  /* distribute blocks */
  ds::const_entity_list entities;
  for (auto& cache : m_caches) {
    entities.push_back(cache.get());
  } /* for(&cache..) */
  entities.push_back(&m_nest);
  bool b = m_block_dispatcher.distribute_blocks(m_blocks, entities);
  ER_ASSERT(b, "Unable to perform initial block distribution");

  /*
   * Once all blocks have been distributed, and (possibly) all caches have been
   * created via block consolidation, all cells that do not have blocks or
   * caches should be empty.
   */
  for (size_t i = 0; i < xdsize(); ++i) {
    for (size_t j = 0; j < ydsize(); ++j) {
      cell2D& cell = decoratee().access<arena_grid::kCell>(i, j);
      if (!cell.state_has_block() && !cell.state_has_cache() &&
          !cell.state_in_cache_extent()) {
        events::cell_empty_visitor op(cell.loc());
        op.visit(cell);
      }
    } /* for(j..) */
  }   /* for(i..) */
} /* distribute_all_blocks() */

void arena_map::cache_remove(const std::shared_ptr<repr::arena_cache>& victim,
                             support::base_loop_functions* loop) {
  /* Remove light for cache from ARGoS */
  auto& medium = loop->GetSimulator().GetMedium<argos::CLEDMedium>(
      config::saa_xml_names().leds_saa);
  medium.RemoveEntity(*victim->light());

  /* Remove cache */
  size_t before = caches().size();
  RCSW_UNUSED int id = victim->id();
  m_caches.erase(std::remove(m_caches.begin(), m_caches.end(), victim));
  ER_ASSERT(caches().size() == before - 1, "Cache%d not removed", id);
} /* cache_remove() */

void arena_map::cache_extent_clear(
    const std::shared_ptr<repr::arena_cache>& victim) {
  auto xspan = victim->xspan();
  auto yspan = victim->yspan();

  /*
   * To reset all cells covered by the cache's extent, we simply send them a
   * CELL_EMPTY event. EXCEPT for the cell that hosted the actual cache, because
   * it is currently in the HAS_BLOCK state as part of a \ref cached_block_pickup,
   * and clearing it here will trigger an assert later.
   */
  auto xmin = static_cast<uint>(std::ceil(xspan.lb() / grid_resolution().v()));
  auto xmax = static_cast<uint>(std::ceil(xspan.ub() / grid_resolution().v()));
  auto ymin = static_cast<uint>(std::ceil(yspan.lb() / grid_resolution().v()));
  auto ymax = static_cast<uint>(std::ceil(yspan.ub() / grid_resolution().v()));

  for (uint i = xmin; i < xmax; ++i) {
    for (uint j = ymin; j < ymax; ++j) {
      rmath::vector2u c = rmath::vector2u(i, j);
      if (c != victim->dloc()) {
        ER_ASSERT(victim->contains_point(
                      rmath::uvec2dvec(c, grid_resolution().v())),
                  "Cache%d does not contain point (%u, %u) within its extent",
                  victim->id(),
                  i,
                  j);

        auto& cell = decoratee().access<arena_grid::kCell>(i, j);
        ER_ASSERT(cell.state_in_cache_extent(),
                  "cell(%u, %u) not in CACHE_EXTENT [state=%d]",
                  i,
                  j,
                  cell.fsm().current_state());
        events::cell_empty_visitor e(c);
        e.visit(cell);
      }
    } /* for(j..) */
  }   /* for(i..) */
} /* cache_extent_clear() */

NS_END(ds, fordyca);
