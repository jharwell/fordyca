/**
 * @file base_perception_subsystem.cpp
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
#include "fordyca/controller/base_perception_subsystem.hpp"
#include <algorithm>

#include "fordyca/ds/cell2D.hpp"
#include "fordyca/ds/perceived_arena_map.hpp"
#include "fordyca/events/block_found.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/fsm/cell2D_fsm.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/representation/line_of_sight.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
using ds::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_perception_subsystem::base_perception_subsystem(
    const params::perception_params* const params,
    const std::string& id)
    : ER_CLIENT_INIT("fordyca.controller.base_perception"),
      m_cell_stats(fsm::cell2D_fsm::ST_MAX_STATES),
      m_map(rcppsw::make_unique<ds::perceived_arena_map>(params, id)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_perception_subsystem::update(
    const representation::line_of_sight* const los) {
  update_cell_stats(los);
  process_los(los);
  processed_los_verify(los);
  m_map->update();
} /* update() */

void base_perception_subsystem::reset(void) { m_map->reset(); }

void base_perception_subsystem::process_los(
    const representation::line_of_sight* const c_los) {
  ER_TRACE("LOS LL=%s, LR=%s, UL=%s UR=%s",
           c_los->abs_ll().to_str().c_str(),
           c_los->abs_lr().to_str().c_str(),
           c_los->abs_ul().to_str().c_str(),
           c_los->abs_ur().to_str().c_str());

  /*
   * Because this is computed, rather than a returned reference to a member
   * variable, we can't use separate begin()/end() calls with it, and need to
   * explicitly assign it.
   */
  ds::const_block_list blocks = c_los->blocks();
  std::string accum;
  std::for_each(blocks.begin(), blocks.end(), [&](const auto& b) {
    accum += "b" + std::to_string(b->id()) + "->" +
             b->discrete_loc().to_str().c_str() + ",";
  });
  if (!blocks.empty()) {
    ER_DEBUG("Blocks in LOS: [%s]", accum.c_str());
  }

  /*
   * If the robot thinks that a cell contains a block, because the cell had one
   * the last time it passed nearby, but when coming near the cell a second time
   * the cell does not contain a block, then someone else picked up the block
   * between then and now, and it needs to update its internal representation
   * accordingly.
   */
  for (uint i = 0; i < c_los->xsize(); ++i) {
    for (uint j = 0; j < c_los->ysize(); ++j) {
      rmath::vector2u d = c_los->cell(i, j).loc();
      if (!c_los->cell(i, j).state_has_block() &&
          m_map->access<occupancy_grid::kCell>(d).state_has_block()) {
        auto block = m_map->access<occupancy_grid::kCell>(d).block();
        ER_DEBUG("Correct block%d %s/%s discrepency",
                 block->id(),
                 block->real_loc().to_str().c_str(),
                 block->discrete_loc().to_str().c_str());
        m_map->block_remove(block);
      } else if (c_los->cell(i, j).state_is_known() &&
                 !m_map->access<occupancy_grid::kCell>(d).state_is_known()) {
        ER_TRACE("Cell@%s now known to be empty", d.to_str().c_str());
        events::cell_empty e(d);
        m_map->accept(e);
      }
    } /* for(j..) */
  }   /* for(i..) */

  for (auto& block : c_los->blocks()) {
    ER_ASSERT(!block->is_out_of_sight(),
              "Block%d out of sight in LOS?",
              block->id());
    auto& cell = m_map->access<occupancy_grid::kCell>(block->discrete_loc());
    if (!cell.state_has_block()) {
      ER_INFO("Discovered block%d@%s/%s",
              block->id(),
              block->real_loc().to_str().c_str(),
              block->discrete_loc().to_str().c_str());
    } else if (cell.state_has_block()) {
      ER_DEBUG("Block%d@%s/%s already known",
               block->id(),
               block->real_loc().to_str().c_str(),
               block->discrete_loc().to_str().c_str());
      auto it = std::find_if(m_map->blocks().begin(),
                             m_map->blocks().end(),
                             [&](const auto& b) {
                               return b->id() == cell.block()->id();
                             });
      ER_ASSERT(it != m_map->blocks().end(),
                "Known block%d not in PAM",
                block->id());
    }
    events::block_found op(block->clone());
    m_map->accept(op);
  } /* for(block..) */
} /* process_los() */

void base_perception_subsystem::processed_los_verify(
    const representation::line_of_sight* const c_los) const {
  /*
   * Verify that for each cell that contained a block in the LOS, the
   * corresponding cell in the PAM also contains the same block.
   */
  for (auto& block : c_los->blocks()) {
    auto& cell = m_map->access<occupancy_grid::kCell>(block->discrete_loc());
    ER_ASSERT(cell.state_has_block(),
              "Cell@%s not in HAS_BLOCK state",
              block->discrete_loc().to_str().c_str());
    ER_ASSERT(cell.block()->id() == block->id(),
              "Cell@%s has wrong block ID (%u vs %u)",
              block->discrete_loc().to_str().c_str(),
              block->id(),
              cell.block()->id());
  } /* for(&block..) */

  /*
   * Verify that for each cell in LOS that was empty or contained a block, that
   * it matches the PAM.
   */
  for (uint i = 0; i < c_los->xsize(); ++i) {
    for (uint j = 0; j < c_los->ysize(); ++j) {
      rmath::vector2u d = c_los->cell(i, j).loc();
      auto& cell1 = c_los->cell(i, j);
      auto& cell2 = m_map->access<occupancy_grid::kCell>(d);

      if (cell1.state_has_block() || cell1.state_is_empty()) {
        ER_ASSERT(cell1.fsm().current_state() == cell2.fsm().current_state(),
                  "LOS/PAM disagree on state of cell@%s: %d/%d",
                  d.to_str().c_str(),
                  cell1.fsm().current_state(),
                  cell2.fsm().current_state());
        if (cell1.state_has_block()) {
          ER_ASSERT(cell1.block()->id() == cell2.block()->id(),
                    "LOS/PAM disagree on block id in cell@%s: %d/%d",
                    d.to_str().c_str(),
                    cell1.block()->id(),
                    cell2.block()->id());
        }
      }
    } /* for(j..) */
  }   /* for(i..) */
} /* processed_los_verify() */

void base_perception_subsystem::update_cell_stats(
    const representation::line_of_sight* const los) {
  for (uint i = 0; i < los->xsize(); ++i) {
    for (uint j = 0; j < los->ysize(); ++j) {
      rmath::vector2u d = los->cell(i, j).loc();
      if (los->cell(i, j).state_is_empty() &&
          m_map->access<occupancy_grid::kCell>(d).state_is_known() &&
          !m_map->access<occupancy_grid::kCell>(d).state_is_empty()) {
        m_cell_stats[fsm::cell2D_fsm::ST_EMPTY]++;
      } else if (los->cell(i, j).state_has_block() &&
                 m_map->access<occupancy_grid::kCell>(d).state_is_known() &&
                 !m_map->access<occupancy_grid::kCell>(d).state_has_block()) {
        m_cell_stats[fsm::cell2D_fsm::ST_HAS_BLOCK]++;
      } else if (los->cell(i, j).state_has_cache() &&
                 m_map->access<occupancy_grid::kCell>(d).state_is_known() &&
                 !m_map->access<occupancy_grid::kCell>(d).state_has_cache()) {
        m_cell_stats[fsm::cell2D_fsm::ST_HAS_CACHE]++;
      }
    } /* for(j..) */
  }   /* for(i..) */
} /* update_cell_stats() */

/*******************************************************************************
 * World Model Metrics
 ******************************************************************************/
__rcsw_pure double base_perception_subsystem::known_percentage(void) const {
  return m_map->known_cell_count() /
         static_cast<double>(m_map->xdsize() * m_map->ydsize());
} /* known_percentage() */

double base_perception_subsystem::unknown_percentage(void) const {
  return 1.0 - known_percentage();
} /* unknown_percentage() */

void base_perception_subsystem::reset_metrics(void) {
  m_cell_stats.assign(m_cell_stats.size(), 0);
} /* reset_metrics() */

NS_END(controller, fordyca);
