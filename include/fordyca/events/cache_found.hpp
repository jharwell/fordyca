/**
 * @file cache_found.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_CACHE_FOUND_HPP_
#define INCLUDE_FORDYCA_EVENTS_CACHE_FOUND_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/events/perceived_cell_op.hpp"
#include "rcppsw/common/er_client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation { class cache; }

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class cache_found : public perceived_cell_op,
                    public rcppsw::common::er_client {
 public:
  cache_found(const std::shared_ptr<rcppsw::common::er_server>& server,
              const representation::cache* cache, size_t x, size_t y);
  ~cache_found(void) { er_client::rmmod(); }

  void visit(representation::cell2D& cell) override;
  void visit(representation::perceived_cell2D& cell) override;

  /* depth1 foraging */
  /**
   * @brief Update the arena_map on a block drop by distributing the block in a
   * new location and updating the block so that it no longer thinks it is
   * carried by a robot.
   *
   * @param map The map to update (there is only ever one...)
   */
  void visit(representation::perceived_arena_map& map) override;

    /**
   * @brief Update the FSM associated with a cell on a block drop.
   *
   * @param fsm The FSM associated with the cell to update.
   */
  void visit(representation::cell2D_fsm& fsm) override;

  /**
   * @brief Drop a carried block in the nest, updating state as appropriate.
   *
   * This needs to be here, rather than in the FSM, because dropping of blocks
   * needs to be done in the loop functions so the area can correctly be drawn
   * each timestep.
   */
  void visit(controller::depth1_foraging_controller& controller) override;
  void visit(controller::depth0_foraging_controller&) override {}

  /**
   * @brief Get the handle on the block that has been dropped.
   */
  const representation::cache* cache(void) const { return m_cache; }

 private:
  cache_found(const cache_found& op) = delete;
  cache_found& operator=(const cache_found& op) = delete;
  const representation::cache* m_cache;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CACHE_FOUND_HPP_ */
