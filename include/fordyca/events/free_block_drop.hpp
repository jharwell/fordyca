/**
 * @file free_block_drop.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_FREE_BLOCK_DROP_HPP_
#define INCLUDE_FORDYCA_EVENTS_FREE_BLOCK_DROP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/er_client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"
#include "fordyca/representation/discrete_coord.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace visitor = rcppsw::patterns::visitor;
namespace representation {
class cell2D;
class cell2D_fsm;
class block;
}
NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class free_block_drop : public visitor::visitor,
                        public rcppsw::common::er_client,
                        public visitor::can_visit<representation::block>,
                        public visitor::can_visit<representation::cell2D>,
                        public visitor::can_visit<representation::cell2D_fsm> {
 public:
  free_block_drop(const std::shared_ptr<rcppsw::common::er_server>& server,
                  representation::block* block, double resolution);
  ~free_block_drop(void) { er_client::rmmod(); }

  /**
   * @brief Update a cell on a block drop.
   *
   * @param cell The cell to update.
   */
  void visit(class representation::cell2D& cell) override;

  /**
   * @brief Update the FSM associated with a cell on a block drop.
   *
   * @param fsm The FSM associated with the cell to update.
   */
  void visit(representation::cell2D_fsm& fsm) override;

  /**
   * @brief Update a block with the knowledge that it has been dropped.
   *
   * @param block The block to update.
   */
  void visit(representation::block& block) override;

  /**
   * @brief Get the handle on the block that has been dropped.
   */
  representation::block* block(void) const { return m_block; }

 private:
  free_block_drop(const free_block_drop& op) = delete;
  free_block_drop& operator=(const free_block_drop& op) = delete;

  double m_resolution;
  representation::block* m_block;
  representation::discrete_coord m_dloc;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_FREE_BLOCK_DROP_HPP_ */