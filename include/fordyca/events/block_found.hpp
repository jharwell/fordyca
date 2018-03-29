/**
 * @file block_found.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_BLOCK_FOUND_HPP_
#define INCLUDE_FORDYCA_EVENTS_BLOCK_FOUND_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/events/perceived_cell_op.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation {
class block;
}

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block_found
 * @ingroup events
 *
 * @brief Event that is created whenever a NEW block (i.e. one that is not
 * currently known to the robot, but that has possibly been seen before and had
 * its relevance expire) is found via appearing in a robot's LOS. These events
 * are not processed by the \ref arena_map, and exist only in a robot's
 * perception.
 */
class block_found : public perceived_cell_op, public rcppsw::er::client {
 public:
  block_found(const std::shared_ptr<rcppsw::er::server>& server,
              std::unique_ptr<representation::block> block);
  ~block_found(void) override;

  block_found(const block_found& op) = delete;
  block_found& operator=(const block_found& op) = delete;

  /* stateful foraging */
  void visit(representation::cell2D& cell) override;
  void visit(fsm::cell2D_fsm& fsm) override;
  void visit(controller::depth0::stateful_foraging_controller&) override {}
  void visit(representation::perceived_arena_map& map) override;

  /* depth1 foraging */
  void visit(controller::depth1::foraging_controller&) override {}

 private:
  // clang-format off
  std::shared_ptr<representation::block> m_block;
  // clang-format on
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_BLOCK_FOUND_HPP_ */
