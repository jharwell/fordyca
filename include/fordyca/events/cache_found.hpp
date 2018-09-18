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
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation {
class base_cache;
}

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/*
 * @class cache_found
 * @ingroup events
 *
 * @brief Created whenever a NEW cache (i.e. one that is not currently known to
 * a robot, but possibly one that it has seen before and whose relevance had
 * expired) is discovered by the robot via it appearing in the robot's LOS.
 */
class cache_found : public perceived_cell_op,
                    public rcppsw::er::client<cache_found> {
 public:
  explicit cache_found(std::unique_ptr<representation::base_cache> cache);
  ~cache_found(void) override = default;

  cache_found(const cache_found& op) = delete;
  cache_found& operator=(const cache_found& op) = delete;

  /* stateful foraging */
  void visit(ds::cell2D& cell) override;

  /* depth1 foraging */
  void visit(ds::perceived_arena_map& map) override;
  void visit(fsm::cell2D_fsm& fsm) override;
  void visit(controller::depth1::foraging_controller&) override {}
  void visit(controller::depth0::stateful_foraging_controller&) override {}

 private:
  std::shared_ptr<representation::base_cache> m_cache;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CACHE_FOUND_HPP_ */
