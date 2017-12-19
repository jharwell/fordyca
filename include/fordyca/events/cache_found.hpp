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

namespace representation { class cache; }

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/*
 * @class cache_found
 *
 * @brief Created whenever a NEW cache (i.e. one that is not currently known to
 * a robot, but possibly one that it has seen before and whose relevance had
 * expired) is discovered by the robot via it appearing in the robot's LOS.
 */
class cache_found : public perceived_cell_op,
                    public rcppsw::er::client {
 public:
  cache_found(const std::shared_ptr<rcppsw::er::server>& server,
              representation::cache* cache,
              size_t x, size_t y);
  ~cache_found(void) override;

  cache_found(const cache_found& op) = delete;
  cache_found& operator=(const cache_found& op) = delete;

  /* stateful foraging */
  void visit(representation::cell2D& cell) override;
  void visit(representation::perceived_cell2D& cell) override;

  /* depth1 foraging */
  void visit(representation::perceived_arena_map& map) override;
  void visit(fsm::cell2D_fsm& fsm) override;
  void visit(__unused controller::depth1::foraging_controller&) override {}
  void visit(__unused controller::depth0::stateful_foraging_controller&) override {}

  /**
   * @brief Get the handle on the cache that has been found.
   */
  /* const representation::cache* cache(void) const { return m_cache; } */

 private:
  representation::cache* m_cache;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CACHE_FOUND_HPP_ */
