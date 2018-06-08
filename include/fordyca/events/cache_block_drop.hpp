/**
 * @file cache_block_drop.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_CACHE_BLOCK_DROP_HPP_
#define INCLUDE_FORDYCA_EVENTS_CACHE_BLOCK_DROP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/events/block_drop_event.hpp"
#include "fordyca/events/cell_op.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace visitor = rcppsw::patterns::visitor;
namespace representation {
class perceived_arena_map;
class arena_cache;
} // namespace representation
namespace fsm { namespace depth1 {
class block_to_goal_fsm;
}} // namespace fsm::depth1
namespace tasks { namespace depth1 {
class harvester;
}}

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cache_block_drop
 * @ingroup events
 *
 * @brief Created whenever a robot drops a block in a cache.
 *
 * The cache usuage penalty, if there is one, is not assessed during the event,
 * but at a higher level.
 */
class cache_block_drop
    : public cell_op,
      public rcppsw::er::client,
      public block_drop_event,
      public visitor::visit_set<tasks::depth1::harvester,
                                fsm::depth1::block_to_goal_fsm,
                                representation::perceived_arena_map,
                                representation::arena_cache> {
 public:
  cache_block_drop(const std::shared_ptr<rcppsw::er::server>& server,
                   const std::shared_ptr<representation::block>& block,
                   const std::shared_ptr<representation::arena_cache>& cache,
                   double resolution);
  ~cache_block_drop(void) override { client::rmmod(); }

  cache_block_drop(const cache_block_drop& op) = delete;
  cache_block_drop& operator=(const cache_block_drop& op) = delete;

  /* depth1 foraging */
  void visit(class representation::cell2D& cell) override;
  void visit(fsm::cell2D_fsm& fsm) override;
  void visit(representation::arena_map& map) override;
  void visit(representation::perceived_arena_map& map) override;
  void visit(representation::block& block) override;
  void visit(representation::arena_cache& cache) override;
  void visit(controller::depth1::foraging_controller& controller) override;
  void visit(fsm::depth1::block_to_goal_fsm& fsm) override;
  void visit(tasks::depth1::harvester& task) override;

  /* depth2 foraging */
  void visit(controller::depth2::foraging_controller&) override {}

 private:
  // clang-format off
  double                                       m_resolution;
  std::shared_ptr<representation::block>       m_block;
  std::shared_ptr<representation::arena_cache> m_cache;
  std::shared_ptr<rcppsw::er::server>          m_server;
  // clang-format on
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CACHE_BLOCK_DROP_HPP_ */
