/**
 * @file cached_block_pickup.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_CACHED_BLOCK_PICKUP_HPP_
#define INCLUDE_FORDYCA_EVENTS_CACHED_BLOCK_PICKUP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/events/block_pickup_event.hpp"
#include "fordyca/events/cell_op.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace visitor = rcppsw::patterns::visitor;
namespace fsm { namespace depth1 {
class block_to_goal_fsm;
class cached_block_to_nest_fsm;
}} // namespace fsm::depth1
namespace controller {
namespace depth1 {
class foraging_controller;
}
namespace depth2 {
class foraging_controller;
}
} // namespace controller
namespace representation {
class arena_cache;
}
namespace tasks {
namespace depth1 {
class collector;
}
namespace depth2 {
class cache_transferer;
}
} // namespace tasks

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/*
 * @class cached_block_pickup
 * @ingroup events
 *
 * @brief Created whenever a robpot picks up a block from a cache.
 *
 * The cache usage penalty, if there is one, is assessed prior to this event
 * being created, at a higher level.
 */
class cached_block_pickup
    : public cell_op,
      public rcppsw::er::client<cached_block_pickup>,
      public block_pickup_event,
      public visitor::visit_set<controller::depth1::foraging_controller,
                                controller::depth2::foraging_controller,
                                fsm::depth1::block_to_goal_fsm,
                                fsm::depth1::cached_block_to_nest_fsm,
                                tasks::depth1::collector,
                                tasks::depth2::cache_transferer,
                                representation::arena_cache> {
 public:
  cached_block_pickup(const std::shared_ptr<representation::arena_cache>& cache,
                      uint robot_index,
                      uint timestep);
  ~cached_block_pickup(void) override = default;

  cached_block_pickup(const cached_block_pickup& op) = delete;
  cached_block_pickup& operator=(const cached_block_pickup& op) = delete;

  /* depth1 foraging */
  void visit(ds::arena_map& map) override;
  void visit(ds::cell2D& cell) override;
  void visit(fsm::cell2D_fsm& fsm) override;
  void visit(ds::perceived_arena_map& map) override;
  void visit(representation::base_block& block) override;
  void visit(representation::arena_cache& cache) override;
  void visit(fsm::depth1::block_to_goal_fsm& fsm) override;
  void visit(fsm::depth1::cached_block_to_nest_fsm& fsm) override;
  void visit(controller::depth1::foraging_controller& controller) override;
  void visit(tasks::depth1::collector& task) override;

  /* depth2 foraging */
  void visit(controller::depth2::foraging_controller& controller) override;
  void visit(tasks::depth2::cache_transferer& task) override;

 private:
  // clang-format off
  uint                                         m_robot_index;
  uint                                         m_timestep;
  std::shared_ptr<representation::arena_cache> m_real_cache;

  /**
   * @brief The block that will be picked up by the robot.
   */
  std::shared_ptr<representation::base_block>  m_pickup_block{nullptr};

  /**
   * @brief The block that is left over when a cache devolves into a single
   * block, that needs to be sent to the cell that the cache used to live on.
   */
  std::shared_ptr<representation::base_block>  m_orphan_block{nullptr};
  // clang-format on
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CACHED_BLOCK_PICKUP_HPP_ */
