/**
 * @file block_proximity.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_BLOCK_PROXIMITY_HPP_
#define INCLUDE_FORDYCA_EVENTS_BLOCK_PROXIMITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace controller { namespace depth2 {
class grp_mdpo_controller;
}} // namespace controller::depth2
namespace representation {
class base_block;
}
namespace fsm {
class block_to_goal_fsm;
} // namespace fsm
namespace tasks { namespace depth2 {
class cache_starter;
}} // namespace tasks::depth2

NS_START(events);
namespace visitor = rcppsw::patterns::visitor;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block_proximity
 * @ingroup events
 *
 * @brief Event that is created whenever a block that a robot is not currently
 * aware of blocks its ability to complete its current task.
 */
class block_proximity
    : public visitor::visit_set<controller::depth2::grp_mdpo_controller,
                                fsm::block_to_goal_fsm,
                                tasks::depth2::cache_starter>,
      public rcppsw::er::client<block_proximity> {
 public:
  explicit block_proximity(
      const std::shared_ptr<representation::base_block>& block);
  ~block_proximity(void) override = default;

  block_proximity(const block_proximity& op) = delete;
  block_proximity& operator=(const block_proximity& op) = delete;

  /* depth2 foraging */
  void visit(controller::depth2::grp_mdpo_controller& controller) override;
  void visit(fsm::block_to_goal_fsm& task) override;
  void visit(tasks::depth2::cache_starter& task) override;

 private:
  // clang-format off
  std::shared_ptr<representation::base_block> m_block;
  // clang-format on
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_BLOCK_PROXIMITY_HPP_ */
