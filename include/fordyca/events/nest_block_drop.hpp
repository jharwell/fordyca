/**
 * @file nest_block_drop.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_NEST_BLOCK_DROP_HPP_
#define INCLUDE_FORDYCA_EVENTS_NEST_BLOCK_DROP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/events/block_drop_event.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace visitor = rcppsw::patterns::visitor;

namespace fsm {
namespace depth0 {
class crw_fsm;
class dpo_fsm;
class free_block_to_nest_fsm;
} // namespace depth0
namespace depth1 {
class block_to_cache_fsm;
class cached_block_to_nest_fsm;
} // namespace depth1
} // namespace fsm
namespace controller {
namespace depth0 {
class crw_controller;
class dpo_controller;
class mdpo_controller;
} // namespace depth0
namespace depth1 {
class gp_dpo_controller;
class gp_mdpo_controller;
}
namespace depth2 {
class grp_mdpo_controller;
}
} // namespace controller

namespace tasks {
namespace depth0 {
class generalist;
}
namespace depth1 {
class collector;
}
} // namespace tasks

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class nest_block_drop
 * @ingroup events
 *
 * @brief Fired whenever a robot drops a block in the nest.
 */
class nest_block_drop
    : public visitor::visitor,
      public block_drop_event,
      public rcppsw::er::client<nest_block_drop>,
      public visitor::visit_set<controller::depth0::crw_controller,
                                controller::depth0::dpo_controller,
                                controller::depth0::mdpo_controller,
                                controller::depth1::gp_dpo_controller,
                                controller::depth1::gp_mdpo_controller,
                                controller::depth2::grp_mdpo_controller,
                                fsm::depth0::crw_fsm,
                                fsm::depth0::dpo_fsm,
                                fsm::depth0::free_block_to_nest_fsm,
                                fsm::depth1::cached_block_to_nest_fsm,
                                tasks::depth0::generalist,
                                tasks::depth1::collector> {
 public:
  nest_block_drop(std::shared_ptr<representation::base_block> block,
                  uint timestep);
  ~nest_block_drop(void) override = default;

  nest_block_drop(const nest_block_drop& op) = delete;
  nest_block_drop& operator=(const nest_block_drop& op) = delete;

  /* Foraging support */
  void visit(ds::arena_map& map) override;

  /* Depth0 DPO/MDPO foraging */
  void visit(representation::base_block& block) override;
  void visit(fsm::depth0::crw_fsm& fsm) override;
  void visit(controller::depth0::crw_controller& controller) override;
  void visit(controller::depth0::dpo_controller& controller) override;
  void visit(fsm::depth0::dpo_fsm& fsm) override;
  void visit(controller::depth0::mdpo_controller& controller) override;

  /* Depth1 foraging */
  void visit(fsm::depth0::free_block_to_nest_fsm& fsm) override;
  void visit(controller::depth1::gp_dpo_controller& controller) override;
  void visit(controller::depth1::gp_mdpo_controller& controller) override;
  void visit(fsm::depth1::cached_block_to_nest_fsm& fsm) override;
  void visit(tasks::depth1::collector& task) override;
  void visit(tasks::depth0::generalist& task) override;

  /* depth2 foraging */
  void visit(controller::depth2::grp_mdpo_controller&) override;

  /**
   * @brief Get the handle on the block that has been dropped.
   */
  std::shared_ptr<representation::base_block> block(void) const {
    return m_block;
  }

 private:
  // clang-format off
  uint                                        m_timestep;
  std::shared_ptr<representation::base_block> m_block;
  // clang-format on
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_NEST_BLOCK_DROP_HPP_ */
