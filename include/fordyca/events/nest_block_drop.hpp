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
#include "fordyca/events/block_drop_base_visit_set.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace rvisitor = rcppsw::patterns::visitor;

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
} // namespace depth1
namespace depth2 {
class grp_dpo_controller;
class grp_mdpo_controller;
} // namespace depth2
} // namespace controller

namespace tasks {
class base_foraging_task;
namespace depth0 {
class generalist;
}
namespace depth1 {
class collector;
}
} // namespace tasks

NS_START(events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
struct nest_block_drop_visit_set {
  using inherited = block_drop_base_visit_set::value;

  using defined = rvisitor::precise_visit_set<
      /* depth0 */
      controller::depth0::crw_controller,
      controller::depth0::dpo_controller,
      controller::depth0::mdpo_controller,
      fsm::depth0::crw_fsm,
      fsm::depth0::dpo_fsm,
      fsm::depth0::free_block_to_nest_fsm,
      /* depth1 */
      controller::depth1::gp_dpo_controller,
      controller::depth1::gp_mdpo_controller,
      fsm::depth1::cached_block_to_nest_fsm,
      tasks::depth0::generalist,
      tasks::depth1::collector,
      /* depth2 */
      controller::depth2::grp_dpo_controller,
      controller::depth2::grp_mdpo_controller>;

  using value = boost::mpl::joint_view<inherited::type, defined::type>;
};

/**
 * @class nest_block_drop
 * @ingroup fordyca events detail
 *
 * @brief Fired whenever a robot drops a block in the nest.
 */
class nest_block_drop : public rcppsw::er::client<nest_block_drop> {
 public:
  nest_block_drop(const std::shared_ptr<repr::base_block>& block, uint timestep);
  ~nest_block_drop(void) override = default;

  nest_block_drop(const nest_block_drop& op) = delete;
  nest_block_drop& operator=(const nest_block_drop& op) = delete;

  /* Foraging support */
  void visit(ds::arena_map& map);

  /* Depth0 DPO/MDPO foraging */
  void visit(repr::base_block& block);
  void visit(fsm::depth0::crw_fsm& fsm);
  void visit(controller::depth0::crw_controller& controller);
  void visit(controller::depth0::dpo_controller& controller);
  void visit(fsm::depth0::dpo_fsm& fsm);
  void visit(controller::depth0::mdpo_controller& controller);

  /* Depth1 foraging */
  void visit(fsm::depth0::free_block_to_nest_fsm& fsm);
  void visit(controller::depth1::gp_dpo_controller& controller);
  void visit(controller::depth1::gp_mdpo_controller& controller);
  void visit(fsm::depth1::cached_block_to_nest_fsm& fsm);
  void visit(tasks::depth1::collector& task);
  void visit(tasks::depth0::generalist& task);

  /* depth2 foraging */
  void visit(controller::depth2::grp_mdpo_controller&);

  /**
   * @brief Get the handle on the block that has been dropped.
   */
  std::shared_ptr<repr::base_block> block(void) const { return m_block; }

 private:
  void dispatch_nest_interactor(tasks::base_foraging_task* task);

  /* clang-format off */
  uint                              m_timestep;
  std::shared_ptr<repr::base_block> m_block;
  /* clang-format on */
};

/**
 * @brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using nest_block_drop_visitor_impl =
    rvisitor::precise_visitor<detail::nest_block_drop,
                             detail::nest_block_drop_visit_set::value>;

NS_END(detail);

class nest_block_drop_visitor : public detail::nest_block_drop_visitor_impl {
  using detail::nest_block_drop_visitor_impl::nest_block_drop_visitor_impl;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_NEST_BLOCK_DROP_HPP_ */
