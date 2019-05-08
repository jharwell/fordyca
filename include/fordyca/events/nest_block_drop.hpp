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
#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/events/block_drop_base_visit_set.hpp"
#include "fordyca/fsm/fsm_fwd.hpp"
#include "fordyca/tasks/tasks_fwd.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class nest_block_drop
 * @ingroup fordyca events detail
 *
 * @brief Fired whenever a robot drops a block in the nest.
 */
class nest_block_drop : public rer::client<nest_block_drop> {
 private:
  struct visit_typelist_impl {
    using inherited = block_drop_base_visit_typelist;
    using controllers = boost::mpl::joint_view<
        boost::mpl::joint_view<controller::depth0::typelist,
                               controller::depth1::typelist>,
        controller::depth2::typelist>;

    using fsms = rmpl::typelist<fsm::depth0::crw_fsm,
                                fsm::depth0::dpo_fsm,
                                fsm::depth0::free_block_to_nest_fsm,
                                fsm::depth1::cached_block_to_nest_fsm>;
    using tasks =
        rmpl::typelist<tasks::depth0::generalist, tasks::depth1::collector>;

    using value = boost::mpl::joint_view<
        boost::mpl::joint_view<boost::mpl::joint_view<controllers::type, tasks::type>,
                               fsms::type>,
        boost::mpl::joint_view<inherited::type, controllers::type> >;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

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
  void visit(fsm::depth0::dpo_fsm& fsm);
  void visit(controller::depth0::dpo_controller& controller);
  void visit(controller::depth0::mdpo_controller& controller);
  void visit(controller::depth0::odpo_controller& controller);
  void visit(controller::depth0::omdpo_controller& controller);

  /* Depth1 foraging */
  void visit(fsm::depth0::free_block_to_nest_fsm& fsm);
  void visit(controller::depth1::gp_dpo_controller& controller);
  void visit(controller::depth1::gp_mdpo_controller& controller);
  void visit(controller::depth1::gp_odpo_controller& controller);
  void visit(controller::depth1::gp_omdpo_controller& controller);
  void visit(fsm::depth1::cached_block_to_nest_fsm& fsm);
  void visit(tasks::depth1::collector& task);
  void visit(tasks::depth0::generalist& task);

  /* depth2 foraging */
  void visit(controller::depth2::grp_dpo_controller& controller);
  void visit(controller::depth2::grp_mdpo_controller& controller);
  void visit(controller::depth2::grp_odpo_controller& controller);
  void visit(controller::depth2::grp_omdpo_controller& controller);

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
                              detail::nest_block_drop::visit_typelist>;

NS_END(detail);

class nest_block_drop_visitor : public detail::nest_block_drop_visitor_impl {
  using detail::nest_block_drop_visitor_impl::nest_block_drop_visitor_impl;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_NEST_BLOCK_DROP_HPP_ */
