/**
 * \file nest_block_drop.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"
#include "rcppsw/types/timestep.hpp"

#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/events/block_drop_base_visit_set.hpp"
#include "fordyca/fsm/fsm_fwd.hpp"
#include "fordyca/tasks/tasks_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class nest_block_drop
 * \ingroup fordyca events detail
 *
 * \brief Fired whenever a robot drops a block in the nest.
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

  /**
   * \brief Initialize a nest block drop event.
   *
   * \param robot_block Clone of arena block which it is giving up ownership of
   *                    for the drop.
   * \param t Current timestep.
   */
  nest_block_drop(std::unique_ptr<crepr::base_block2D> robot_block,
                  const rtypes::timestep& t);
  ~nest_block_drop(void) override = default;

  nest_block_drop(const nest_block_drop& op) = delete;
  nest_block_drop& operator=(const nest_block_drop& op) = delete;

  /* Depth0 DPO/MDPO foraging */

  /**
   * \brief Perform actual nest block drop in the arena.
   *
   * Internally takes \ref arena_map block, grid mutexes to protect block
   * re-distribution and block updates, and releases afterwards. See #594.
   */
  void visit(ds::arena_map& map);

  void visit(crepr::base_block2D& block);
  void visit(fsm::depth0::crw_fsm& fsm);
  void visit(controller::depth0::crw_controller& controller);
  void visit(fsm::depth0::dpo_fsm& fsm);
  void visit(controller::depth0::dpo_controller& controller);
  void visit(controller::depth0::mdpo_controller& controller);
  void visit(controller::depth0::odpo_controller& controller);
  void visit(controller::depth0::omdpo_controller& controller);

  /* Depth1 foraging */
  void visit(fsm::depth0::free_block_to_nest_fsm& fsm);
  void visit(controller::depth1::bitd_dpo_controller& controller);
  void visit(controller::depth1::bitd_mdpo_controller& controller);
  void visit(controller::depth1::bitd_odpo_controller& controller);
  void visit(controller::depth1::bitd_omdpo_controller& controller);
  void visit(fsm::depth1::cached_block_to_nest_fsm& fsm);
  void visit(tasks::depth1::collector& task);
  void visit(tasks::depth0::generalist& task);

  /* depth2 foraging */
  void visit(controller::depth2::birtd_dpo_controller& controller);
  void visit(controller::depth2::birtd_mdpo_controller& controller);
  void visit(controller::depth2::birtd_odpo_controller& controller);
  void visit(controller::depth2::birtd_omdpo_controller& controller);

 private:
  void dispatch_nest_interactor(tasks::base_foraging_task* task);

  /* clang-format off */
  const rtypes::timestep            mc_timestep;

  std::unique_ptr<crepr::base_block2D> m_robot_block;
  std::shared_ptr<crepr::base_block2D> m_arena_block{};
  /* clang-format on */
};

/**
 * \brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using nest_block_drop_visitor_impl =
    rpvisitor::precise_visitor<detail::nest_block_drop,
                               detail::nest_block_drop::visit_typelist>;

NS_END(detail);

class nest_block_drop_visitor : public detail::nest_block_drop_visitor_impl {
  using detail::nest_block_drop_visitor_impl::nest_block_drop_visitor_impl;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_NEST_BLOCK_DROP_HPP_ */
