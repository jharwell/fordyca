/**
 * \file robot_free_block_drop.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_ROBOT_FREE_BLOCK_DROP_HPP_
#define INCLUDE_FORDYCA_EVENTS_ROBOT_FREE_BLOCK_DROP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"

#include "cosm/events/cell2D_op.hpp"

#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/fsm/fsm_fwd.hpp"
#include "fordyca/tasks/tasks_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class base_block2D;
} /* namespace cosm::repr */

namespace fordyca::controller {
class block_sel_matrix;
} /* namespace fordyca::controller */

namespace fordyca::ds {
class dpo_semantic_map;
} /* namespace fordyca::ds */

NS_START(fordyca, events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class robot_free_block_drop
 * \ingroup events detail
 *
 * \brief Created whenever a block is dropped somewhere in the arena that is not
 * a cache or the nest by a robot. Complement to \ref arena_block_drop to handle
 * updates needed by robot controllers.
 *
 * This can happen when:
 *
 * - A robot aborts its task, and is carrying a block.
 */
class robot_free_block_drop : public rer::client<robot_free_block_drop>,
                              public cevents::cell2D_op {
 private:
  struct visit_typelist_impl {
    using inherited = cevents::cell2D_op::visit_typelist;
    using controllers = boost::mpl::joint_view<
        boost::mpl::joint_view<controller::depth0::typelist,
                               controller::depth1::typelist>,
        controller::depth2::typelist>;

    using others = rmpl::typelist<
        /* depth0 */
        fsm::block_to_goal_fsm,
        ds::dpo_semantic_map,
        /* depth2 */
        tasks::depth2::cache_starter,
        tasks::depth2::cache_finisher,
        fsm::block_to_goal_fsm,
        ds::dpo_semantic_map>;

    using value = boost::mpl::joint_view<
        boost::mpl::joint_view<controllers::type, others::type>,
        inherited>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  ~robot_free_block_drop(void) override = default;

  robot_free_block_drop(const robot_free_block_drop& op) = delete;
  robot_free_block_drop& operator=(const robot_free_block_drop& op) = delete;

  /* depth1 */
  void visit(class cds::cell2D& cell);
  void visit(cfsm::cell2D_fsm& fsm);
  void visit(crepr::base_block2D& block);
  void visit(controller::depth1::bitd_dpo_controller&) {}
  void visit(controller::depth1::bitd_mdpo_controller&) {}
  void visit(controller::depth1::bitd_odpo_controller&) {}
  void visit(controller::depth1::bitd_omdpo_controller&) {}

  /* depth2 */
  void visit(controller::depth2::birtd_dpo_controller&);
  void visit(controller::depth2::birtd_mdpo_controller&);
  void visit(controller::depth2::birtd_odpo_controller&);
  void visit(controller::depth2::birtd_omdpo_controller&);
  void visit(tasks::depth2::cache_starter&);
  void visit(tasks::depth2::cache_finisher&);
  void visit(fsm::block_to_goal_fsm&);
  void visit(ds::dpo_semantic_map& map);

  /**
   * \brief Get the handle on the block that has been dropped.
   */
  std::shared_ptr<crepr::base_block2D> block(void) const { return m_block; }

 protected:
  /**
   * \param block The block to drop, which was owned by the robot.
   * \param coord The discrete coordinates of the cell to drop the block in.
   * \param resolution The resolution of the arena map.
   */
  robot_free_block_drop(std::unique_ptr<crepr::base_block2D> block,
                        const rmath::vector2u& coord,
                        const rtypes::discretize_ratio& resolution);

 private:
  bool dispatch_free_block_interactor(tasks::base_foraging_task* task,
                                      controller::block_sel_matrix* bsel_matrix);

  /* clang-format off */
  const rtypes::discretize_ratio       mc_resolution;

  std::shared_ptr<crepr::base_block2D> m_block;
  /* clang-format on */
};

/**
 * \brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using robot_free_block_drop_visitor_impl =
    rpvisitor::precise_visitor<detail::robot_free_block_drop,
                               detail::robot_free_block_drop::visit_typelist>;

NS_END(detail);

class robot_free_block_drop_visitor
    : public detail::robot_free_block_drop_visitor_impl {
  using detail::robot_free_block_drop_visitor_impl::robot_free_block_drop_visitor_impl;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_ROBOT_FREE_BLOCK_DROP_HPP_ */
