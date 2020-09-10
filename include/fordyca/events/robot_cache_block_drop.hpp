/**
 * \file robot_cache_block_drop.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_ROBOT_CACHE_BLOCK_DROP_HPP_
#define INCLUDE_FORDYCA_EVENTS_ROBOT_CACHE_BLOCK_DROP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"
#include "rcppsw/types/discretize_ratio.hpp"

#include "cosm/ds/operations/cell2D_op.hpp"
#include "cosm/repr/base_block3D.hpp"

#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/fordyca.hpp"
#include "fordyca/fsm/fsm_fwd.hpp"
#include "fordyca/tasks/tasks_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::repr {
class arena_cache;
} // namespace cosm::arena::repr

namespace fordyca::ds {
class dpo_semantic_map;
} // namespace fordyca::ds

namespace fordyca::controller::cognitive {
class cache_sel_matrix;
} // namespace fordyca::controller

NS_START(fordyca, events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class robot_cache_block_drop
 * \ingroup events detail
 *
 * \brief Created whenever a robot drops a block in a cache to handle updates
 * needed by robot controllers. Complement to \ref arena_robot_cache_block_drop.
 */
class robot_cache_block_drop : public rer::client<robot_cache_block_drop>,
                               public cdops::cell2D_op {
 private:
  struct visit_typelist_impl {
    using inherited = cdops::cell2D_op::visit_typelist;
    using controllers = boost::mpl::joint_view<controller::depth1::typelist,
                                               controller::depth2::typelist>;

    using others = rmpl::typelist<
        /* depth1 */
        fsm::block_to_goal_fsm,
        ds::dpo_semantic_map,
        carepr::arena_cache,
        tasks::depth1::harvester,
        /* depth2 */
        tasks::depth2::cache_transferer>;

    using value = boost::mpl::joint_view<
        boost::mpl::joint_view<others::type, controllers::type>,
        inherited::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  ~robot_cache_block_drop(void) override = default;

  robot_cache_block_drop(const robot_cache_block_drop& op) = delete;
  robot_cache_block_drop& operator=(const robot_cache_block_drop& op) = delete;

  /* depth1 foraging */
  void visit(class cds::cell2D& cell);
  void visit(cfsm::cell2D_fsm& fsm);

  void visit(ds::dpo_semantic_map& map);
  void visit(fsm::block_to_goal_fsm& fsm);
  void visit(tasks::depth1::harvester& task);
  void visit(controller::cognitive::depth1::bitd_dpo_controller& controller);
  void visit(controller::cognitive::depth1::bitd_mdpo_controller& controller);
  void visit(controller::cognitive::depth1::bitd_odpo_controller& controller);
  void visit(controller::cognitive::depth1::bitd_omdpo_controller& controller);

  /* depth2 foraging */
  void visit(controller::cognitive::depth2::birtd_dpo_controller& controller);
  void visit(controller::cognitive::depth2::birtd_mdpo_controller& controller);
  void visit(controller::cognitive::depth2::birtd_odpo_controller& controller);
  void visit(controller::cognitive::depth2::birtd_omdpo_controller& controller);
  void visit(tasks::depth2::cache_transferer& task);

 protected:
  /**
   * \brief Initialize a robot_cache_block_drop event
   *
   * \param block Block to drop (owned by robot).
   * \param cache Cache to drop into (owned by arena).
   * \param resolution Arena resolution.
   */
  robot_cache_block_drop(std::unique_ptr<crepr::base_block3D> block,
                         carepr::arena_cache* cache,
                         const rtypes::discretize_ratio& resolution);

 private:
  /* clang-format off */
  void dispatch_d1_cache_interactor(tasks::base_foraging_task* task);
  bool dispatch_d2_cache_interactor(tasks::base_foraging_task* task,
                                    controller::cognitive::cache_sel_matrix* csel_matrix);

  const rtypes::discretize_ratio       mc_resolution;

  std::unique_ptr<crepr::base_block3D> m_block;
  carepr::arena_cache*                 m_cache;
  /* clang-format on */
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using robot_cache_block_drop_visitor_impl =
    rpvisitor::precise_visitor<detail::robot_cache_block_drop,
                               detail::robot_cache_block_drop::visit_typelist>;

NS_END(detail);

class robot_cache_block_drop_visitor
    : public detail::robot_cache_block_drop_visitor_impl {
 public:
  using detail::robot_cache_block_drop_visitor_impl::robot_cache_block_drop_visitor_impl;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_ROBOT_CACHE_BLOCK_DROP_HPP_ */
