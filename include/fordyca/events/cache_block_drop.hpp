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
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"
#include "rcppsw/types/discretize_ratio.hpp"

#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/events/block_drop_base_visit_set.hpp"
#include "fordyca/events/cell_op.hpp"
#include "fordyca/fordyca.hpp"
#include "fordyca/fsm/fsm_fwd.hpp"
#include "fordyca/tasks/tasks_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace repr {
class arena_cache;
} // namespace repr

namespace ds {
class dpo_semantic_map;
} // namespace ds

namespace controller {
class cache_sel_matrix;
} /* namespace controller */

NS_START(events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cache_block_drop
 * @ingroup fordyca events
 *
 * @brief Created whenever a robot drops a block in a cache.
 *
 * The cache usuage penalty, if there is one, is not assessed during the event,
 * but at a higher level.
 */
class cache_block_drop : public rer::client<cache_block_drop>,
                         public detail::cell_op {
 private:
  struct visit_typelist_impl {
    using inherited = boost::mpl::joint_view<block_drop_base_visit_typelist,
                                             cell_op::visit_typelist>;

    using controllers = boost::mpl::joint_view<controller::depth1::typelist,
                                               controller::depth2::typelist>;

    using others = rmpl::typelist<
        /* depth1 */
        fsm::block_to_goal_fsm,
        ds::dpo_semantic_map,
        repr::arena_cache,
        tasks::depth1::harvester,
        /* depth2 */
        tasks::depth2::cache_transferer>;

    using value = boost::mpl::joint_view<
        boost::mpl::joint_view<inherited::type, controllers::type>,
        others::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  /**
   * @brief Initialize a cache_block_drop event
   *
   * @param robot_block Block to drop (owned by robot).
   * @param cache Cache to drop into (owned by arena).
   * @param resolution Arena resolution.
   *
   * If you use this constructor, any visitation function is valid.
   */
  cache_block_drop(std::unique_ptr<repr::base_block> robot_block,
                   const std::shared_ptr<repr::arena_cache>& cache,
                   rtypes::discretize_ratio resolution);

  /**
   * @brief Initialize a cache_block_drop event
   *
   * @param robot_block Block to drop (owned by arena).
   * @param cache Cache to drop into (owned by arena).
   * @param resolution Arena resolution.
   *
   * If you use this constructor, only \ref arena_map visitation functions are
   * valid.
   */
  cache_block_drop(const std::shared_ptr<repr::base_block>& robot_block,
                   const std::shared_ptr<repr::arena_cache>& cache,
                   rtypes::discretize_ratio resolution);

  ~cache_block_drop(void) override = default;

  cache_block_drop(const cache_block_drop& op) = delete;
  cache_block_drop& operator=(const cache_block_drop& op) = delete;

  /* depth1 foraging */
  /**
   * @brief Perform actual cache block drop in the arena.
   *
   * Assumes \ref arena_map cache mutex held by the caller. Takes \ref arena_map
   * block mutex to perform block updates and releases afterwards.
   */
  void visit(ds::arena_map& map);
  void visit(class ds::cell2D& cell);
  void visit(fsm::cell2D_fsm& fsm);
  void visit(ds::dpo_semantic_map& map);
  void visit(repr::base_block& block);
  void visit(repr::arena_cache& cache);
  void visit(fsm::block_to_goal_fsm& fsm);
  void visit(tasks::depth1::harvester& task);
  void visit(controller::depth1::gp_dpo_controller& controller);
  void visit(controller::depth1::gp_mdpo_controller& controller);
  void visit(controller::depth1::gp_odpo_controller& controller);
  void visit(controller::depth1::gp_omdpo_controller& controller);

  /* depth2 foraging */
  void visit(controller::depth2::grp_dpo_controller& controller);
  void visit(controller::depth2::grp_mdpo_controller& controller);
  void visit(controller::depth2::grp_odpo_controller& controller);
  void visit(controller::depth2::grp_omdpo_controller& controller);
  void visit(tasks::depth2::cache_transferer& task);

 private:
  /* clang-format off */
  void dispatch_d1_cache_interactor(tasks::base_foraging_task* task);
  bool dispatch_d2_cache_interactor(tasks::base_foraging_task* task,
                                    controller::cache_sel_matrix* csel_matrix);

  const rtypes::discretize_ratio     mc_resolution;
  std::unique_ptr<repr::base_block>  m_robot_block;
  std::shared_ptr<repr::base_block>  m_arena_block;
  std::shared_ptr<repr::arena_cache> m_cache;
  /* clang-format on */
};

/**
 * @brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cache_block_drop_visitor_impl =
    rpvisitor::precise_visitor<detail::cache_block_drop,
                               detail::cache_block_drop::visit_typelist>;

NS_END(detail);

class cache_block_drop_visitor : public detail::cache_block_drop_visitor_impl {
  using detail::cache_block_drop_visitor_impl::cache_block_drop_visitor_impl;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CACHE_BLOCK_DROP_HPP_ */
