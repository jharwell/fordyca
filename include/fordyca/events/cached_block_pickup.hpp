/**
 * \file cached_block_pickup.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_CACHED_BLOCK_PICKUP_HPP_
#define INCLUDE_FORDYCA_EVENTS_CACHED_BLOCK_PICKUP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/types/timestep.hpp"

#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/events/block_pickup_base_visit_set.hpp"
#include "fordyca/events/cell_op.hpp"
#include "fordyca/fsm/fsm_fwd.hpp"
#include "fordyca/tasks/tasks_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace controller {
class cache_sel_matrix;
}
namespace repr {
class arena_cache;
}
namespace support {
class base_cache_manager;
class base_loop_functions;
} /* namespace support */

NS_START(events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cached_block_pickup
 * \ingroup fordyca events detail
 *
 * \brief Created whenever a robpot picks up a block from a cache.
 *
 * The cache usage penalty, if there is one, is assessed prior to this event
 * being created, at a higher level.
 */
class cached_block_pickup : public rer::client<cached_block_pickup>,
                            public cell_op {
 private:
  struct visit_typelist_impl {
    using inherited =
        boost::mpl::joint_view<cell_op::visit_typelist::type,
                               block_pickup_base_visit_typelist::type>;
    using controllers =
        boost::mpl::joint_view<controller::depth1::typelist::type,
                               controller::depth2::typelist::type>;
    using others = rmpl::typelist<
        /* depth1 */
        fsm::block_to_goal_fsm,
        fsm::depth1::cached_block_to_nest_fsm,
        tasks::depth1::collector,
        support::base_cache_manager,
        /* depth2 */
        tasks::depth2::cache_transferer,
        tasks::depth2::cache_collector,
        repr::arena_cache>;

    using value =
        boost::mpl::joint_view<boost::mpl::joint_view<inherited, controllers>,
                               others::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  cached_block_pickup(support::base_loop_functions* loop,
                      const std::shared_ptr<repr::arena_cache>& cache,
                      uint robot_index,
                      const rtypes::timestep& t);
  ~cached_block_pickup(void) override = default;

  cached_block_pickup(const cached_block_pickup& op) = delete;
  cached_block_pickup& operator=(const cached_block_pickup& op) = delete;

  /* depth1 foraging */
  /**
   * \brief Perform actual cache block pickup in the arena.
   *
   * Assumes caller is holding \ref arena_map cache mutex. Takes \ref arena_map
   * block mutex, and then releases it after cache updates.
   */
  void visit(ds::arena_map& map);

  void visit(ds::cell2D& cell);
  void visit(fsm::cell2D_fsm& fsm);
  void visit(ds::dpo_semantic_map& map);
  void visit(ds::dpo_store& store);
  void visit(crepr::base_block2D& block);
  void visit(repr::arena_cache& cache);
  void visit(tasks::depth1::collector& task);
  void visit(fsm::block_to_goal_fsm& fsm);
  void visit(fsm::depth1::cached_block_to_nest_fsm& fsm);
  void visit(controller::depth1::bitd_dpo_controller& controller);
  void visit(controller::depth1::bitd_mdpo_controller& controller);
  void visit(controller::depth1::bitd_odpo_controller& controller);
  void visit(controller::depth1::bitd_omdpo_controller& controller);

  /**
   * \brief Update the cache manager upon cache pickup. Protected by mutex in
   * order to ensure consistency between concurrent robot updates if multiple
   * caches are present in the arena.
   */
  void visit(support::base_cache_manager& manager);

  /* depth2 foraging */
  void visit(controller::depth2::birtd_dpo_controller& controller);
  void visit(controller::depth2::birtd_mdpo_controller& controller);
  void visit(controller::depth2::birtd_odpo_controller& controller);
  void visit(controller::depth2::birtd_omdpo_controller& controller);
  void visit(tasks::depth2::cache_transferer& task);
  void visit(tasks::depth2::cache_collector& task);

 private:
  void dispatch_d1_cache_interactor(tasks::base_foraging_task* task);
  bool dispatch_d2_cache_interactor(tasks::base_foraging_task* task,
                                    controller::cache_sel_matrix* csel_matrix);

  /* clang-format off */
  const uint                         mc_robot_index;
  const rtypes::timestep             mc_timestep;

  support::base_loop_functions       *m_loop;
  std::shared_ptr<repr::arena_cache> m_real_cache;

  /**
   * \brief The block that will be picked up by the robot.
   */
  std::shared_ptr<crepr::base_block2D>  m_pickup_block{nullptr};

  /**
   * \brief The block that is left over when a cache devolves into a single
   * block, that needs to be sent to the cell that the cache used to live on.
   */
  std::shared_ptr<crepr::base_block2D>  m_orphan_block{nullptr};
  /* clang-format on */
};

/**
 * \brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cached_block_pickup_visitor_impl =
    rpvisitor::precise_visitor<detail::cached_block_pickup,
                               detail::cached_block_pickup::visit_typelist>;

NS_END(detail);

class cached_block_pickup_visitor
    : public detail::cached_block_pickup_visitor_impl {
  using detail::cached_block_pickup_visitor_impl::cached_block_pickup_visitor_impl;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CACHED_BLOCK_PICKUP_HPP_ */
