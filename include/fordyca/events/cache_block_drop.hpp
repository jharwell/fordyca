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
#include "fordyca/events/block_drop_base_visit_set.hpp"
#include "fordyca/events/cell_op.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace rvisitor = rcppsw::patterns::visitor;
namespace controller {
class cache_sel_matrix;
namespace depth1 {
class gp_dpo_controller;
class gp_mdpo_controller;
} // namespace depth1
namespace depth2 {
class grp_dpo_controller;
class grp_mdpo_controller;
} // namespace depth2
} // namespace controller

namespace repr {
class arena_cache;
} // namespace repr

namespace ds {
class dpo_semantic_map;
} // namespace ds
namespace fsm {
class block_to_goal_fsm;
} // namespace fsm
namespace tasks {
class base_foraging_task;
namespace depth1 {
class harvester;
}
namespace depth2 {
class cache_transferer;
}
} // namespace tasks

NS_START(events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
struct cache_block_drop_visit_set {
  using inherited = boost::mpl::joint_view<block_drop_base_visit_set::value,
                                           cell_op_visit_set::value>;

  using defined = rvisitor::precise_visit_set<
      /* depth1 */
      fsm::block_to_goal_fsm,
      ds::dpo_semantic_map,
      repr::arena_cache,
      controller::depth1::gp_dpo_controller,
      controller::depth1::gp_mdpo_controller,
      tasks::depth1::harvester,
      /* depth2 */
      controller::depth2::grp_dpo_controller,
      controller::depth2::grp_mdpo_controller,
      tasks::depth2::cache_transferer>;

  using value = boost::mpl::joint_view<inherited::type, defined::type>;
};

/**
 * @class cache_block_drop
 * @ingroup fordyca events
 *
 * @brief Created whenever a robot drops a block in a cache.
 *
 * The cache usuage penalty, if there is one, is not assessed during the event,
 * but at a higher level.
 */
class cache_block_drop : public rcppsw::er::client<cache_block_drop>,
                         public detail::cell_op {
 public:
  cache_block_drop(const std::shared_ptr<repr::base_block>& block,
                   const std::shared_ptr<repr::arena_cache>& cache,
                   double resolution);
  ~cache_block_drop(void) override = default;

  cache_block_drop(const cache_block_drop& op) = delete;
  cache_block_drop& operator=(const cache_block_drop& op) = delete;

  /* depth1 foraging */
  void visit(class ds::cell2D& cell);
  void visit(fsm::cell2D_fsm& fsm);
  void visit(ds::arena_map& map);
  void visit(ds::dpo_semantic_map& map);
  void visit(repr::base_block& block);
  void visit(repr::arena_cache& cache);
  void visit(fsm::block_to_goal_fsm& fsm);
  void visit(tasks::depth1::harvester& task);
  void visit(controller::depth1::gp_dpo_controller& controller);
  void visit(controller::depth1::gp_mdpo_controller& controller);

  /* depth2 foraging */
  void visit(controller::depth2::grp_dpo_controller&);
  void visit(controller::depth2::grp_mdpo_controller&);
  void visit(tasks::depth2::cache_transferer& task);

 private:
  /* clang-format off */
  void dispatch_d1_cache_interactor(tasks::base_foraging_task* task);
  bool dispatch_d2_cache_interactor(tasks::base_foraging_task* task,
                                    controller::cache_sel_matrix* csel_matrix);

  double                             m_resolution;
  std::shared_ptr<repr::base_block>  m_block;
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
    rvisitor::precise_visitor<detail::cache_block_drop,
                             detail::cache_block_drop_visit_set::value>;

NS_END(detail);

class cache_block_drop_visitor : public detail::cache_block_drop_visitor_impl {
  using detail::cache_block_drop_visitor_impl::cache_block_drop_visitor_impl;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CACHE_BLOCK_DROP_HPP_ */
