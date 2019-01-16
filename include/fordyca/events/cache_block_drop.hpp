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
#include "fordyca/events/block_drop_event.hpp"
#include "fordyca/events/cell_op.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace visitor = rcppsw::patterns::visitor;
namespace controller {
namespace depth1 {
class gp_dpo_controller;
class gp_mdpo_controller;
} // namespace depth1
namespace depth2 {
class grp_mdpo_controller;
}
} // namespace controller

namespace representation {
class arena_cache;
} // namespace representation

namespace ds {
class dpo_semantic_map;
} // namespace ds
namespace fsm {
class block_to_goal_fsm;
} // namespace fsm
namespace tasks {
namespace depth1 {
class harvester;
}
namespace depth2 {
class cache_transferer;
}
} // namespace tasks

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cache_block_drop
 * @ingroup events
 *
 * @brief Created whenever a robot drops a block in a cache.
 *
 * The cache usuage penalty, if there is one, is not assessed during the event,
 * but at a higher level.
 */
class cache_block_drop
    : public cell_op,
      public rcppsw::er::client<cache_block_drop>,
      public block_drop_event,
      public visitor::visit_set<controller::depth1::gp_dpo_controller,
                                controller::depth1::gp_mdpo_controller,
                                controller::depth2::grp_mdpo_controller,
                                tasks::depth1::harvester,
                                tasks::depth2::cache_transferer,
                                fsm::block_to_goal_fsm,
                                ds::dpo_semantic_map,
                                representation::arena_cache> {
 public:
  cache_block_drop(const std::shared_ptr<representation::base_block>& block,
                   const std::shared_ptr<representation::arena_cache>& cache,
                   double resolution);
  ~cache_block_drop(void) override = default;

  cache_block_drop(const cache_block_drop& op) = delete;
  cache_block_drop& operator=(const cache_block_drop& op) = delete;

  /* depth1 foraging */
  void visit(class ds::cell2D& cell) override;
  void visit(fsm::cell2D_fsm& fsm) override;
  void visit(ds::arena_map& map) override;
  void visit(ds::dpo_semantic_map& map) override;
  void visit(representation::base_block& block) override;
  void visit(representation::arena_cache& cache) override;
  void visit(fsm::block_to_goal_fsm& fsm) override;
  void visit(tasks::depth1::harvester& task) override;
  void visit(controller::depth1::gp_dpo_controller& controller) override;
  void visit(controller::depth1::gp_mdpo_controller& controller) override;

  /* depth2 foraging */
  void visit(controller::depth2::grp_mdpo_controller&) override;
  void visit(tasks::depth2::cache_transferer& task) override;

 private:
  /* clang-format off */
  double                                       m_resolution;
  std::shared_ptr<representation::base_block>  m_block;
  std::shared_ptr<representation::arena_cache> m_cache;
  /* clang-format on */
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CACHE_BLOCK_DROP_HPP_ */
