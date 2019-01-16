/**
 * @file free_block_drop.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_FREE_BLOCK_DROP_HPP_
#define INCLUDE_FORDYCA_EVENTS_FREE_BLOCK_DROP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/events/block_drop_event.hpp"
#include "fordyca/events/cell_op.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace visitor = rcppsw::patterns::visitor;
namespace rmath = rcppsw::math;
namespace ds {
class dpo_semantic_map;
}
namespace fsm {
class block_to_goal_fsm;
} // namespace fsm
namespace controller {
namespace depth1 {
class gp_dpo_controller;
class gp_mdpo_controller;
} // namespace depth1
namespace depth2 {
class grp_mdpo_controller;
}
} // namespace controller
namespace tasks { namespace depth2 {
class cache_starter;
class cache_finisher;
}} // namespace tasks::depth2
NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class free_block_drop
 * @ingroup events
 *
 * @brief Created whenever a block is dropped somewhere in the arena that is not
 * a cache or the nest.
 *
 * This can happen when:
 *
 * - The loop functions are doing block distribution.
 * - A robot aborts its task, and is carrying a block.
 */
class free_block_drop
    : public cell_op,
      public rcppsw::er::client<free_block_drop>,
      public block_drop_event,
      public visitor::visit_set<controller::depth1::gp_dpo_controller,
                                controller::depth1::gp_mdpo_controller,
                                controller::depth2::grp_mdpo_controller,
                                tasks::depth2::cache_starter,
                                tasks::depth2::cache_finisher,
                                fsm::block_to_goal_fsm,
                                ds::dpo_semantic_map> {
 public:
  /**
   * @param block The block to drop.
   * @param coord The discrete coordinates of the cell to drop the block in.
   * @param resolution The resolution of the arena map.
   */
  free_block_drop(const std::shared_ptr<representation::base_block>& block,
                  const rmath::vector2u& coord,
                  double resolution);
  ~free_block_drop(void) override = default;

  free_block_drop(const free_block_drop& op) = delete;
  free_block_drop& operator=(const free_block_drop& op) = delete;

  /* depth0 foraging */
  void visit(ds::cell2D& cell) override;
  void visit(representation::base_block& block) override;
  void visit(fsm::cell2D_fsm& fsm) override;
  void visit(ds::arena_map& map) override;

  /* depth1 foraging */
  void visit(controller::depth1::gp_dpo_controller&) override;
  void visit(controller::depth1::gp_mdpo_controller&) override;

  /* depth2 foraging */
  void visit(controller::depth2::grp_mdpo_controller&) override;
  void visit(tasks::depth2::cache_starter&) override;
  void visit(tasks::depth2::cache_finisher&) override;
  void visit(fsm::block_to_goal_fsm&) override;
  void visit(ds::dpo_semantic_map& map) override;

  /**
   * @brief Get the handle on the block that has been dropped.
   */
  std::shared_ptr<representation::base_block> block(void) const {
    return m_block;
  }

 private:
  /* clang-format off */
  double                                      m_resolution;
  std::shared_ptr<representation::base_block> m_block;
  /* clang-format on */
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_FREE_BLOCK_DROP_HPP_ */
