/**
 * @file depth2_loop_functions.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH2_DEPTH2_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH2_DEPTH2_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include "fordyca/support/depth1/depth1_loop_functions.hpp"
#include "fordyca/tasks/depth2/foraging_task.hpp"
#include "fordyca/support/depth2/robot_arena_interactor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);
class depth2_metrics_aggregator;
class dynamic_cache_manager;

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class loop_functions
 * @ingroup support depth2
 *
 * @brief The loop functions for depth 2 foraging.
 *
 * Handles all operations robots perform relating to dynamic caches: pickup,
 * drop, creation, depletion, etc.
 */
class depth2_loop_functions : public depth1::depth1_loop_functions,
                              public er::client<depth2_loop_functions> {
 public:
  depth2_loop_functions(void);
  ~depth2_loop_functions(void) override;

  void Init(ticpp::Element& node) override;
  void PreStep() override;
  void Reset(void) override;

 private:
  using interactor = robot_arena_interactor<controller::depth2::greedy_recpart_controller>;

  /**
   * @brief Handle creation of dynamic caches during initialization, reset, or
   * when triggered by events during simulation.
   *
   * @param on_drop \c TRUE if caches are to be (potentially) created as a
   * result of a robot block drop. If \c FALSE, then consider dynamic cache
   * creation in other situations.
   *
   * @return \c TRUE if one or more caches were creation, \c FALSE otherwise.
   */
  bool cache_creation_handle(bool on_drop);

  void pre_step_final(void) override;
  void pre_step_iter(argos::CFootBotEntity& robot);
  argos::CColor GetFloorColor(const argos::CVector2& plane_pos) override;
  void controller_configure(controller::base_controller& c);
  void cache_handling_init(const struct params::caches::caches_params* cachep);

  // clang-format off
  std::unique_ptr<interactor>                m_interactor{};
  std::unique_ptr<depth2_metrics_aggregator> m_metrics_agg{};
  std::unique_ptr<dynamic_cache_manager>     m_cache_manager{};
  // clang-format on
};

NS_END(depth2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH2_DEPTH2_LOOP_FUNCTIONS_HPP_ */
