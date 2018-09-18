/**
 * @file foraging_loop_functions.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_FORAGING_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH1_FORAGING_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include "fordyca/support/depth0/stateful_foraging_loop_functions.hpp"
#include "fordyca/tasks/depth1/foraging_task.hpp"
#include "fordyca/support/depth1/arena_interactor.hpp"
#include "fordyca/metrics/caches/lifecycle_collator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);

class metrics_aggregator;

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class foraging_loop_functions
 * @ingroup support depth1
 *
 * @brief The loop functions for depth 1 foraging.
 *
 * Handles all operations robots perform relating to static caches: pickup,
 * drop, etc.
 */
class foraging_loop_functions : public depth0::stateful_foraging_loop_functions,
                                public er::client<foraging_loop_functions> {
 public:
  foraging_loop_functions(void)
      : ER_CLIENT_INIT("fordyca.loop.depth1"),
        m_metrics_agg(nullptr) {}
  ~foraging_loop_functions(void) override = default;

  void Init(ticpp::Element& node) override;
  void PreStep() override;
  void Reset(void) override;

 private:
  using interactor = arena_interactor<controller::depth1::foraging_controller>;

  void pre_step_final(void) override;
  void pre_step_iter(argos::CFootBotEntity& robot);
  argos::CColor GetFloorColor(const argos::CVector2& plane_pos) override;
  void cache_handling_init(const struct params::arena::arena_map_params *arenap);

  // clang-format off
  double                              mc_cache_respawn_scale_factor{0.0};
  std::unique_ptr<interactor>         m_interactor{nullptr};
  metrics::caches::lifecycle_collator m_cache_collator{};
  std::unique_ptr<metrics_aggregator> m_metrics_agg;
  // clang-format on
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_FORAGING_LOOP_FUNCTIONS_HPP_ */
