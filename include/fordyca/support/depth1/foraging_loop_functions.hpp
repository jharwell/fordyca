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
class foraging_loop_functions : public depth0::stateful_foraging_loop_functions {
 public:
  foraging_loop_functions(void) : m_metrics_agg(nullptr) {}
  ~foraging_loop_functions(void) override = default;

  void Init(ticpp::Element& node) override;
  void PreStep() override;
  void Reset(void) override;

  /**
   * @brief Set the LOS of a robot in the arena, INCLUDING handling caches whose
   * extent overlaps the LOS but whose host cell is not in the LOS (see #244).
   */
  template<typename T>
  void set_robot_los(argos::CFootBotEntity& robot,
                     representation::arena_map& map) {
    argos::CVector2 pos;
    pos.Set(const_cast<argos::CFootBotEntity&>(robot).GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
            const_cast<argos::CFootBotEntity&>(robot).GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

    rcppsw::math::dcoord2 robot_loc =
        math::rcoord_to_dcoord(pos, map.grid_resolution());
    auto& controller = dynamic_cast<T&>(robot.GetControllableEntity().GetController());
    std::unique_ptr<representation::line_of_sight> new_los =
        rcppsw::make_unique<representation::line_of_sight>(
            map.subgrid(robot_loc.first, robot_loc.second, 2),
            robot_loc);

    for (auto &c : map.caches()) {
      argos::CVector2 ll = math::dcoord_to_rcoord(new_los->abs_ll(),
                                                  map.grid_resolution());
      argos::CVector2 lr = math::dcoord_to_rcoord(new_los->abs_lr(),
                                                  map.grid_resolution());
      argos::CVector2 ul = math::dcoord_to_rcoord(new_los->abs_ul(),
                                                  map.grid_resolution());
      argos::CVector2 ur = math::dcoord_to_rcoord(new_los->abs_ur(),
                                                  map.grid_resolution());
      if (c->contains_point(ll) || c->contains_point(lr) ||
          c->contains_point(ul) || c->contains_point(ur)) {
        ER_VER("Add partially overlapping cache to %s LOS",
                controller.GetId().c_str());
        new_los->cache_add(c);
      }
    } /* for(&c..) */

    controller.los(new_los);
  }

 private:
  using interactor = arena_interactor<controller::depth1::foraging_controller>;

  void pre_step_final(void) override;
  void pre_step_iter(argos::CFootBotEntity& robot);
  argos::CColor GetFloorColor(const argos::CVector2& plane_pos) override;
  void cache_handling_init(const struct params::arena_map_params *arenap);

  // clang-format off
  double                              mc_cache_respawn_scale_factor{0.0};
  std::unique_ptr<interactor>         m_interactor{nullptr};
  metrics::caches::lifecycle_collator m_cache_collator{};
  std::unique_ptr<metrics_aggregator> m_metrics_agg;
  // clang-format on
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_FORAGING_LOOP_FUNCTIONS_HPP_ */
