/**
 * @file base_loop_functions.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_BASE_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_BASE_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/simulator/loop_functions.h>
#include <string>
#include <vector>

#include "fordyca/metrics/temporal_variance_metrics.hpp"
#include "fordyca/params/loop_function_repository.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/metrics/swarm/convergence_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params {
struct output_params;
}
namespace ds {
class arena_map;
}

NS_START(support);

namespace rmetrics = rcppsw::metrics;
namespace rmath = rcppsw::math;

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class base_loop_functions
 * @ingroup support
 *
 * @brief The base loop functions in FORDYCA that all other loop functions
 * inherit from.
 *
 * This class is not a functional set of loop functions, but it provides
 * functions needed across multiple derived classes, but functionality that
 * could not just be free functions because they require access to members in
 * the \ref argos::CLoopFunctions class.
 */
class base_loop_functions : public argos::CLoopFunctions,
                            public rmetrics::swarm::convergence_metrics,
                            public metrics::temporal_variance_metrics,
                            public rcppsw::er::client<base_loop_functions> {
 public:
  base_loop_functions(void);
  base_loop_functions(const base_loop_functions& s) = delete;
  base_loop_functions& operator=(const base_loop_functions& s) = delete;

  /* CLoopFunctions overrides */
  void Init(ticpp::Element&) override;
  void Reset(void) override;

  /* convergence metrics */
  std::vector<double> robot_nearest_neighbors(void) const override;
  std::vector<rmath::radians> robot_headings(void) const override;
  std::vector<rmath::vector2d> robot_positions(void) const override;

  /* temporal variance metrics */
  double swarm_motion_throttle(void) const override;
  double env_block_manipulation(void) const override { return 0.0; }
  double env_cache_usage(void) const override { return 0.0; }

  void ndc_push(void) {
    ER_NDC_PUSH("[t=" + std::to_string(GetSpace().GetSimulationClock()) + "]");
  }
  void ndc_pop(void) { ER_NDC_POP(); }

 protected:
  argos::CFloorEntity* floor(void) const { return m_floor; }
  const std::string& output_root(void) const { return m_output_root; }
  const params::loop_function_repository* params(void) const {
    return &m_params;
  }
  params::loop_function_repository* params(void) { return &m_params; }
  const ds::arena_map* arena_map(void) const { return m_arena_map.get(); }
  ds::arena_map* arena_map(void) { return m_arena_map.get(); }

 private:
  /**
   * @brief Initialize logging for all support/loop function code.
   *
   * @param output Parsed output parameters.
   */
  void output_init(const struct params::output_params* const output);
  void arena_map_init(const params::loop_function_repository* repo);

  /* clang-format off */
  uint                             m_loop_threads{0};
  argos::CFloorEntity*             m_floor{nullptr};
  std::string                      m_output_root{""};
  params::loop_function_repository m_params{};
  std::unique_ptr<ds::arena_map>   m_arena_map;
  /* clang-format on */
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BASE_LOOP_FUNCTIONS_HPP_ */
