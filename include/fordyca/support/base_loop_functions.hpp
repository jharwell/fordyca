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

#include "rcppsw/er/client.hpp"
#include "fordyca/params/loop_function_repository.hpp"
#include "fordyca/metrics/robot_interaction_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params {
struct output_params;
}
NS_START(support);


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
                                     public metrics::robot_interaction_metrics,
                                     public rcppsw::er::client<base_loop_functions> {
 public:
  base_loop_functions(void);
  base_loop_functions(const base_loop_functions& s) = delete;
  base_loop_functions& operator=(
      const base_loop_functions& s) = delete;

  /* CLoopFunctions overrides */
  void Init(ticpp::Element&) override;
  void PreStep(void) override;

  /* loop metrics */
  std::vector<double> nearest_neighbors(void) const override;

  void ndc_push(void) {
    ER_NDC_PUSH("[t=" + std::to_string(GetSpace().GetSimulationClock()) + "]");
  }
  void ndc_pop(void) { ER_NDC_POP(); }

 protected:
  argos::CFloorEntity* floor(void) const { return m_floor; }
  const std::string& output_root(void) const { return m_output_root; }
  const params::loop_function_repository& params(void) const { return m_params; }
  params::loop_function_repository& params(void) { return m_params; }

 private:
  /**
   * @brief Initialize logging for all support/loop function code.
   *
   * @param output Parsed output parameters.n
   */
  void output_init(const struct params::output_params* const output);

  // clang-format off
  argos::CFloorEntity*                              m_floor{nullptr};
  std::string                                       m_output_root{""};
  params::loop_function_repository                  m_params{};
  // clang-format on
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BASE_LOOP_FUNCTIONS_HPP_ */
