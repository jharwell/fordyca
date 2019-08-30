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
#include <memory>
#include <string>
#include <vector>

#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/radians.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/utils/color.hpp"

#include "fordyca/config/loop_function_repository.hpp"
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::convergence {
class convergence_calculator;
namespace config {
struct convergence_config;
} /* namespace config */
} /* namespace cosm::convergence */

NS_START(fordyca);

namespace config {
struct output_config;
namespace tv {
struct tv_manager_config;
}
namespace oracle {
struct oracle_manager_config;
} /* namespace oracle */
} // namespace config
namespace ds {
class arena_map;
}
NS_START(support);

namespace tv {
class tv_manager;
}
namespace oracle {
class oracle_manager;
} /* namespace oracle */

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class base_loop_functions
 * @ingroup fordyca support
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
                            public rer::client<base_loop_functions> {
 public:
  base_loop_functions(void) RCSW_COLD;
  ~base_loop_functions(void) override RCSW_COLD;

  base_loop_functions(const base_loop_functions& s) = delete;
  base_loop_functions& operator=(const base_loop_functions& s) = delete;

  /* CLoopFunctions overrides */
  void Init(ticpp::Element&) override RCSW_COLD;
  void Reset(void) override RCSW_COLD;
  void PreStep(void) override;
  void PostStep(void) override;

  void ndc_push(void) {
    ER_NDC_PUSH("[t=" + std::to_string(GetSpace().GetSimulationClock()) + "]");
  }
  void ndc_pop(void) { ER_NDC_POP(); }
  const tv::tv_manager* tv_manager(void) const { return m_tv_manager.get(); }
  const cconvergence::convergence_calculator* conv_calculator(void) const {
    return m_conv_calc.get();
  }

 protected:
  argos::CFloorEntity* floor(void) const { return m_floor; }
  const std::string& output_root(void) const { return m_output_root; }
  const config::loop_function_repository* config(void) const {
    return &m_config;
  }
  tv::tv_manager* tv_manager(void) { return m_tv_manager.get(); }

  config::loop_function_repository* config(void) { return &m_config; }
  const ds::arena_map* arena_map(void) const { return m_arena_map.get(); }
  ds::arena_map* arena_map(void) { return m_arena_map.get(); }
  cconvergence::convergence_calculator* conv_calculator(void) {
    return m_conv_calc.get();
  }
  const oracle::oracle_manager* oracle_manager(void) const {
    return m_oracle_manager.get();
  }
  oracle::oracle_manager* oracle_manager(void) {
    return m_oracle_manager.get();
  }

 private:
  /**
   * @brief Initialize convergence calculations.
   *
   * @param config Parsed convergence parameters.
   */
  void convergence_init(
      const cconvergence::config::convergence_config* config) RCSW_COLD;

  /**
   * @brief Initialize logging for all support/loop function code.
   *
   * @param output Parsed output parameters.
   */
  void output_init(const config::output_config* output) RCSW_COLD;

  /**
   * @brief Initialize the arena contents.
   *
   * @param repo Repository of parsed parameters.
   */
  void arena_map_init(const config::loop_function_repository* repo) RCSW_COLD;

  /**
   * @brief Initialize temporal variance handling.
   *
   * @param tvp Parsed TV parameters.
   */
  void tv_init(const config::tv::tv_manager_config* tvp) RCSW_COLD;

  /**
   * @brief Initialize oracular information injection.
   *
   * @param oraclep Parsed \ref oracle_manager parameters.
   */
  void oracle_init(const config::oracle::oracle_manager_config* oraclep) RCSW_COLD;

  std::vector<double> calc_robot_nn(uint n_threads) const;
  std::vector<rmath::radians> calc_robot_headings(uint n_threads) const;
  std::vector<rmath::vector2d> calc_robot_positions(uint n_threads) const;

  /* clang-format off */
  argos::CFloorEntity*                                  m_floor{nullptr};
  std::string                                           m_output_root{};
  config::loop_function_repository                      m_config{};
  std::unique_ptr<ds::arena_map>                        m_arena_map;
  std::unique_ptr<tv::tv_manager>                       m_tv_manager;
  std::unique_ptr<cconvergence::convergence_calculator> m_conv_calc;
  std::unique_ptr<oracle::oracle_manager>               m_oracle_manager;
  /* clang-format on */
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BASE_LOOP_FUNCTIONS_HPP_ */
