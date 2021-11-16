/**
 * \file base_loop_functions.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/config/rng_config.hpp"
#include "rcppsw/math/radians.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/utils/color.hpp"

#include "cosm/pal/argos/swarm_manager_adaptor.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/support/tv/tv_manager.hpp"
#include "fordyca/support/config/loop_function_repository.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::convergence {
class convergence_calculator;
namespace config {
struct convergence_config;
} /* namespace config */
} /* namespace cosm::convergence */

namespace cosm::pal::argos {
template <class TController>
class convergence_calculator;
} /* namespace cosm::pal */

namespace cosm::pal::config {
struct output_config;
} /* namespace cosm::metrics::config */

namespace cosm::oracle::config {
struct aggregate_oracle_config;
} /* namespace cosm::oracle::config */
namespace cosm::foraging::oracle {
class foraging_oracle;
}

NS_START(fordyca, support);

namespace config {
namespace tv {
struct tv_manager_config;
}
} // namespace config

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class base_loop_functions
 * \ingroup support
 *
 * \brief The base loop functions in FORDYCA that all other loop functions
 * inherit from.
 *
 * This class is not a functional set of loop functions, but it provides
 * functions needed across multiple derived classes, but functionality that
 * could not just be free functions because they require access to members in
 * the \ref argos::CLoopFunctions class.
 */
class base_loop_functions : public cpargos::swarm_manager_adaptor,
                            public rer::client<base_loop_functions> {
 public:
  using convergence_calculator_type =
      cpargos::convergence_calculator<cpcontroller::controller2D>;

  base_loop_functions(void) RCPPSW_COLD;
  ~base_loop_functions(void) override RCPPSW_COLD;

  /* Not copy constructible/assignable by default */
  base_loop_functions(const base_loop_functions& s) = delete;
  base_loop_functions& operator=(const base_loop_functions& s) = delete;

  /* swarm manager overrides */
  void init(ticpp::Element&) override RCPPSW_COLD;
  void reset(void) override RCPPSW_COLD;
  void pre_step(void) override;
  void post_step(void) override;

  const tv::tv_manager* tv_manager(void) const { return m_tv_manager.get(); }
  const convergence_calculator_type* conv_calculator(void) const {
    return m_conv_calc.get();
  }
  const cforacle::foraging_oracle* oracle(void) const { return m_oracle.get(); }
  const carena::caching_arena_map* arena_map(void) const RCPPSW_PURE;

 protected:
  tv::tv_manager* tv_manager(void) { return m_tv_manager.get(); }
  const config::loop_function_repository* config(void) const { return &m_config; }
  config::loop_function_repository* config(void) { return &m_config; }
  convergence_calculator_type* conv_calculator(void) { return m_conv_calc.get(); }
  cforacle::foraging_oracle* oracle(void) { return m_oracle.get(); }
  carena::caching_arena_map* arena_map(void) RCPPSW_PURE;
  void config_parse(ticpp::Element& node) RCPPSW_COLD;

  /*
   * If we are doing a powerlaw distribution we may need to create caches BEFORE
   * clusters, so that cluster mapping will avoid the placed caches, and we
   * don't know where the clusters are going to map to ahead of time. This hook
   * allows derived classes to do that if needed.
   */
  void delay_arena_map_init(bool b) { m_delay_arena_map_init = b; }
  bool delay_arena_map_init(void) const { return m_delay_arena_map_init; }

 private:
  /**
   * \brief Initialize convergence calculations.
   *
   * \param config Parsed convergence parameters.
   */
  void
  convergence_init(const cconvconfig::convergence_config* config) RCPPSW_COLD;

  /**
   * \brief Initialize temporal variance handling.
   *
   * \param tvp Parsed TV parameters.
   */
  void tv_init(const config::tv::tv_manager_config* tvp) RCPPSW_COLD;

  /**
   * \brief Initialize logging for all support/loop function code.
   *
   * \param output Parsed output parameters.
   */
  void output_init(const cpconfig::output_config* output) override RCPPSW_COLD;

  /**
   * \brief Initialize oracular information injection.
   *
   * \param oraclep Parsed \ref aggregate_oracle parameters.
   */
  void oracle_init(const coconfig::aggregate_oracle_config* oraclep) RCPPSW_COLD;

  /* clang-format off */
  bool                                         m_delay_arena_map_init{false};
  config::loop_function_repository             m_config{};
  std::unique_ptr<tv::tv_manager>              m_tv_manager;
  std::unique_ptr<convergence_calculator_type> m_conv_calc;
  std::unique_ptr<cforacle::foraging_oracle>   m_oracle;
  /* clang-format on */
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BASE_LOOP_FUNCTIONS_HPP_ */
