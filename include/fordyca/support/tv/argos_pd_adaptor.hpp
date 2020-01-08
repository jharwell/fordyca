/**
 * \file argos_pd_adaptor.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_TV_ARGOS_PD_ADAPTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_TV_ARGOS_PD_ADAPTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"

#include "cosm/tv/switchable_tv_generator.hpp"
#include "cosm/tv/population_dynamics.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca);
namespace ds {
class arena_map;
} /* namespace repr */

NS_START(support);

class base_loop_functions;

NS_START(tv);

class env_dynamics;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class argos_pd_adaptor
 * \ingroup support tv
 *
 * \brief Adapts \ref ctv::population_dynamics to work within the ARGoS
 * simulator.
 */
class argos_pd_adaptor final : public rer::client<argos_pd_adaptor>,
                               public ctv::population_dynamics {
 public:
  /**
   * @brief When adding/removing a robot, try this many times to complete the
   * operation.
   */
  static constexpr size_t kMaxOperationAttempts = 1000;

  argos_pd_adaptor(const ctv::config::population_dynamics_config* config,
                support::base_loop_functions* lf,
                ds::arena_map * map,
                env_dynamics *envd,
                const std::string& entity_prefix,
                const std::string& controller_xml_id,
                rmath::rng* rng);

  argos_pd_adaptor(const argos_pd_adaptor&) = delete;
  const argos_pd_adaptor& operator=(const argos_pd_adaptor&) = delete;

  op_result robot_kill(void) override;
  op_result robot_add(size_t max_pop,
                      const rtypes::type_uuid& id) override;
  op_result robot_malfunction(void) override;
  op_result robot_repair(const rtypes::type_uuid& id) override;

 private:
  /* clang-format off */
  const std::string                   mc_entity_prefix;
  const std::string                   mc_controller_xml_id;
  const support::base_loop_functions* mc_lf;

  ds::arena_map *                     m_map;
  env_dynamics*                       m_envd;
  rmath::rng*                         m_rng;
  support::base_loop_functions*       m_lf;
  /* clang-format on */
};

NS_END(tv, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_TV_ARGOS_PD_ADAPTOR_HPP_ */
