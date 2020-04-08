/**
 * \file base_loop_functions.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/base_loop_functions.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/arena/config/arena_map_config.hpp"
#include "cosm/pal/argos_convergence_calculator.hpp"
#include "cosm/metrics/config/output_config.hpp"
#include "cosm/oracle/config/aggregate_oracle_config.hpp"
#include "cosm/foraging/oracle/foraging_oracle.hpp"
#include "cosm/oracle/tasking_oracle.hpp"
#include "cosm/pal/argos_swarm_iterator.hpp"
#include "cosm/vis/config/visualization_config.hpp"

#include "fordyca/config/tv/tv_manager_config.hpp"
#include "fordyca/controller/foraging_controller.hpp"
#include "fordyca/support/tv/env_dynamics.hpp"
#include "fordyca/support/tv/fordyca_pd_adaptor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_loop_functions::base_loop_functions(void)
    : ER_CLIENT_INIT("fordyca.loop.base"),
      m_tv_manager(nullptr),
      m_conv_calc(nullptr),
      m_oracle(nullptr) {}

base_loop_functions::~base_loop_functions(void) = default;

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/
void base_loop_functions::init(ticpp::Element& node) {
  /* parse simulation input file */
  m_config.parse_all(node);

  if (!m_config.validate_all()) {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }

  /* initialize RNG */
  rng_init(config()->config_get<rmath::config::rng_config>());

  /* initialize output and metrics collection */
  output_init(m_config.config_get<cmconfig::output_config>());

  /* initialize arena map and distribute blocks */
  auto* aconfig = config()->config_get<caconfig::arena_map_config>();
  auto* vconfig = config()->config_get<cvconfig::visualization_config>();
  arena_map_init<carena::caching_arena_map>(aconfig, vconfig);

  /* initialize convergence calculations */
  convergence_init(
      config()->config_get<cconvergence::config::convergence_config>());

  /* initialize temporal variance injection */
  tv_init(config()->config_get<config::tv::tv_manager_config>());

  /* initialize oracle, if configured */
  oracle_init(config()->config_get<coconfig::aggregate_oracle_config>());
} /* init() */

void base_loop_functions::convergence_init(
    const ccconfig::convergence_config* const config) {
  if (nullptr == config) {
    return;
  }
  ER_INFO("Creating convergence calculator");
  m_conv_calc = std::make_unique<convergence_calculator_type>(config, this);
} /* convergence_init() */

void base_loop_functions::tv_init(const config::tv::tv_manager_config* tvp) {
  ER_INFO("Creating temporal variance manager");

  /*
   * We unconditionally create environmental dynamics because they are used to
   * generate the 1 timestep penalties for robot-arena interactions even when
   * they are disabled, and trying to figure out how to get things to work if
   * they are omitted is waaayyyy too much work. See #621 too.
   */
  auto envd = std::make_unique<tv::env_dynamics>(
      &tvp->env_dynamics, this, arena_map());

  auto popd = std::make_unique<tv::fordyca_pd_adaptor>(
      &tvp->population_dynamics,
      this,
      envd.get(),
      arena_map(),
      rng());

  m_tv_manager =
      std::make_unique<tv::tv_manager>(std::move(envd), std::move(popd));

  /*
   * Register all controllers with temporal variance manager in order to be able
   * to apply environmental variances if configured. Note that we MUST use
   * static ordering, because we use robot ID to create the mapping.
   */
  auto cb = [&](auto* c) {
    m_tv_manager->dynamics<ctv::dynamics_type::ekENVIRONMENT>()
        ->register_controller(*c);
    c->irv_init(m_tv_manager->dynamics<ctv::dynamics_type::ekENVIRONMENT>()
                    ->rda_adaptor());
  };
  cpal::argos_swarm_iterator::controllers<argos::CFootBotEntity,
                                          controller::foraging_controller,
                                          cpal::iteration_order::ekSTATIC>(
      this, cb, kARGoSRobotType);
} /* tv_init() */

void base_loop_functions::output_init(const cmconfig::output_config* output) {
  argos_sm_adaptor::output_init(output->output_root, output->output_dir);

#if (LIBRA_ER == LIBRA_ER_ALL)
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("fordyca.events"),
                 output_root() + "/events.log");
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("fordyca.support"),
                 output_root() + "/support.log");
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("fordyca.loop"),
                 output_root() + "/sim.log");
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("cosm.foraging.ds.arena_map"),
                 output_root() + "/sim.log");
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("fordyca.metrics"),
                 output_root() + "/metrics.log");
#endif
} /* output_init() */

void base_loop_functions::oracle_init(
    const coconfig::aggregate_oracle_config* const oraclep) {
  if (nullptr != oraclep) {
    ER_INFO("Creating foraging oracle");
    m_oracle = std::make_unique<cforacle::foraging_oracle>(oraclep);
  }
} /* oracle_init() */

/*******************************************************************************
 * ARGoS Hooks
 ******************************************************************************/
void base_loop_functions::pre_step(void) {
  /*
   * Needs to be before robot controllers are run, so they run with the correct
   * throttling/are subjected to the correct penalties, etc.
   */
  if (nullptr != m_tv_manager) {
    m_tv_manager->update(rtypes::timestep(GetSpace().GetSimulationClock()));
  }

  if (nullptr != oracle()) {
    oracle()->update(arena_map());
  }
} /* pre_step() */

void base_loop_functions::post_step(void) {
  /*
   * Needs to be after robot controllers are run, because compute convergence
   * before that gives you the convergence status for the LAST timestep.
   */
  if (nullptr != m_conv_calc) {
    m_conv_calc->update();
  }
} /* post_step() */

void base_loop_functions::reset(void) {
  arena_map()->distribute_all_blocks();
} /* reset() */


NS_END(support, fordyca);
