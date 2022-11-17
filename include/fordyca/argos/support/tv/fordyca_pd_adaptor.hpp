/**
 * \file fordyca_pd_adaptor.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/argos/tv/pd_adaptor.hpp"
#include "cosm/pal/controller/controller2D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace fordyca::controller {
class foraging_controller;
} /* namespace fordyca::controller */
namespace cosm::arena {
class caching_arena_map;
} /* namespace cosm::arena */

NS_START(fordyca, argos, support, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class fordyca_pd_adaptor
 * \ingroup argos support tv
 *
 * \brief Further adapt \ref cptv::argos_pd_adaptor to the FORDYCA project,
 * providing additional callbacks to maintain simulation consistency/fidelity
 * during population dynamics application.
 */
class fordyca_pd_adaptor final : rer::client<fordyca_pd_adaptor>,
                                 public cargos::tv::pd_adaptor<cpcontroller::controller2D>{
 public:
  fordyca_pd_adaptor(const ctv::config::population_dynamics_config* config,
                     cpargos::swarm_manager_adaptor* sm,
                     env_dynamics_type *envd,
                     carena::caching_arena_map* map,
                     rmath::rng* rng);

  /* Not copy constructable/assignable by default */
  fordyca_pd_adaptor(const fordyca_pd_adaptor&) = delete;
  const fordyca_pd_adaptor& operator=(const fordyca_pd_adaptor&) = delete;

  /* ARGoS PD apdaptor overrides */
  void pre_kill_cleanup(cpcontroller::controller2D* controller) override;

 private:
  /* clang-format off */
  carena::caching_arena_map* m_map;
  /* clang-format on */
};

NS_END(tv, support, argos, fordyca);

