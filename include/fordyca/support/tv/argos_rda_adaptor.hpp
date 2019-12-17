/**
 * \file argos_rda_adaptor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_TV_ARGOS_RDA_ADAPTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_TV_ARGOS_RDA_ADAPTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"

#include "cosm/tv/switchable_tv_generator.hpp"
#include "cosm/tv/robot_dynamics_applicator.hpp"
#include "cosm/cosm.hpp"
#include "cosm/pal/swarm_manager.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class argos_rda_adaptor
 * \ingroup fordyca support tv
 *
 * \brief Adapts \ref ctv::robot_dynamics_applicator to work within the ARGoS
 * simulator.
 */
class argos_rda_adaptor final : public rer::client<argos_rda_adaptor>,
                                public ctv::robot_dynamics_applicator {
 public:
  argos_rda_adaptor(const ctv::config::robot_dynamics_applicator_config* config,
                 const cpal::swarm_manager* sm);

  argos_rda_adaptor(const argos_rda_adaptor&) = delete;
  const argos_rda_adaptor& operator=(const argos_rda_adaptor&) = delete;

  void update(void) override;
  double avg_motion_throttle(void) const override;

 private:
  /* clang-format off */
  const cpal::swarm_manager* const mc_sm;
  /* clang-format on */
};

NS_END(tv, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_TV_ARGOS_RDA_ADAPTOR_HPP_ */
