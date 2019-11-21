/**
 * \file irv_lf_adaptor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_TV_IRV_LF_ADAPTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_TV_IRV_LF_ADAPTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"

#include "cosm/tv/switchable_tv_generator.hpp"
#include "cosm/tv/swarm_irv_manager.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support);

class base_loop_functions;

NS_START(tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class irv_lf_adaptor
 * \ingroup fordyca support tv
 *
 * \brief Adapts \ref ctv::swarm_irv_manager to work within the ARGoS simulator.
 */
class irv_lf_adaptor final : public rer::client<irv_lf_adaptor>,
                             public ctv::swarm_irv_manager {
 public:
  irv_lf_adaptor(const ctv::config::swarm_irv_manager_config* config,
                const support::base_loop_functions* lf);

  irv_lf_adaptor(const irv_lf_adaptor& other) = delete;
  const irv_lf_adaptor& operator=(const irv_lf_adaptor& other) = delete;

  void update(void) override;
  double avg_motion_throttle(void) const override;

 private:
  /* clang-format off */
  const support::base_loop_functions* const mc_lf;
  /* clang-format on */
};

NS_END(tv, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_TV_IRV_LF_ADAPTOR_HPP_ */
