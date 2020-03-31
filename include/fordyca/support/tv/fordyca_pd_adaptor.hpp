/**
 * \file fordyca_pd_adaptor.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_TV_FORDYCA_PD_ADAPTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_TV_FORDYCA_PD_ADAPTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/pal/tv/argos_pd_adaptor.hpp"
#include "cosm/pal/argos_controller2D_adaptor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace fordyca::controller {
class foraging_controller;
} /* namespace fordyca::controller */

NS_START(fordyca, support, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class fordyca_pd_adaptor
 * \ingroup support tv
 *
 * \brief Further adapt \ref cptv::argos_pd_adaptor to the FORDYCA project,
 * providing additional callbacks to maintain simulation consistency/fidelity
 * during population dynamics application.
 */
class fordyca_pd_adaptor final : rer::client<fordyca_pd_adaptor>,
                           public cptv::argos_pd_adaptor<cpal::argos_controller2D_adaptor>{
 public:
  template <typename... Args>
  fordyca_pd_adaptor(Args&&... args)
      : ER_CLIENT_INIT("fordyca.support.tv.fordyca_pd_adaptor"),
        argos_pd_adaptor<cpal::argos_controller2D_adaptor>(std::forward<Args>(args)...) {}

  /* Not copy constructable/assignable by default */
  fordyca_pd_adaptor(const fordyca_pd_adaptor&) = delete;
  const fordyca_pd_adaptor& operator=(const fordyca_pd_adaptor&) = delete;

  /* ARGoS PD apdaptor overrides */
  void pre_kill_cleanup(cpal::argos_controller2D_adaptor* controller) override;
};

NS_END(tv, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_TV_FORDYCA_PD_ADAPTOR_HPP_ */
