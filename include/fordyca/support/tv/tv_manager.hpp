/**
 * \file tv_manager.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_TV_TV_MANAGER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_TV_TV_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>
#include <memory>

#include "fordyca/support/tv/env_dynamics.hpp"
#include "fordyca/support/tv/argos_pd_adaptor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class tv_manager
 * \ingroup support tv
 *
 * \brief Orchestrates all application of temporal variance to robot interations
 * with the environment/robotic mechanical functioning.
 */

class tv_manager {
 public:
  tv_manager(std::unique_ptr<env_dynamics> envd,
             std::unique_ptr<argos_pd_adaptor> popd) :
      m_envd(std::move(envd)),
      m_popd(std::move(popd)) {}

  tv_manager(const tv_manager&) = delete;
  const tv_manager& operator=(const tv_manager&) = delete;

  const env_dynamics* environ_dynamics(void) const { return m_envd.get(); }
  env_dynamics* environ_dynamics(void) { return m_envd.get(); }
  const argos_pd_adaptor* population_dynamics(void) const { return m_popd.get(); }

  /**
   * \brief Update the state of all applied variances. Should be called once per
   * timestep.
   */
  void update(const rtypes::timestep& t) {
    if (m_envd) {
      m_envd->update(t);
    }
    if (m_popd) {
      m_popd->update(t);
    }
  }

 private:
  /* clang-format off */
  std::unique_ptr<env_dynamics>     m_envd;
  std::unique_ptr<argos_pd_adaptor> m_popd;
  /* clang-format on */
};

NS_END(tv, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_TV_TV_MANAGER_HPP_ */
