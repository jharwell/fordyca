/**
 * \file d1/robot_configurer.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_D1_ROBOT_CONFIGURER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_D1_ROBOT_CONFIGURER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>

#include "fordyca/controller/controller_fwd.hpp"
#include "cosm/oracle/tasking_oracle.hpp"
#include "cosm/vis/config/visualization_config.hpp"
#include "cosm/foraging/oracle/foraging_oracle.hpp"

#include "fordyca/support/d1/d1_metrics_manager.hpp"
#include "fordyca/subsystem/perception/oracular_info_receptor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, d1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct robot_configurer
 * \ingroup support d1
 *
 * \brief Configure a d1 controller during initialization:
 *
 * - Displaying task text
 * - Enabled oracles (if applicable)
 * - Enabling tasking metric aggregation via task executive hooks
 */
template <class TController, class TMetricsManager>
class robot_configurer {
 public:
  using controller_type = TController;

  robot_configurer(const cvconfig::visualization_config* const config,
                   cforacle::foraging_oracle* const oracle,
                   TMetricsManager* const metrics_manager)
      : mc_config(config),
        m_oracle(oracle),
        m_metrics_manager(metrics_manager) {}

  template<typename U = TController,
           RCPPSW_SFINAE_TYPELIST_REJECT(controller::d1::oracular_typelist,
                                         U)>
  void operator()(controller_type* const c) const {
    controller_config_vis(c);
    metric_callbacks_bind(c);
  } /* operator() */

  template<typename U = TController,
           RCPPSW_SFINAE_TYPELIST_REQUIRE(controller::d1::oracular_typelist,
                                          U)>
  void operator()(controller_type* const c) const {
    metric_callbacks_bind(c);
    controller_config_vis(c);
    controller_config_oracle(c);
  } /* operator() */

 protected:
  void metric_callbacks_bind(controller_type* const c) const {
    c->executive()->task_finish_notify(
        std::bind(&TMetricsManager::task_finish_or_abort_cb,
                  m_metrics_manager,
                  std::placeholders::_1));
    c->executive()->task_abort_notify(
        std::bind(&TMetricsManager::task_finish_or_abort_cb,
                  m_metrics_manager,
                  std::placeholders::_1));
    c->executive()->task_start_notify(
        std::bind(&TMetricsManager::task_start_cb,
                  m_metrics_manager,
                  std::placeholders::_1,
                  std::placeholders::_2));
  } /* metric_callbacks_bind() */

  void controller_config_vis(controller_type* const c) const {
    /*
     * If NULL, then visualization has been disabled.
     */
    if (nullptr != mc_config) {
      c->display_los(mc_config->robot_los);
      c->display_id(mc_config->robot_id);
      c->display_task(mc_config->robot_task);
    }
  } /* controller_config_vis() */

  void controller_config_oracle(controller_type *const c) const {
    if (nullptr != m_oracle->tasking()) {
      m_oracle->tasking()->listener_add(c->executive());
    }
    if (nullptr != m_oracle) {
      auto receptor = std::make_unique<fsperception::oracular_info_receptor>(m_oracle);
      c->oracle_init(std::move(receptor));
    }
  } /* controller_config_oracle() */

 private:
  /* clang-format off */
  const cvconfig::visualization_config* const mc_config;
  cforacle::foraging_oracle* const            m_oracle;

  TMetricsManager* const                      m_metrics_manager;
  /* clang-format on */
};

NS_END(d1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_D1_ROBOT_CONFIGURER_HPP_ */
