/**
 * @file robot_configurer.hpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_ROBOT_CONFIGURER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH1_ROBOT_CONFIGURER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/support/oracle/entities_oracle.hpp"
#include "fordyca/support/oracle/tasking_oracle.hpp"
#include "fordyca/config/visualization_config.hpp"
#include "fordyca/support/depth1/depth1_metrics_aggregator.hpp"
#include "fordyca/controller/oracular_info_receptor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @struct robot_configurer
 * @ingroup fordyca support depth1
 *
 * @brief Configure a depth1 controller during initialization:
 *
 * - Displaying task text
 * - Enabled oracles (if applicable) via task executive hooks
 * - Enabling tasking metric aggregation via task executive hooks
 */
template <class ControllerType, class AggregatorType>
class robot_configurer {
 public:
  using controller_type = ControllerType;

  robot_configurer(const config::visualization_config* const config,
                   oracle::entities_oracle* const ent_oracle,
                   oracle::tasking_oracle* const task_oracle,
                   AggregatorType* const agg)
      : mc_config(config),
        m_tasking_oracle(task_oracle),
        m_ent_oracle(ent_oracle),
        m_agg(agg) {}

  template<typename U = ControllerType,
           RCPPSW_SFINAE_TYPELIST_REJECT(controller::depth1::oracular_typelist,
                                         U)>
  void operator()(controller_type* const c) const {
    controller_config_vis(c);
    metric_callbacks_bind(c);
  } /* operator() */

  template<typename U = ControllerType,
           RCPPSW_SFINAE_TYPELIST_REQUIRE(controller::depth1::oracular_typelist,
                                          U)>
  void operator()(controller_type* const c) const {
    metric_callbacks_bind(c);
    controller_config_vis(c);
    controller_config_oracle(c);
  } /* operator() */

 private:
  void metric_callbacks_bind(controller_type* const c) const {
    c->executive()->task_finish_notify(
        std::bind(&AggregatorType::task_finish_or_abort_cb,
                  m_agg,
                  std::placeholders::_1));
    c->executive()->task_abort_notify(
        std::bind(&AggregatorType::task_finish_or_abort_cb,
                  m_agg,
                  std::placeholders::_1));
    c->executive()->task_alloc_notify(
        std::bind(&AggregatorType::task_alloc_cb,
                  m_agg,
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
    if (nullptr != m_tasking_oracle) {
      m_tasking_oracle->listener_add(c->executive());
      c->executive()->task_finish_notify(
          std::bind(&oracle::tasking_oracle::task_finish_cb,
                    m_tasking_oracle,
                    std::placeholders::_1));
      c->executive()->task_abort_notify(
          std::bind(&oracle::tasking_oracle::task_abort_cb,
                    m_tasking_oracle,
                    std::placeholders::_1));
    }
    if (nullptr != m_tasking_oracle || nullptr != m_ent_oracle) {
      auto receptor = rcppsw::make_unique<controller::oracular_info_receptor>(
          m_tasking_oracle, m_ent_oracle);
      c->oracle_init(std::move(receptor));
    }
  } /* controller_config_oracle() */

  /* clang-format off */
  const config::visualization_config* const mc_config;
  oracle::tasking_oracle* const             m_tasking_oracle;
  oracle::entities_oracle* const            m_ent_oracle;
  AggregatorType* const                     m_agg;
  /* clang-format on */
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_ROBOT_CONFIGURER_HPP_ */
