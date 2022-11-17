/**
 * \file d2/robot_configurer.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/argos/support/d1/robot_configurer.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, argos, support, d2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct robot_configurer
 * \ingroup support argos d2
 *
 * \brief Configure a d2 controller during initialization:
 *
 * - Displaying task text
 * - Enabled oracles (if applicable)
 * - Enabling tasking metric aggregation via task executive hooks
 */
template<class TController, class TAggregator>
class robot_configurer final : public d1::robot_configurer<TController,
                                                           TAggregator> {
 public:
  using controller_type = typename d1::robot_configurer<TController,
                                                            TAggregator>::controller_type;
  using d1::robot_configurer<TController,
                                 TAggregator>::robot_configurer;
  using d1::robot_configurer<TController,
                                 TAggregator>::controller_config_vis;
  using d1::robot_configurer<TController,
                                 TAggregator>::controller_config_oracle;
  using d1::robot_configurer<TController,
                                 TAggregator>::metric_callbacks_bind;

  template<typename U = TController,
           RCPPSW_SFINAE_TYPELIST_REJECT(controller::d2::oracular_typelist,
                                         U)>
  void operator()(controller_type* const c) const {
    controller_config_vis(c);
    metric_callbacks_bind(c);
  } /* operator() */

  template<typename U = TController,
           RCPPSW_SFINAE_TYPELIST_REQUIRE(controller::d2::oracular_typelist,
                                          U)>
  void operator()(controller_type* const c) const {
    metric_callbacks_bind(c);
    controller_config_vis(c);
    controller_config_oracle(c);
  } /* operator() */
};
NS_END(d2, support, argos, fordyca);
