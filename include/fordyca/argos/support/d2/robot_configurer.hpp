/**
 * \file d2/robot_configurer.hpp
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
class robot_configurer : public d1::robot_configurer<TController,
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

