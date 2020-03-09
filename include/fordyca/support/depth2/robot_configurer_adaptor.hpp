/**
 * \file depth2/robot_configurer_adaptor.hpp
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
#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH2_ROBOT_CONFIGURER_ADAPTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH2_ROBOT_CONFIGURER_ADAPTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fordyca.hpp"
#include "fordyca/controller/controller_fwd.hpp"
#include "rcppsw/ds/type_map.hpp"
#include "fordyca/support/depth1/robot_configurer_adaptor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, depth2);
class depth2_metrics_aggregator;
template<class ControllerType, class AggregatorType>
class robot_configurer;
NS_START(detail);

using configurer_map_type = rds::type_map<
   rmpl::typelist_wrap_apply<controller::depth2::typelist,
                             robot_configurer,
                             depth2_metrics_aggregator>::type>;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class robot_configurer_adaptor
 * \ingroup support depth2
 *
 * \brief Wrapping functor to perform robot controller configuration during
 * initialization.
 */
class robot_configurer_adaptor {
 public:
  explicit robot_configurer_adaptor(controller::foraging_controller* const c)
      : controller(c) {}

  template<typename TController, typename TAggregator>
  void operator()(robot_configurer<TController, TAggregator>& configurer) const {
    auto cast = dynamic_cast<
      typename robot_configurer<TController, TAggregator>::controller_type*
      >(controller);
    configurer(cast);
  }

 private:
  /* clang-format off */
  controller::foraging_controller* const controller;
  /* clang-format on */
};

NS_END(detail, depth2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH2_ROBOT_CONFIGURER_ADAPTOR_HPP_ */
