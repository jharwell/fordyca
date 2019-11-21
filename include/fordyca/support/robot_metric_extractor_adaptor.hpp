/**
 * \file robot_metric_extractor_adaptor.hpp
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
#ifndef INCLUDE_FORDYCA_SUPPORT_ROBOT_METRIC_EXTRACTOR_ADAPTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_ROBOT_METRIC_EXTRACTOR_ADAPTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/ds/type_map.hpp"

#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/fordyca.hpp"
#include "fordyca/support/robot_metric_extractor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * \struct robot_metric_extractor_adaptor
 * \ingroup fordyca support depth0
 *
 * \brief Wrapping functor to perform metric extractor which provides the
 * indirection in mapping from templated controller type in operator() to the
 * actual extractor_adaptor implementation (needed for use with
 * boost::static_visitor).
 */
template <class AggregatorType>
struct robot_metric_extractor_adaptor {
  explicit robot_metric_extractor_adaptor(
      const controller::base_controller* const c)
      : controller(c) {}
  template <class ControllerType>
  void operator()(const robot_metric_extractor<AggregatorType, ControllerType>&
                      extractor) const {
    auto cast = dynamic_cast<
        const typename robot_metric_extractor<AggregatorType,
                                              ControllerType>::controller_type*>(
        controller);
    extractor(cast);
  }
  const controller::base_controller* const controller;
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_ROBOT_METRIC_EXTRACTOR_ADAPTOR_HPP_ */
