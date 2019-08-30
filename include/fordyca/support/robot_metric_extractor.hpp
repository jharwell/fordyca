/**
 * @file robot_metric_extractor.hpp
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
#ifndef INCLUDE_FORDYCA_SUPPORT_ROBOT_METRIC_EXTRACTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_ROBOT_METRIC_EXTRACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant/static_visitor.hpp>

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @struct robot_metric_extractor
 * @ingroup fordyca support
 *
 * @brief Functor to perform metric extraction from a controller on each
 * timestep.
 */
template <class AggregatorType, class ControllerType>
class robot_metric_extractor : public boost::static_visitor<void> {
 public:
  using controller_type = ControllerType;
  explicit robot_metric_extractor(AggregatorType* const agg) : m_agg(agg) {}

  void operator()(const ControllerType* const c) const {
    m_agg->collect_from_controller(c);
  }

 private:
  /* clang-format off */
  AggregatorType* const m_agg;
  /* clang-format on */
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_ROBOT_METRIC_EXTRACTOR_HPP_ */
