/**
 * \file robot_task_extractor_adaptor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_ROBOT_TASK_EXTRACTOR_ADAPTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_ROBOT_TASK_EXTRACTOR_ADAPTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant/static_visitor.hpp>

#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/support/robot_task_extractor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct robot_task_extractor_adaptor
 * \ingroup fordyca support
 *
 * \brief Map a controller instance to the task extraction action run during
 * convergence calculations.
 */
struct robot_task_extractor_adaptor : public boost::static_visitor<int> {
  explicit robot_task_extractor_adaptor(const controller::base_controller* const c)
      : mc_controller(c) {}

  template <typename ControllerType>
  int operator()(const robot_task_extractor<ControllerType>& extractor) const {
    return extractor(dynamic_cast<const ControllerType*>(mc_controller));
  }
  const controller::base_controller* const mc_controller;
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORYDCA_SUPPORT_ROBOT_TASK_EXTRACTOR_ADAPTOR_HPP_ */
