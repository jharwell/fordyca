/**
 * @file task_repository.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_PARAMS_DEPTH2_TASK_REPOSITORY_HPP_
#define INCLUDE_FORDYCA_PARAMS_DEPTH2_TASK_REPOSITORY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/params/xml_param_repository.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class task_repository
 * @ingroup params depth2
 *
 * @brief Collection of all parameter parsers and parse results needed
 * by the \ref depth2::foraging_controller.
 */
class task_repository: public rcppsw::params::xml_param_repository {
 public:
  static constexpr char kName[] = "depth2_task_repository";
  task_repository(const std::shared_ptr<rcppsw::er::server>& server);
};

NS_END(depth2, params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_DEPTH2_TASK_REPOSITORY_HPP_ */
