/**
 * @file task_repository.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_PARAMS_DEPTH1_TASK_REPOSITORY_HPP_
#define INCLUDE_FORDYCA_PARAMS_DEPTH1_TASK_REPOSITORY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/params/depth0/stateful_param_repository.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class param_repository
 * @ingroup params depth1
 *
 * @brief Collection of all parameter parsers and parse results needed
 * by the \ref depth1::foraging_controller.
 */
class param_repository: public depth0::stateful_param_repository {
 public:
  explicit param_repository(std::shared_ptr<rcppsw::er::server>& server);
};

NS_END(depth1, params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_DEPTH1_PARAM_REPOSITORY_HPP_ */
