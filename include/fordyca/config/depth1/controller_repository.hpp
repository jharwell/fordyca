/**
 * @file depth1/controller_repository.hpp
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

#ifndef INCLUDE_FORDYCA_CONFIG_DEPTH1_CONTROLLER_REPOSITORY_HPP_
#define INCLUDE_FORDYCA_CONFIG_DEPTH1_CONTROLLER_REPOSITORY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/config/depth0/mdpo_controller_repository.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class controller_repository
 * @ingroup fordyca config depth1
 *
 * @brief Collection of all parameter parsers and parse results needed
 * by depth1 controllers.
 */
class controller_repository: public depth0::mdpo_controller_repository {
 public:
  controller_repository(void);
};

NS_END(depth1, config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_DEPTH1_CONTROLLER_REPOSITORY_HPP_ */
