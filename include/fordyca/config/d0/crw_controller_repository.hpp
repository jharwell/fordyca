/**
 * \file crw_controller_repository.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONFIG_DEPTH0_CRW_CONTROLLER_REPOSITORY_HPP_
#define INCLUDE_FORDYCA_CONFIG_DEPTH0_CRW_CONTROLLER_REPOSITORY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/config/foraging_controller_repository.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, d0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class crw_controller_repository
 * \ingroup config d0
 *
 * \brief Collection of all parameter parsers and parse results needed by
 * \ref crw_controller.
 */
class crw_controller_repository: public foraging_controller_repository {
 public:
  crw_controller_repository();
};

NS_END(config, fordyca, d0);

#endif /* INCLUDE_FORDYCA_CONFIG_DEPTH0_CRW_CONTROLLER_REPOSITORY_HPP_ */
