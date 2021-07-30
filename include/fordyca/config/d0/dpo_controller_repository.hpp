/**
 * \file dpo_controller_repository.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONFIG_D0_DPO_CONTROLLER_REPOSITORY_HPP_
#define INCLUDE_FORDYCA_CONFIG_D0_DPO_CONTROLLER_REPOSITORY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/config/perceptive_controller_repository.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, d0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dpo_controller_repository
 * \ingroup config d0
 *
 * \brief Collection of all parameter parsers and parse results needed by
 * \ref dpo_controller.
 */
class dpo_controller_repository: public perceptive_controller_repository {
 public:
  dpo_controller_repository(void) RCPPSW_COLD;
};

NS_END(d0, config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_D0_DPO_CONTROLLER_REPOSITORY_HPP_ */
