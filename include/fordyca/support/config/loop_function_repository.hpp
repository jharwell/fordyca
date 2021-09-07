/**
 * \file loop_function_repository.hpp
 *
 * Handles parsing of all XML parameters at runtime.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_CONFIG_LOOP_FUNCTION_REPOSITORY_HPP_
#define INCLUDE_FORDYCA_SUPPORT_CONFIG_LOOP_FUNCTION_REPOSITORY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/pal/config/xml/base_sm_repository.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class loop_function_repository
 * \ingroup support config
 *
 * \brief Extra loop function XML parsers/results specific to the FORDYCA
 * project.
 */
class loop_function_repository : public cpal::config::xml::base_sm_repository {
 public:
  loop_function_repository(void) noexcept RCPPSW_COLD;
};

NS_END(config, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_CONFIG_LOOP_FUNCTION_REPOSITORY_HPP_ */
