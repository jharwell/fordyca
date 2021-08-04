/**
 * \file perception_config.hpp
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

#ifndef INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_CONFIG_PERCEPTION_CONFIG_HPP_
#define INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_CONFIG_PERCEPTION_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/config/base_config.hpp"

#include "cosm/subsystem/perception/config/rlos_config.hpp"

#include "fordyca/subsystem/perception/config/dpo_config.hpp"
#include "fordyca/subsystem/perception/config/mdpo_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, subsystem, perception, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct perception_config
 * \ingroup subsystem perception config
 *
 * \brief Configuration for robot perception.
 */
struct perception_config final : public rconfig::base_config {
  std::string type{""};

  dpo_config dpo {};
  mdpo_config mdpo {};
};

NS_END(config, perception, subsystem, fordyca);

#endif /* INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_CONFIG_PERCEPTION_CONFIG_HPP_ */
