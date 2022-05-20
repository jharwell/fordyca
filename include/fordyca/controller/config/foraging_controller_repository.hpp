/**
 * \file foraging_controller_repository.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/controller/config/xml/base_controller_repository.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class foraging_controller_repository
 * \ingroup controller config
 *
 * \brief Collection of all parameter parsers and parse results needed by
 * all foraging controllers.
 */
class foraging_controller_repository
    : public virtual ccontroller::config::xml::base_controller_repository {
 public:
  foraging_controller_repository(void) RCPPSW_COLD;
};

NS_END(config, controller, fordyca);
