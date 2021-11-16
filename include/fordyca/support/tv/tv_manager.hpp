/**
 * \file tv_manager.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_TV_TV_MANAGER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_TV_TV_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/tv/tv_manager.hpp"

#include "fordyca/support/tv/env_dynamics.hpp"
#include "fordyca/support/tv/fordyca_pd_adaptor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
using tv_manager = ctv::tv_manager<env_dynamics, fordyca_pd_adaptor>;

NS_END(tv, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_TV_TV_MANAGER_HPP_ */
