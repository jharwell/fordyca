/**
 * @file tv_manager_params.hpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_PARAMS_TV_TV_CONTROLLER_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_TV_TV_CONTROLLER_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcppsw/params/base_params.hpp"
#include "rcppsw/control/waveform_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, tv);
namespace rct = rcppsw::control;

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct tv_manager_params
 * @ingroup fordyca params tv
 */
struct tv_manager_params : public rcppsw::params::base_params {
  rct::waveform_params block_manipulation_penalty{};
  rct::waveform_params block_carry_throttle{};
  rct::waveform_params cache_usage_penalty{};
};

NS_END(tv, params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_TV_TV_CONTROLLER_PARAMS_HPP_ */
