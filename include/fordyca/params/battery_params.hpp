/**
 * @file battery_params.hpp
 *
 * @copyright 2018 Nimer WazWaz, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_PARAMS_BATTERY_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_BATTERY_PARAMS_HPP_

/*******************************************************************************
  * Includes
******************************************************************************/
#include <string>
#include "rcppsw/params/base_params.hpp"

/*******************************************************************************
  * Namespaces
******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
  * Structure Definitions
******************************************************************************/
/**
  * @struct battery_params
  * @ingroup params
  */
struct battery_params : public rcppsw::params::base_params {
  int power_station_amount{0};
  float power_station_size{1};
  std::string power_station_distrubtion{"normal"};
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_BATTERY_PARAMS_HPP_ */
