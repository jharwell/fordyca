/**
 * @file occupancy_grid_params.hpp
 *
 * @copyright 2019 Anthony Chen/John Harwell, All rights reserved.
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

 #ifndef INCLUDE_FORDYCA_PARAMS_ENERGY_PARAMS_HPP_
 #define INCLUDE_FORDYCA_PARAMS_ENERGY_PARAMS_HPP_

 /*******************************************************************************
  * Includes
  ******************************************************************************/
 #include <argos3/core/utility/math/vector2.h>
 #include "rcppsw/params/base_params.hpp"
 /*******************************************************************************
  * Namespaces
  ******************************************************************************/
 NS_START(fordyca, params);

 /*******************************************************************************
  * Structure Definitions
  ******************************************************************************/
 /**
  * @struct energy_params
  * @ingroup params
  */
  struct energy_params : public rcppsw::params::base_params {
    float elow{-1};
    float ehigh{-1};
    float capacity{-1};
    float weight1{-1};
    float weight2{-1};
    float weight3{-1};
    float weight1C{-1};
    float weight2C{-1};
    float weight3C{-1};
    std::string EEE{""};

  };

  NS_END(params, fordyca);

  #endif /* INCLUDE_FORDYCA_PARAMS_ENERGY_PARAMS_HPP_ */
