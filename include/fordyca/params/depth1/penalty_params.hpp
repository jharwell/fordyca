/**
 * @file penalty_params.hpp
 *
 * @copyright 2018 John Harwell/Anthony Chen, All rights reserved.
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

 #ifndef INCLUDE_FORDYCA_PARAMS_PENALTY_PARAMS_HPP_
 #define INCLUDE_FORDYCA_PARAMS_PENALTY_PARAMS_HPP_

 /*******************************************************************************
  * Includes
  ******************************************************************************/
 #include <string>
 #include "rcppsw/common/base_params.hpp"

 /*******************************************************************************
  * Namespaces
  ******************************************************************************/
 NS_START(fordyca, params);

 /*******************************************************************************
  * Structure Definitions
  ******************************************************************************/
 /**
  * @struct penalty_params
  * @ingroup params
  */
 enum penalty_function {
    kSine,
    kSquare,
    kStep,
    kSaw,
    kNull
  };

 struct penalty_params : public rcppsw::common::base_params {
   enum penalty_function pen_func{kNull};
   int amp{0};
   int per{0};
   int phase{0};
   int square{0};
   int step{0};
   int saw{0};
 };

 NS_END(params, fordyca);

 #endif // INCLUDE_FORDYCA_PARAMS_PENALTY_PARAMS_HPP_
