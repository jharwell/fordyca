/**
 * \file controller_fwd.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_CONTROLLER_FWD_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_CONTROLLER_FWD_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include "rcppsw/mpl/typelist.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Macros
 ******************************************************************************/
#define DEPTH0_NON_ORACULAR_CONTROLLER_TYPES                              \
  controller::depth0::crw_controller, controller::depth0::dpo_controller, \
      controller::depth0::mdpo_controller

#define DEPTH0_ORACULAR_CONTROLLER_TYPES \
  controller::depth0::odpo_controller, controller::depth0::omdpo_controller

#define DEPTH0_CONTROLLER_TYPES \
  DEPTH0_NON_ORACULAR_CONTROLLER_TYPES, DEPTH0_ORACULAR_CONTROLLER_TYPES

#define DEPTH1_ORACULAR_CONTROLLER_TYPES    \
  controller::depth1::bitd_odpo_controller, \
      controller::depth1::bitd_omdpo_controller

#define DEPTH1_NON_ORACULAR_CONTROLLER_TYPES \
  controller::depth1::bitd_dpo_controller,   \
      controller::depth1::bitd_mdpo_controller

#define DEPTH1_CONTROLLER_TYPES \
  DEPTH1_NON_ORACULAR_CONTROLLER_TYPES, DEPTH1_ORACULAR_CONTROLLER_TYPES

#define DEPTH2_ORACULAR_CONTROLLER_TYPES     \
  controller::depth2::birtd_odpo_controller, \
      controller::depth2::birtd_omdpo_controller

#define DEPTH2_NON_ORACULAR_CONTROLLER_TYPES \
  controller::depth2::birtd_dpo_controller,  \
      controller::depth2::birtd_mdpo_controller

#define DEPTH2_CONTROLLER_TYPES \
  DEPTH2_NON_ORACULAR_CONTROLLER_TYPES, DEPTH2_ORACULAR_CONTROLLER_TYPES

#define ORACULAR_CONTROLLER_TYPES                                     \
  DEPTH0_ORACULAR_CONTROLLER_TYPES, DEPTH1_ORACULAR_CONTROLLER_TYPES, \
      DEPTH2_ORACULAR_CONTROLLER_TYPES

#define NON_ORACULAR_CONTROLLER_TYPES                                         \
  DEPTH0_NON_ORACULAR_CONTROLLER_TYPES, DEPTH1_NON_ORACULAR_CONTROLLER_TYPES, \
      DEPTH2_NON_ORACULAR_CONTROLLER_TYPES

#define CONTROLLER_TYPES \
  ORACULAR_CONTROLLER_TYPES, NON_ORACULAR_CONTROLLER_TYPES

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, controller);
class base_controller;
namespace depth0 {
class crw_controller;
class dpo_controller;
class odpo_controller;
class mdpo_controller;
class omdpo_controller;
using oracular_typelist = rmpl::typelist<DEPTH0_ORACULAR_CONTROLLER_TYPES>;
using non_oracular_typelist =
    rmpl::typelist<DEPTH0_NON_ORACULAR_CONTROLLER_TYPES>;
using typelist = rmpl::typelist<DEPTH0_ORACULAR_CONTROLLER_TYPES,
                                DEPTH0_NON_ORACULAR_CONTROLLER_TYPES>;
} // namespace depth0
namespace depth1 {
class bitd_dpo_controller;
class bitd_mdpo_controller;
class bitd_odpo_controller;
class bitd_omdpo_controller;
using oracular_typelist = rmpl::typelist<DEPTH1_ORACULAR_CONTROLLER_TYPES>;
using typelist = rmpl::typelist<DEPTH1_ORACULAR_CONTROLLER_TYPES,
                                DEPTH1_NON_ORACULAR_CONTROLLER_TYPES>;
} // namespace depth1
namespace depth2 {
class birtd_dpo_controller;
class birtd_mdpo_controller;
class birtd_odpo_controller;
class birtd_omdpo_controller;
using oracular_typelist = rmpl::typelist<DEPTH2_ORACULAR_CONTROLLER_TYPES>;
using typelist = rmpl::typelist<DEPTH2_ORACULAR_CONTROLLER_TYPES,
                                DEPTH2_NON_ORACULAR_CONTROLLER_TYPES>;
} // namespace depth2
using oracular_typelist = rmpl::typelist<ORACULAR_CONTROLLER_TYPES>;
using non_oracular_typelist = rmpl::typelist<NON_ORACULAR_CONTROLLER_TYPES>;
using typelist =
    rmpl::typelist<ORACULAR_CONTROLLER_TYPES, NON_ORACULAR_CONTROLLER_TYPES>;
using d1d2_typelist = rmpl::typelist<DEPTH1_ORACULAR_CONTROLLER_TYPES,
                                     DEPTH1_NON_ORACULAR_CONTROLLER_TYPES,
                                     DEPTH2_ORACULAR_CONTROLLER_TYPES,
                                     DEPTH2_NON_ORACULAR_CONTROLLER_TYPES>;

template <typename T>
using is_depth0 =
    typename boost::mpl::contains<controller::depth0::typelist, T>::type;

template <typename T>
using is_depth1 =
    typename boost::mpl::contains<controller::depth1::typelist, T>::type;

template <typename T>
using is_depth2 =
    typename boost::mpl::contains<controller::depth2::typelist, T>::type;

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_CONTROLLER_FWD_HPP_ */
