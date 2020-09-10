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
 * Depth0 Macros
 ******************************************************************************/
#define DEPTH0_NON_ORACULAR_CONTROLLER_TYPES                            \
  controller::reactive::depth0::crw_controller,                         \
    controller::cognitive::depth0::dpo_controller,                      \
    controller::cognitive::depth0::mdpo_controller

#define DEPTH0_ORACULAR_CONTROLLER_TYPES        \
  controller::cognitive::depth0::odpo_controller,       \
    controller::cognitive::depth0::omdpo_controller

#define DEPTH0_REACTIVE_CONTROLLER_TYPES        \
  controller::cognitive::depth0::reactive::crw_controller

#define DEPTH0_COGNITIVE_CONTROLLER_TYPES       \
  controller::cognitive::depth0::dpo_controller,           \
    controller::cognitive::depth0::mdpo_controller,        \
    controller::cognitive::depth0::odpo_controller,        \
    controller::cognitive::depth0::omdpo_controller

#define DEPTH0_CONTROLLER_TYPES \
  DEPTH0_NON_ORACULAR_CONTROLLER_TYPES, DEPTH0_ORACULAR_CONTROLLER_TYPES

/*******************************************************************************
 * Depth1 Macros
 ******************************************************************************/
#define DEPTH1_ORACULAR_CONTROLLER_TYPES        \
  controller::cognitive::depth1::bitd_odpo_controller,     \
    controller::cognitive::depth1::bitd_omdpo_controller

#define DEPTH1_NON_ORACULAR_CONTROLLER_TYPES    \
  controller::cognitive::depth1::bitd_dpo_controller,      \
    controller::cognitive::depth1::bitd_mdpo_controller

#define DEPTH1_REACTIVE_CONTROLLER_TYPES

#define DEPTH1_COGNITIVE_CONTROLLER_TYPES            \
  controller::cognitive::depth1::bitd_dpo_controller,           \
    controller::cognitive::depth1::btd_mdpo_controller,         \
    controller::cognitive::depth1::bitd_odpo_controller,        \
    controller::cognitive::depth1::bitd_omdpo_controller

#define DEPTH1_CONTROLLER_TYPES                                         \
  DEPTH1_NON_ORACULAR_CONTROLLER_TYPES, DEPTH1_ORACULAR_CONTROLLER_TYPES

/*******************************************************************************
 * Depth2 Macros
 ******************************************************************************/
#define DEPTH2_ORACULAR_CONTROLLER_TYPES     \
  controller::cognitive::depth2::birtd_odpo_controller, \
      controller::cognitive::depth2::birtd_omdpo_controller

#define DEPTH2_NON_ORACULAR_CONTROLLER_TYPES \
  controller::cognitive::depth2::birtd_dpo_controller,  \
      controller::cognitive::depth2::birtd_mdpo_controller

#define DEPTH2_REACTIVE_CONTROLLER_TYPES

#define DEPTH2_COGNITIVE_CONTROLLER_TYPES       \
  controller::cognitive::depth2::birtd_dpo_controller,      \
    controller::cognitive::depth2::birtd_mdpo_controller,    \
    controller::cognitive::depth2::birtd_odpo_controller,   \
    controller::cognitive::depth2::birtd_omdpo_controller

#define DEPTH2_CONTROLLER_TYPES \
  DEPTH2_NON_ORACULAR_CONTROLLER_TYPES, DEPTH2_ORACULAR_CONTROLLER_TYPES

/*******************************************************************************
 * Other Macros
 ******************************************************************************/
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
class foraging_controller;
namespace reactive::depth0 {
class crw_controller;
}
namespace cognitive::depth0 {
class dpo_controller;
class odpo_controller;
class mdpo_controller;
class omdpo_controller;
}
namespace depth0 {
using oracular_typelist = rmpl::typelist<DEPTH0_ORACULAR_CONTROLLER_TYPES>;
using non_oracular_typelist =
    rmpl::typelist<DEPTH0_NON_ORACULAR_CONTROLLER_TYPES>;
using typelist = rmpl::typelist<DEPTH0_ORACULAR_CONTROLLER_TYPES,
                                DEPTH0_NON_ORACULAR_CONTROLLER_TYPES>;
}
namespace cognitive::depth1 {
class bitd_dpo_controller;
class bitd_mdpo_controller;
class bitd_odpo_controller;
class bitd_omdpo_controller;
}
namespace depth1 {
using oracular_typelist = rmpl::typelist<DEPTH1_ORACULAR_CONTROLLER_TYPES>;
using typelist = rmpl::typelist<DEPTH1_ORACULAR_CONTROLLER_TYPES,
                                DEPTH1_NON_ORACULAR_CONTROLLER_TYPES>;
} // namespace depth1
namespace cognitive::depth2 {
class birtd_dpo_controller;
class birtd_mdpo_controller;
class birtd_odpo_controller;
class birtd_omdpo_controller;
}
namespace depth2 {
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
