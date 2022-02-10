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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include "rcppsw/mpl/typelist.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Depth0 Macros
 ******************************************************************************/
#define D0_REACTIVE_CONTROLLER_TYPES            \
  fcontroller::reactive::d0::crw_controller

#define D0_NON_ORACULAR_CONTROLLER_TYPES        \
  fcontroller::cognitive::d0::dpo_controller,   \
    fcontroller::cognitive::d0::mdpo_controller

#define D0_ORACULAR_CONTROLLER_TYPES                    \
  fcontroller::cognitive::d0::odpo_controller,          \
    fcontroller::cognitive::d0::omdpo_controller

#define D0_COGNITIVE_CONTROLLER_TYPES                   \
  fcontroller::cognitive::d0::dpo_controller,           \
    fcontroller::cognitive::d0::mdpo_controller,        \
    fcontroller::cognitive::d0::odpo_controller,        \
    fcontroller::cognitive::d0::omdpo_controller

#define D0_CONTROLLER_TYPES                                             \
  D0_NON_ORACULAR_CONTROLLER_TYPES, D0_ORACULAR_CONTROLLER_TYPES

/*******************************************************************************
 * Depth1 Macros
 ******************************************************************************/
#define D1_ORACULAR_CONTROLLER_TYPES                    \
  fcontroller::cognitive::d1::bitd_odpo_controller,     \
    fcontroller::cognitive::d1::bitd_omdpo_controller

#define D1_NON_ORACULAR_CONTROLLER_TYPES                \
  fcontroller::cognitive::d1::bitd_dpo_controller,      \
    fcontroller::cognitive::d1::bitd_mdpo_controller

#define D1_REACTIVE_CONTROLLER_TYPES

#define D1_COGNITIVE_CONTROLLER_TYPES                   \
  fcontroller::cognitive::d1::bitd_dpo_controller,      \
    fcontroller::cognitive::d1::btd_mdpo_controller,    \
    fcontroller::cognitive::d1::bitd_odpo_controller,   \
    fcontroller::cognitive::d1::bitd_omdpo_controller

#define D1_CONTROLLER_TYPES                                             \
  D1_NON_ORACULAR_CONTROLLER_TYPES, D1_ORACULAR_CONTROLLER_TYPES

/*******************************************************************************
 * Depth2 Macros
 ******************************************************************************/
#define D2_ORACULAR_CONTROLLER_TYPES                    \
  fcontroller::cognitive::d2::birtd_odpo_controller,    \
    fcontroller::cognitive::d2::birtd_omdpo_controller

#define D2_NON_ORACULAR_CONTROLLER_TYPES                \
  fcontroller::cognitive::d2::birtd_dpo_controller,     \
    fcontroller::cognitive::d2::birtd_mdpo_controller

#define D2_REACTIVE_CONTROLLER_TYPES

#define D2_COGNITIVE_CONTROLLER_TYPES                   \
  fcontroller::cognitive::d2::birtd_dpo_controller,     \
    fcontroller::cognitive::d2::birtd_mdpo_controller,  \
    fcontroller::cognitive::d2::birtd_odpo_controller,  \
    fcontroller::cognitive::d2::birtd_omdpo_controller

#define D2_CONTROLLER_TYPES                                             \
  D2_NON_ORACULAR_CONTROLLER_TYPES, D2_ORACULAR_CONTROLLER_TYPES

/*******************************************************************************
 * Other Macros
 ******************************************************************************/
#define ORACULAR_CONTROLLER_TYPES                               \
  D0_ORACULAR_CONTROLLER_TYPES, D1_ORACULAR_CONTROLLER_TYPES,   \
    D2_ORACULAR_CONTROLLER_TYPES

#define NON_ORACULAR_CONTROLLER_TYPES                                   \
  D0_NON_ORACULAR_CONTROLLER_TYPES, D1_NON_ORACULAR_CONTROLLER_TYPES,   \
    D2_NON_ORACULAR_CONTROLLER_TYPES

#define CONTROLLER_TYPES ORACULAR_CONTROLLER_TYPES, NON_ORACULAR_CONTROLLER_TYPES

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, controller);
class foraging_controller;
namespace reactive::d0 {
class crw_controller;
}
namespace cognitive::d0 {
class dpo_controller;
class odpo_controller;
class mdpo_controller;
class omdpo_controller;
} // namespace cognitive::d0

namespace d0 {
using reactive_typelist = rmpl::typelist<D0_REACTIVE_CONTROLLER_TYPES>;
using cognitive_typelist = rmpl::typelist<D0_ORACULAR_CONTROLLER_TYPES,
                                          D0_NON_ORACULAR_CONTROLLER_TYPES>;

using oracular_typelist = rmpl::typelist<D0_ORACULAR_CONTROLLER_TYPES>;
using non_oracular_typelist = rmpl::typelist<D0_NON_ORACULAR_CONTROLLER_TYPES>;
using typelist = rmpl::typelist<D0_REACTIVE_CONTROLLER_TYPES,
                                D0_ORACULAR_CONTROLLER_TYPES,
                                D0_NON_ORACULAR_CONTROLLER_TYPES>;
template <typename T>
using is_cognitive = typename boost::mpl::contains<fcontroller::d0::cognitive_typelist, T>::type;

template <typename T>
using is_reactive = typename boost::mpl::contains<fcontroller::d0::reactive_typelist, T>::type;
} // namespace d0

namespace cognitive::d1 {
class bitd_dpo_controller;
class bitd_mdpo_controller;
class bitd_odpo_controller;
class bitd_omdpo_controller;
} // namespace cognitive::d1

namespace d1 {
using cognitive_typelist = rmpl::typelist<D1_ORACULAR_CONTROLLER_TYPES,
                                          D1_NON_ORACULAR_CONTROLLER_TYPES>;
using oracular_typelist = rmpl::typelist<D1_ORACULAR_CONTROLLER_TYPES>;
using typelist = rmpl::typelist<D1_ORACULAR_CONTROLLER_TYPES,
                                D1_NON_ORACULAR_CONTROLLER_TYPES>;
} // namespace d1

namespace cognitive::d2 {
class birtd_dpo_controller;
class birtd_mdpo_controller;
class birtd_odpo_controller;
class birtd_omdpo_controller;
} // namespace cognitive::d2

namespace d2 {
using cognitive_typelist = rmpl::typelist<D2_ORACULAR_CONTROLLER_TYPES,
                                          D2_NON_ORACULAR_CONTROLLER_TYPES>;
using oracular_typelist = rmpl::typelist<D2_ORACULAR_CONTROLLER_TYPES>;
using typelist = rmpl::typelist<D2_ORACULAR_CONTROLLER_TYPES,
                                D2_NON_ORACULAR_CONTROLLER_TYPES>;
} // namespace d2

using oracular_typelist = rmpl::typelist<ORACULAR_CONTROLLER_TYPES>;
using non_oracular_typelist = rmpl::typelist<NON_ORACULAR_CONTROLLER_TYPES>;
using typelist =
    rmpl::typelist<ORACULAR_CONTROLLER_TYPES, NON_ORACULAR_CONTROLLER_TYPES>;
using d1d2_typelist = rmpl::typelist<D1_ORACULAR_CONTROLLER_TYPES,
                                     D1_NON_ORACULAR_CONTROLLER_TYPES,
                                     D2_ORACULAR_CONTROLLER_TYPES,
                                     D2_NON_ORACULAR_CONTROLLER_TYPES>;

template <typename T>
using is_d0 = typename boost::mpl::contains<fcontroller::d0::typelist, T>::type;

template <typename T>
using is_d1 = typename boost::mpl::contains<fcontroller::d1::typelist, T>::type;

template <typename T>
using is_d2 = typename boost::mpl::contains<fcontroller::d2::typelist, T>::type;

NS_END(controller, fordyca);

