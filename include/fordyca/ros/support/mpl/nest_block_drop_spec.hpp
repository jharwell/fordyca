/**
 * \file nest_block_drop_spec.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
#include <boost/mpl/map.hpp>
#include <boost/mpl/fold.hpp>

#include "rcppsw/mpl/typelist.hpp"

#include "fordyca/support/interactor_status.hpp"
#include "fordyca/ros/metrics/d0/d0_robot_metrics_manager.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, ros, support, mpl, detail);

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
template<typename TNestBlockProcessVisitor>
struct nest_drop_spec_value {
  using interactor_status_type = fsupport::interactor_status;
  using robot_metrics_manager_type = frmetrics::d0::d0_robot_metrics_manager;
  using robot_nest_block_process_visitor_type = TNestBlockProcessVisitor;
};

/*
 * First argument is the map as it is built, second in the thing to insert,
 * built from each type in the specified typelist.
 */
template<typename TNestBlockProcessVisitor>
using nest_drop_inserter = boost::mpl::insert<
  boost::mpl::_1,
  boost::mpl::pair<boost::mpl::_2,
                   nest_drop_spec_value<
                     TNestBlockProcessVisitor
                     >
                   >
  >;

NS_END(detail);

template<typename TTypelist, typename TNestBlockProcessVisitor>
using nest_block_drop_spec = typename boost::mpl::fold<
  TTypelist,
  boost::mpl::map0<>,
  detail::nest_drop_inserter<
    TNestBlockProcessVisitor
    >
  >::type;
NS_END(mpl, support, ros, fordyca);

