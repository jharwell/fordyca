/**
 * \file nest_block_drop_spec.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/mpl/map.hpp>
#include <boost/mpl/fold.hpp>

#include "rcppsw/mpl/typelist.hpp"

#include "cosm/arena/caching_arena_map.hpp"

#include "fordyca/argos/support/tv/block_op_penalty_handler.hpp"
#include "fordyca/support/interactor_status.hpp"
#include "fordyca/argos/metrics/d0/d0_metrics_manager.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, argos, support, mpl, detail);

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
template<typename TNestBlockProcessVisitor>
struct nest_drop_spec_value {
  using arena_map_type = carena::caching_arena_map;
  using penalty_handler_type = tv::block_op_penalty_handler;
  using interactor_status_type = fsupport::interactor_status;
  using metrics_manager_type = fametrics::d0::d0_metrics_manager;
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
NS_END(mpl, support, argos, fordyca);

