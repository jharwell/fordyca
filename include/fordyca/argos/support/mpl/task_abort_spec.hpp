/**
 * \file task_abort_spec.hpp
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

#include "fordyca/argos/support/tv/env_dynamics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, argos, support, mpl, detail);

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
template<typename TFreeBlockDropVisitor>
struct task_abort_spec_value {
  using arena_map_type = carena::caching_arena_map;
  using env_dynamics_type = tv::env_dynamics;
  using robot_free_block_drop_visitor_type = TFreeBlockDropVisitor;
};

/*
 * First argument is the map as it is built, second in the thing to insert,
 * built from each type in the specified typelist.
 */
template<typename TFreeBlockDropVisitor>
using task_abort_inserter = boost::mpl::insert<
  boost::mpl::_1,
  boost::mpl::pair<boost::mpl::_2,
                   task_abort_spec_value<
                     TFreeBlockDropVisitor>
                   >
  >;

NS_END(detail);

template<typename TTypelist, typename TFreeBlockDropVisitor>
using task_abort_spec = typename boost::mpl::fold<
  TTypelist,
  boost::mpl::map0<>,
  detail::task_abort_inserter<
    TFreeBlockDropVisitor
    >
  >::type;

NS_END(mpl, support, argos, fordyca);
