/**
 * \file cached_block_pickup_spec.hpp
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

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, argos, support, mpl, detail);

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
template<typename TCachedBlockPickupVisitor,
         typename TCacheVanishedVisitor>
struct cached_pickup_spec_value {
  using robot_cached_block_pickup_visitor_type = TCachedBlockPickupVisitor;
  using robot_cache_vanished_visitor_type = TCacheVanishedVisitor;
};

/*
 * First argument is the map as it is built, second in the thing to insert,
 * built from each type in the specified typelist.
 */
template<typename TCachedBlockPickupVisitor,
         typename TCacheVanishedVisitor>
using cached_pickup_inserter = boost::mpl::insert<
  boost::mpl::_1,
  boost::mpl::pair<boost::mpl::_2,
                   cached_pickup_spec_value<
                     TCachedBlockPickupVisitor,
                     TCacheVanishedVisitor
                     >
                   >
  >;

NS_END(detail);

template<typename TTypelist,
         typename TCachedBlockPickupVisitor,
         typename TCacheVanishedVisitor>
using cached_block_pickup_spec = typename boost::mpl::fold<
  TTypelist,
  boost::mpl::map0<>,
  detail::cached_pickup_inserter<
    TCachedBlockPickupVisitor,
    TCacheVanishedVisitor
    >
  >::type;
NS_END(mpl, support, argos, fordyca);

