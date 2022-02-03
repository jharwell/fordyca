/**
 * \file cached_block_pickup_spec.hpp
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

#ifndef INCLUDE_FORDYCA_ARGOS_SUPPORT_MPL_CACHED_BLOCK_PICKUP_SPEC_HPP_
#define INCLUDE_FORDYCA_ARGOS_SUPPORT_MPL_CACHED_BLOCK_PICKUP_SPEC_HPP_

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

#endif /* INCLUDE_FORDYCA_ARGOS_SUPPORT_MPL_CACHED_BLOCK_PICKUP_SPEC_HPP_ */
