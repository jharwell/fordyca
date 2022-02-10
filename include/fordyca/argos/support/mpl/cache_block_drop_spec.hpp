/**
 * \file cache_block_drop_spec.hpp
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
template<typename TCacheBlockDropVisitor,
         typename TCacheVanishedVisitor>
struct cache_drop_spec_value {
  using robot_cache_block_drop_visitor_type = TCacheBlockDropVisitor;
  using robot_cache_vanished_visitor_type = TCacheVanishedVisitor;
};

/*
 * First argument is the map as it is built, second in the thing to insert,
 * built from each type in the specified typelist.
 */
template<typename TCacheBlockDropVisitor,
         typename TCacheVanishedVisitor>
using cache_drop_inserter = boost::mpl::insert<
  boost::mpl::_1,
  boost::mpl::pair<boost::mpl::_2,
                   cache_drop_spec_value<
                     TCacheBlockDropVisitor,
                     TCacheVanishedVisitor
                     >
                   >
  >;

NS_END(detail);

template<typename TTypelist,
         typename TCacheBlockDropVisitor,
         typename TCacheVanishedVisitor
         >
using cache_block_drop_spec = typename boost::mpl::fold<
  TTypelist,
  boost::mpl::map0<>,
  detail::cache_drop_inserter<
    TCacheBlockDropVisitor,
    TCacheVanishedVisitor
    >
  >::type;
NS_END(mpl, support, argos, fordyca);

