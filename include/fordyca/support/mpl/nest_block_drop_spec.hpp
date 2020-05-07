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

#ifndef INCLUDE_FORDYCA_SUPPORT_MPL_NEST_BLOCK_DROP_SPEC_HPP_
#define INCLUDE_FORDYCA_SUPPORT_MPL_NEST_BLOCK_DROP_SPEC_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/mpl/map.hpp>
#include <boost/mpl/fold.hpp>

#include "rcppsw/mpl/typelist.hpp"

#include "cosm/arena/caching_arena_map.hpp"

#include "fordyca/support/tv/block_op_penalty_handler.hpp"
#include "fordyca/support/interactor_status.hpp"
#include "fordyca/events/robot_nest_block_drop.hpp"
#include "fordyca/support/depth0/depth0_metrics_aggregator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, mpl, detail);

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
struct nest_drop_spec_value {
  using arena_map_type = carena::caching_arena_map;
  using penalty_handler_type = tv::block_op_penalty_handler;
  using interactor_status_type = interactor_status;
  using metrics_agg_type = depth0::depth0_metrics_aggregator;
  using robot_nest_block_drop_visitor_type = events::robot_nest_block_drop_visitor;
};

/*
 * First argument is the map as it is built, second in the thing to insert,
 * built from each type in the specified typelist.
 */
using nest_drop_inserter = boost::mpl::insert<boost::mpl::_1,
                                              boost::mpl::pair<boost::mpl::_2,
                                                               nest_drop_spec_value>>;

NS_END(detail);

template<typename TTypelist>
using nest_block_drop_spec = typename boost::mpl::fold<TTypelist,
                                                       boost::mpl::map0<>,
                                                       detail::nest_drop_inserter>::type;
NS_END(mpl, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_MPL_NEST_BLOCK_DROP_SPEC_HPP_ */
