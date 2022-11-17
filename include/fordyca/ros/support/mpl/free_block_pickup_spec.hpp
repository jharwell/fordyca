/**
 * \file free_block_pickup_spec.hpp
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

#include "fordyca/support/interactor_status.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, ros, support, mpl, detail);

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
template<typename TFreeBlockPickupVisitor>
struct free_pickup_spec_value {
  using robot_block_pickup_visitor_type = TFreeBlockPickupVisitor;
  using interactor_status_type = fsupport::interactor_status;
};

/*
 * First argument is the map as it is built, second is the thing to insert,
 * built from each type in the specified typelist.
 */
template<typename TFreeBlockPickupVisitor>
using free_pickup_inserter = boost::mpl::insert<
  boost::mpl::_1,
  boost::mpl::pair<boost::mpl::_2,
                   free_pickup_spec_value<
                     TFreeBlockPickupVisitor
                     >
                   >
  >;

NS_END(detail);

template<typename TTypelist,
         typename TFreeBlockPickupVisitor>
using free_block_pickup_spec = typename boost::mpl::fold<
  TTypelist,
  boost::mpl::map0<>,
  detail::free_pickup_inserter<
    TFreeBlockPickupVisitor
    >
  >::type;

NS_END(mpl, support, ros, fordyca);

