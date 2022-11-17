/**
 * \file free_block_drop_spec.hpp
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

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, mpl, detail);

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
template<typename TFreeBlockDropVisitor>
struct free_block_drop_spec_value {
  using robot_free_block_drop_visitor_type = TFreeBlockDropVisitor;
};

/*
 * First argument is the map as it is built, second in the thing to insert,
 * built from each type in the specified typelist.
 */
template<typename TFreeBlockDropVisitor>
using free_block_drop_inserter = boost::mpl::insert<
  boost::mpl::_1,
  boost::mpl::pair<boost::mpl::_2,
                   free_block_drop_spec_value<
                     TFreeBlockDropVisitor>
                   >
  >;

NS_END(detail);

template<typename TTypelist, typename TFreeBlockDropVisitor>
using free_block_drop_spec = typename boost::mpl::fold<
  TTypelist,
  boost::mpl::map0<>,
  detail::free_block_drop_inserter<
    TFreeBlockDropVisitor
    >
  >::type;

NS_END(mpl, support, fordyca);

