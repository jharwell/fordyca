/**
 * \file block_pickup_base_visit_set.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/mpl/typelist.hpp"

#include "cosm/repr/base_block3D.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/subsystem/perception/perception_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::ds {
class arena_map;
} /* namespace cosm::foraging::ds */

NS_START(fordyca, events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \ingroup events detail
 *
 * \brief Interface specifying the core class of classes any action involving
 * dropping a block will need to visit (think data structures).
 */
using block_pickup_base_visit_typelist = rmpl::typelist<cfds::arena_map,
                                                        fspds::dpo_semantic_map,
                                                        fspds::dpo_store,
                                                        crepr::base_block3D>;
NS_END(detail, events, fordyca);
