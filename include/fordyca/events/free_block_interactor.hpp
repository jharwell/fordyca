/**
 * \file free_block_interactor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/visitor/polymorphic_visitable.hpp"

#include "fordyca/controller/cognitive/events_fwd.hpp"
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \class free_block_interactor
 * \ingroup events
 *
 * \brief Interactor specifying the event visit set for all foraging tasks that
 * interact with free blocks in FORDYCA.
 */
class free_block_interactor
    : public rpvisitor::polymorphic_accept_set<fccd0::events::free_block_drop,
                                               fccd0::events::free_block_pickup,
                                               fccd0::events::block_vanished,

                                               fccd1::events::free_block_drop,
                                               fccd1::events::free_block_pickup,
                                               fccd1::events::block_vanished,

                                               fccd2::events::free_block_drop,
                                               fccd2::events::free_block_pickup,
                                               fccd2::events::block_vanished> {};

NS_END(events, fordyca);
