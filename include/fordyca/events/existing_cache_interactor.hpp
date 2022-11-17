/**
 * \file existing_cache_interactor.hpp
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
 * \class existing_cache_interactor
 * \ingroup tasks d1
 *
 * \brief Interactor specifying the event visit set for all foraging tasks that
 * interact with existing caches in FORDYCA.
 */
class existing_cache_interactor
    : public rpvisitor::polymorphic_accept_set<fccd1::events::cache_block_drop,
                                               fccd1::events::cached_block_pickup,
                                               fccd1::events::cache_vanished,

                                               fccd2::events::cache_block_drop,
                                               fccd2::events::cached_block_pickup,
                                               fccd2::events::cache_vanished> {};

NS_END(events, fordyca);
