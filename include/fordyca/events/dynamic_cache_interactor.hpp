/**
 * \file dynamic_cache_interactor.hpp
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

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \class dynamic_cache_interactor
 * \ingroup events
 *
 * \brief Interactor specifying the event visit set for all foraging tasks that
 * interact with dynamic caches in FORDYCA.
 */
class dynamic_cache_interactor
    : public rpvisitor::polymorphic_accept_set<fccd2::events::block_proximity,
                                               fccd2::events::cache_proximity> {};

NS_END(events, fordyca);
