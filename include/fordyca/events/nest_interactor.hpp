/**
 * \file nest_interactor.hpp
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
 * \class nest_interactor
 * \ingroup events
 *
 * \brief Interactor specifying the event visit set for all foraging tasks that
 * interact with the nest in FORDYCA.
 */
class nest_interactor
    : public rpvisitor::polymorphic_accept_set<fccd0::events::nest_block_drop,
                                               fccd1::events::nest_block_drop,
                                               fccd2::events::nest_block_drop> {};

NS_END(tasks, fordyca);
