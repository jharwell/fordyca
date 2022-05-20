/**
 * \file nest_interactor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
