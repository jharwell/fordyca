/**
 * \file foraging_expstrat.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fsm/expstrat/foraging_expstrat.hpp"

#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm, expstrat);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
foraging_expstrat::foraging_expstrat(crfootbot::footbot_saa_subsystem* saa,
                                     rmath::rng* rng)
    : base_expstrat(saa, rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
crfootbot::footbot_saa_subsystem* foraging_expstrat::saa(void) const {
  return static_cast<crfootbot::footbot_saa_subsystem*>(base_expstrat::saa());
} /* saa() */

crfootbot::footbot_saa_subsystem* foraging_expstrat::saa(void) {
  return static_cast<crfootbot::footbot_saa_subsystem*>(base_expstrat::saa());
} /* saa() */

NS_END(expstrat, fsm, fordyca);
