/**
 * \file crw_adaptor.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/strategy/explore/crw_adaptor.hpp"

#include "cosm/subsystem/saa_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces
******************************************************************************/
NS_START(fordyca, strategy, explore);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
crw_adaptor::crw_adaptor(const fstrategy::strategy_params* params,
                         rmath::rng* rng)
    : foraging_strategy(params, rng),
      decorator(params->fsm, params->explore, rng) {}

NS_END(explore, strategy, fordyca);
