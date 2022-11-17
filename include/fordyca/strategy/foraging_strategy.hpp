/**
 * \file foraging_strategy.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/explore/base_explore.hpp"

#include "fordyca/strategy/strategy_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, strategy);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class foraging_strategy
 * \ingroup strategy
 *
 * \brief Base class for different behaviors that controllers can
 * exhibit when looking for stuff, avoiding collision, etc.
 */
class foraging_strategy : public cssexplore::base_explore {
 public:
  foraging_strategy(const fstrategy::strategy_params* params, rmath::rng* rng)
      : base_explore(params->fsm, params->explore, rng),
        mc_accessor(params->accessor) {}

  foraging_strategy(const foraging_strategy&) = delete;
  foraging_strategy& operator=(const foraging_strategy&) = delete;

 protected:
  const fsperception::known_objects_accessor* accessor(void) const {
    return mc_accessor;
  }

  /* clang-format off */
  const fsperception::known_objects_accessor* mc_accessor;
  /* clang-format on */
};

NS_END(strategy, fordyca);
