/**
 * \file likelihood_cache_search.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "fordyca/strategy/explore/localized_search.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace ds {
class dpo_store;
} /* namespace ds */

NS_START(strategy, explore);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class likelihood_cache_search
 * \ingroup strategy explore
 *
 * \brief Vector to the last known location of a cache, then begin performing
 * CRW at that location, with the idea being that the likelihood of another
 * cache being nearby is higher, given that you've found one there before.
 */
class likelihood_cache_search : public localized_search {
 public:
  likelihood_cache_search(const fstrategy::strategy_params* const c_params,
                          rmath::rng* rng);

  ~likelihood_cache_search(void) override = default;
  likelihood_cache_search(const likelihood_cache_search&) = delete;
  likelihood_cache_search& operator=(const likelihood_cache_search&) = delete;

  /* taskable overrides */
  void task_start(cta::taskable_argument*) override;

  /* prototype overrides */
  std::unique_ptr<csstrategy::base_strategy> clone(void) const override {
    csfsm::fsm_params fsm_params {
      saa(),
      inta_tracker(),
      nz_tracker(),
    };
    fstrategy::strategy_params params {
      &fsm_params,
      nullptr,
      nullptr,
      accessor(),
      {}
    };

    return std::make_unique<likelihood_cache_search>(&params, rng());
  }
};

NS_END(explore, strategy, fordyca);

