/**
 * \file likelihood_cache_search.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
class likelihood_cache_search final : public localized_search {
 public:
  likelihood_cache_search(const fstrategy::strategy_params* const c_params,
                          rmath::rng* rng);

  ~likelihood_cache_search(void) override = default;
  likelihood_cache_search(const likelihood_cache_search&) = delete;
  likelihood_cache_search& operator=(const likelihood_cache_search&) = delete;

  /* taskable overrides */
  void task_start(cta::taskable_argument*) override;

  /* prototype overrides */
  std::unique_ptr<cssexplore::base_explore> clone(void) const override {
    csfsm::fsm_params fsm_params {
      saa(),
      inta_tracker(),
      nz_tracker(),
    };
    fstrategy::strategy_params params {
      &fsm_params,
      config(),
      nullptr,
      nullptr,
      accessor(),
      {}
    };

    return std::make_unique<likelihood_cache_search>(&params, rng());
  }
};

NS_END(explore, strategy, fordyca);
