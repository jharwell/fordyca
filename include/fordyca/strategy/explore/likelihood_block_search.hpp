/**
 * \file likelihood_block_search.hpp
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
#include "fordyca/subsystem/perception/known_objects_accessor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, strategy, explore);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class likelihood_block_search
 * \ingroup strategy explore
 *
 * \brief Vector to the last known location of a block, then begin performing
 * CRW at that location, with the idea being that the likelihood of another
 * block being nearby is higher, given that you've found one there before.
 */
class likelihood_block_search final : public localized_search {
 public:
  likelihood_block_search(const fstrategy::strategy_params* const c_params,
                          rmath::rng* rng);

  ~likelihood_block_search(void) override = default;
  likelihood_block_search(const likelihood_block_search&) = delete;
  likelihood_block_search& operator=(const likelihood_block_search&) = delete;

  /* taskable overrides */
  void task_start(cta::taskable_argument*) override final;

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

    return std::make_unique<likelihood_block_search>(&params, rng());
  }
};

NS_END(explore, strategy, fordyca);

