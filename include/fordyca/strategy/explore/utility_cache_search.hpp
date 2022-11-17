/**
 * \file utility_cache_search.hpp
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
NS_START(fordyca, strategy, explore);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class utility_cache_search
 * \ingroup strategy explore
 *
 * \brief Using \ref cache_site_selector and the average location of all known
 * blocks (or the robot's current location if there are not any known blocks),
 * compute where a cache would ideally be located, and vector to it, begining
 * performing CRW at that location.
 */
class utility_cache_search final : public localized_search {
 public:
  utility_cache_search(const fstrategy::strategy_params* const c_params,
                       rmath::rng* rng);

  ~utility_cache_search(void) override = default;
  utility_cache_search(const utility_cache_search&) = delete;
  utility_cache_search& operator=(const utility_cache_search&) = delete;

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
      mc_matrix,
      accessor(),
      {}
    };

    return std::make_unique<utility_cache_search>(&params, rng());
  }

 private:
  /* clang-format off */
  const controller::cognitive::cache_sel_matrix* mc_matrix;
  /* clang-format on */
};

NS_END(explore, strategy, fordyca);
