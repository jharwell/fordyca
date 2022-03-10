/**
 * \file utility_cache_search.hpp
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
  std::unique_ptr<csstrategy::base_strategy> clone(void) const override {
    csfsm::fsm_params fsm_params {
      saa(),
      inta_tracker(),
      nz_tracker(),
    };
    fstrategy::strategy_params params {
      &fsm_params,
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

