/**
 * \file likelihood_block_search.hpp
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

#ifndef INCLUDE_FORDYCA_STRATEGY_EXPLORE_LIKELIHOOD_BLOCK_SEARCH_HPP_
#define INCLUDE_FORDYCA_STRATEGY_EXPLORE_LIKELIHOOD_BLOCK_SEARCH_HPP_

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
class likelihood_block_search : public localized_search {
 public:
  likelihood_block_search(const foraging_strategy::params* const c_params,
                          rmath::rng* rng);

  likelihood_block_search(csubsystem::saa_subsystemQ3D* saa,
                          const fsperception::known_objects_accessor* accessor,
                          rmath::rng* rng);

  ~likelihood_block_search(void) override = default;
  likelihood_block_search(const likelihood_block_search&) = delete;
  likelihood_block_search& operator=(const likelihood_block_search&) = delete;

  /* taskable overrides */
  void task_start(cta::taskable_argument*) override final;

  /* prototype overrides */
  std::unique_ptr<csstrategy::base_strategy> clone(void) const override {
    return std::make_unique<likelihood_block_search>(saa(),
                                                     accessor(),
                                                     rng());
  }
};

NS_END(explore, strategy, fordyca);

#endif /* INCLUDE_FORDYCA_STRATEGY_EXPLORE_LIKELIHOOD_BLOCK_SEARCH_HPP_ */
