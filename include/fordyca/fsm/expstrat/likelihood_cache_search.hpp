/**
 * @file likelihood_cache_search.hpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_FSM_EXPSTRAT_LIKELIHOOD_CACHE_SEARCH_HPP_
#define INCLUDE_FORDYCA_FSM_EXPSTRAT_LIKELIHOOD_CACHE_SEARCH_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "fordyca/fsm/expstrat/localized_search.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace ds {
class dpo_store;
} /* namespace ds */

NS_START(fsm, expstrat);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class likelihood_cache_search
 * @ingroup fordyca fsm expstrat
 *
 * @brief Vector to the last known location of a cache, then begin performing
 * CRW at that location, with the idea being that the likelihood of another
 * cache being nearby is higher, given that you've found one there before.
 */
class likelihood_cache_search : public localized_search {
 public:
  explicit likelihood_cache_search(const base_expstrat::params* const c_params)
      : likelihood_cache_search(c_params->saa, c_params->store) {}
  likelihood_cache_search(controller::saa_subsystem* saa,
                          const ds::dpo_store* store)
      : localized_search(saa),
        mc_store(store) {}

  ~likelihood_cache_search(void) override = default;
  likelihood_cache_search(const likelihood_cache_search&) = delete;
  likelihood_cache_search& operator=(const likelihood_cache_search&) = delete;

  /* taskable overrides */
  void task_start(const rta::taskable_argument*) override;

  /* prototype overrides */
  std::unique_ptr<base_expstrat> clone(void) const override {
    return std::make_unique<likelihood_cache_search>(saa_subsystem(),
                                                       mc_store);
  }

 private:
  /* clang-format off */
  const ds::dpo_store* mc_store;
  /* clang-format on */
};

NS_END(expstrat, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_EXPSTRAT_LIKELIHOOD_CACHE_SEARCH_HPP_ */
