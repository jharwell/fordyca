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

#ifndef INCLUDE_FORDYCA_FSM_EXPSTRAT_UTILITY_CACHE_SEARCH_HPP_
#define INCLUDE_FORDYCA_FSM_EXPSTRAT_UTILITY_CACHE_SEARCH_HPP_

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
 * \class utility_cache_search
 * \ingroup fsm expstrat
 *
 * \brief Using \ref cache_site_selector and the average location of all known
 * blocks (or the robot's current location if there are not any known blocks),
 * compute where a cache would ideally be located, and vector to it, begining
 * performing CRW at that location.
 */
class utility_cache_search : public localized_search {
 public:
  utility_cache_search(const foraging_expstrat::params* const c_params,
                       rmath::rng* rng)
      : utility_cache_search(c_params->csel_matrix,
                             c_params->dpo_store,
                             c_params->saa,
                             rng) {}
  utility_cache_search(const controller::cache_sel_matrix* csel_matrix,
                       const ds::dpo_store* store,
                       crfootbot::footbot_saa_subsystem* saa,
                       rmath::rng* rng)
      : localized_search(saa, rng),
        mc_matrix(csel_matrix),
        mc_store(store) {}

  ~utility_cache_search(void) override = default;
  utility_cache_search(const utility_cache_search&) = delete;
  utility_cache_search& operator=(const utility_cache_search&) = delete;

  /* taskable overrides */
  void task_start(const cta::taskable_argument*) override;

  /* prototype overrides */
  std::unique_ptr<cfsm::expstrat::base_expstrat> clone(void) const override {
    return std::make_unique<utility_cache_search>(mc_matrix,
                                                  mc_store,
                                                  saa(),
                                                  rng());
  }

 private:
  /* clang-format off */
  const controller::cache_sel_matrix* mc_matrix;
  const ds::dpo_store*                mc_store;
  /* clang-format on */
};

NS_END(expstrat, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_EXPSTRAT_UTILITY_CACHE_SEARCH_HPP_ */
