/**
 * @file localized_block_search.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_EXPSTRAT_LOCALIZED_BLOCK_SEARCH_HPP_
#define INCLUDE_FORDYCA_FSM_EXPSTRAT_LOCALIZED_BLOCK_SEARCH_HPP_

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
 * @class localized_block_search
 * @ingroup fordyca fsm expstrat
 *
 * @brief An exploration behavior in which robots vector to the last location of
 * a known block and then begin exploration there.
 */
class localized_block_search : public localized_search {
 public:
  explicit localized_block_search(const base_expstrat::params* const c_params)
      : localized_block_search(c_params->saa, c_params->store) {}

  explicit localized_block_search(controller::saa_subsystem* saa,
                                  const ds::dpo_store* store)
      : localized_search(saa),
        mc_store(store) {}

  ~localized_block_search(void) override = default;
  localized_block_search(const localized_block_search&) = delete;
  localized_block_search& operator=(const localized_block_search&) = delete;

  /* taskable overrides */
  void task_start(const rta::taskable_argument*) override final;

  /* prototype overrides */
  std::unique_ptr<base_expstrat> clone(void) const override {
    return rcppsw::make_unique<localized_block_search>(saa_subsystem(),
                                                       mc_store);
  }

 private:
  /* clang-format off */
  const ds::dpo_store* mc_store;
  /* clang-format on */
};

NS_END(expstrat, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_EXPSTRAT_LOCALIZED_BLOCK_SEARCH_HPP_ */
