/**
 * \file ledtaxis_cache_search.hpp
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
#include "fordyca/strategy/explore/ledtaxis.hpp"
#include "fordyca/strategy/explore/crw_adaptor.hpp"
#include "fordyca/strategy/foraging_strategy.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, strategy, explore);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ledtaxis_cache_search
 * \ingroup strategy explore
 *
 * \brief Vector to the last known location of a cache, then begin performing
 * CRW at that location, with the idea being that the ledtaxis of another
 * cache being nearby is higher, given that you've found one there before.
 */
class ledtaxis_cache_search final : public foraging_strategy,
                              public rer::client<ledtaxis_cache_search> {
 public:
  ledtaxis_cache_search(const fstrategy::strategy_params* params,
                        rmath::rng* rng);

  ~ledtaxis_cache_search(void) override = default;
  ledtaxis_cache_search(const ledtaxis_cache_search&) = delete;
  ledtaxis_cache_search& operator=(const ledtaxis_cache_search&) = delete;

  /* taskable overrides */

  /**
   * \brief Start LED taxis cache search. Crucially, this enables the camera
   * sensor for use during exploration. See FORDYCA#593.
   */
  void task_start(cta::taskable_argument*) override;

  /**
   * \brief Reset LED taxis cache search after a cache is successfully
   * discovered. Crucially, this disable the camera sensor for increased
   * computational efficiency. See FORDYCA#593.
   */
  void task_reset(void) override final;

  bool task_running(void) const override final {
    return m_taxis.task_running() || m_crw.task_running();
  }

  /**
   * \brief Since we are exploring for something we don't know about, we will
   * never finish (stopping exploration is handled at a higher level).
   */
  bool task_finished(void) const override final { return false; }
  void task_execute(void) override final;

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
      m_taxis.target()
    };
    return std::make_unique<ledtaxis_cache_search>(&params, rng());
  }

 private:
  /* clang-format off */
  crw_adaptor m_crw;
  ledtaxis    m_taxis;
  /* clang-format on */
};

NS_END(explore, strategy, fordyca);
