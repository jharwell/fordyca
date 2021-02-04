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

#ifndef INCLUDE_FORDYCA_STRATEGY_EXPLORE_LEDTAXIS_CACHE_SEARCH_HPP_
#define INCLUDE_FORDYCA_STRATEGY_EXPLORE_LEDTAXIS_CACHE_SEARCH_HPP_

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
class ledtaxis_cache_search : public foraging_strategy,
                              public rer::client<ledtaxis_cache_search> {
 public:
  explicit ledtaxis_cache_search(const foraging_strategy::params* const c_params,
                                 rmath::rng* rng)
      : ledtaxis_cache_search(c_params->saa,
                              c_params->ledtaxis_target,
                              rng) {}
  ledtaxis_cache_search(crfootbot::footbot_saa_subsystem* saa,
                        const rutils::color& ledtaxis_target,
                        rmath::rng* rng)
      : foraging_strategy(saa, rng),
        ER_CLIENT_INIT("fordyca.fsm.strategy.ledtaxis_cache_search"),
        m_crw(saa, rng),
        m_taxis(saa, ledtaxis_target, rng) {}

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

  /* interference metrics */
  bool exp_interference(void) const override final RCPPSW_PURE;
  bool entered_interference(void) const override final RCPPSW_PURE;
  bool exited_interference(void) const override final RCPPSW_PURE;
  rtypes::timestep interference_duration(void) const override final;
  rmath::vector3z interference_loc3D(void) const override final RCPPSW_PURE;

  /* prototype overrides */
  std::unique_ptr<csstrategy::base_strategy> clone(void) const override {
    return std::make_unique<ledtaxis_cache_search>(saa(),
                                                   m_taxis.target(),
                                                   rng());
  }

 private:
  /* clang-format off */
  crw_adaptor m_crw;
  ledtaxis    m_taxis;
  /* clang-format on */
};

NS_END(explore, strategy, fordyca);

#endif /* INCLUDE_FORDYCA_STRATEGY_EXPLORE_LEDTAXIS_CACHE_SEARCH_HPP_ */
