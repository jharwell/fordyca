/**
 * \file ledtaxis.hpp
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

#include "fordyca/strategy/foraging_strategy.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, strategy, explore);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ledtaxis
 * \ingroup strategy explore
 *
 * \brief Assumes that the target entity type isequipped with an LED entity that
 * robots can detect with their blob camera. Performs phototaxis towards the
 * source of the signal if one is found, and performs phototaxis to it.
 */
class ledtaxis : public foraging_strategy,
                 public rer::client<ledtaxis> {
 public:
  ledtaxis(const fstrategy::strategy_params* params, rmath::rng* rng);

  ~ledtaxis(void) override = default;
  ledtaxis(const ledtaxis&) = delete;
  ledtaxis& operator=(const ledtaxis&) = delete;

  /* taskable overrides */
  void task_start(cta::taskable_argument*) override final {
    m_task_running = true;
  }

  void task_reset(void) override final {
    m_task_running = false;
  }

  bool task_running(void) const override final {
    return m_task_running;
  }

  bool task_finished(void) const override final;
  void task_execute(void) override final;

  /* prototype overrides */
  std::unique_ptr<cssexplore::base_explore> clone(void) const override {
    return nullptr; /* Should not be a top level explore behavior */
  }

  const rutils::color& target(void) const { return m_target; }

 private:
  /**
   * \brief The tolerance within which a robot's location has to be in order to
   * be considered having arrived at the taxis' source.
   */
  static constexpr double kARRIVAL_TOL = 1.0;

  /* clang-format off */
  mutable bool            m_task_running{false};
  rutils::color           m_target;
  /* clang-format on */
};

NS_END(explore, strategy, fordyca);
