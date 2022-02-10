/**
 * \file cognitive_controller.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#include "fordyca/controller/foraging_controller.hpp"
#include "fordyca/subsystem/perception/perception_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cognitive_controller
 * \ingroup controller cognitive
 *
 * \brief A foraging controller that has SOME model of what it has seen over
 * time (as distinct from reactive controllers which do not have such a model ).
 */
class cognitive_controller : public foraging_controller,
                             public rer::client<cognitive_controller> {
 public:
  cognitive_controller(void) RCPPSW_COLD;
  ~cognitive_controller(void) override RCPPSW_COLD;

  /* foraging_controller overrides */
  void reset(void) override RCPPSW_COLD;

  double los_dim(void) const RCPPSW_PURE;

  /**
   * \brief Set whether or not a robot is supposed to display it's LOS as a
   * square of the appropriate size during simulation.
   */
  void display_los(bool display_los) { m_display_los = display_los; }

  /**
   * \brief If \c TRUE, then the robot should display its approximate LOS as a
   * circle on the ground during simulation.
   */
  bool display_los(void) const { return m_display_los; }

  const fsperception::foraging_perception_subsystem* perception(void) const {
    return m_perception.get();
  }
  fsperception::foraging_perception_subsystem* perception(void) {
    return m_perception.get();
  }

 protected:
  /**
   * \brief Mutator to allow replacement of the perception subsystem object
   * managed by the the controller (strategy pattern), so that derived classes
   * can reuse the same accessors that this classes provides. Cleaner to do it
   * this way than to have each derived class have its own private version and
   * require duplicate accessors in each derived class.
   */
  void perception(std::unique_ptr<fsperception::foraging_perception_subsystem> perception);

 private:
  /* clang-format off */
  bool                                                         m_display_los{false};
  std::unique_ptr<fsperception::foraging_perception_subsystem> m_perception;
  /* clang-format on */
};

NS_END(cognitive, controller, fordyca);

