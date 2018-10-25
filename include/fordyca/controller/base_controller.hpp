/**
 * @file base_controller.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_BASE_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_BASE_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/vector2.h>
#include <string>
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation {
class base_block;
class line_of_sight;
} // namespace representation
namespace params {
struct output_params;
}

NS_START(controller);

class saa_subsystem;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class base_controller
 * @ingroup controller
 *
 * @brief The base controller foraging class that all FORDYCA controllers derive
 * from. It holds all functionality common to all controllers, as well that some
 * that is stubbed out here, but overridden in derived classes which allows this
 * class to be used as the robot controller handle when rendering QT graphics
 * overlays.
 */
class base_controller : public argos::CCI_Controller,
                        public rcppsw::er::client<base_controller> {
 public:
  base_controller(void);
  ~base_controller(void) override = default;

  base_controller(const base_controller& other) = delete;
  base_controller& operator=(const base_controller& other) = delete;

  /* CCI_Controller overrides */
  void Init(ticpp::Element& node) override;
  void Reset(void) override;

  /**
   * @brief Get the ID of the entity. Argos also provides this, but it doesn't
   * work in gdb, so I provide my own.
   */
  int entity_id(void) const;

  /**
   * @brief Set whether or not a robot is supposed to display it's ID above its
   * head during simulation.
   */
  void display_id(bool display_id) { m_display_id = display_id; }

  /**
   * @brief If \c TRUE, then the robot should display its ID above its head
   * during simulation.
   */
  bool display_id(void) const { return m_display_id; }

  /**
   * @brief If \c TRUE, the robot is currently at least most of the way in the
   * nest, as reported by the sensors.
   */
  bool in_nest(void) const;

  /**
   * @brief Return if the robot is currently carrying a block.
   */
  bool is_carrying_block(void) const { return nullptr != m_block; }

  /**
   * @brief Return the block robot is carrying, or NULL if the robot is not
   * currently carrying a block.
   */
  std::shared_ptr<representation::base_block> block(void) const {
    return m_block;
  }

  /**
   * @brief Set the block that the robot is carrying.
   */
  void block(const std::shared_ptr<representation::base_block>& block) {
    m_block = block;
  }

  /**
   * @brief If \c TRUE, then the robot thinks that it is on top of a block.
   *
   * On rare occasions this may be a false positive, which is why it is also
   * checked in the loop functions before passing any events to the
   * controller. One such occasion that is known to occur is the first timestep,
   * because the sensors have not yet finished initializing, and will return the
   * values that are incidentally the same as those that correspond to a block
   * being found.
   */
  bool block_detected(void) const;

  /**
   * @brief Set the current clock tick.
   *
   * In a real world, each robot would maintain its own clock tick, and overall
   * there would no doubt be considerable skew; this is a simulation hack that
   * makes things much nicer/easier to deal with.
   */
  void tick(uint tick);

  /**
   * @brief Set the current location of the robot.
   *
   * This is a hack, as real world robot's would have to do their own
   * localization. This is far superior to that, in terms of ease of
   * programming. Plus it helps me focus in on my actual research. Ideally,
   * robots would calculate this from sensor values, rather than it being set by
   * the loop functions.
   */
  void robot_loc(argos::CVector2 loc);
  argos::CVector2 robot_loc(void) const;

  /**
   * @brief Convenience function to add footbot ID to salient messages during
   * loop function execution (timestep is already there).
   */
  void ndc_push(void) { ER_NDC_PUSH("[" + GetId() + "]"); }

  /**
   * @brief Convenience function to add footbot ID+timestep to messages during
   * \ref ControlStep().
   */
  void ndc_pusht(void);

  /**
   * @brief Remove the last NDC.
   */
  void ndc_pop(void) { ER_NDC_POP(); }

 protected:
  const class saa_subsystem* saa_subsystem(void) const { return m_saa.get(); }
  class saa_subsystem* saa_subsystem(void) {
    return m_saa.get();
  }

 private:
  void output_init(const struct params::output_params* params);

  // clang-format off
  bool                                        m_display_id{false};
  std::shared_ptr<representation::base_block> m_block{nullptr};
  std::unique_ptr<controller::saa_subsystem>  m_saa;
  // clang-format on
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_BASE_CONTROLLER_HPP_ */
