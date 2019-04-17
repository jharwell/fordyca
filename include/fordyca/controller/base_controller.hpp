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
#include <string>
#include <typeindex>

#include "fordyca/controller/block_manip_collator.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/metrics/fsm/movement_metrics.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace support { namespace tv {
class tv_manager;
}} // namespace support::tv

namespace repr {
class base_block;
class line_of_sight;
} // namespace repr
namespace params {
struct output_params;
struct sensing_params;
struct actuation_params;
} // namespace params

NS_START(controller);
class base_perception_subsystem;
class saa_subsystem;
namespace rmath = rcppsw::math;
namespace er = rcppsw::er;

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
                        public metrics::fsm::movement_metrics,
                        public metrics::fsm::goal_acquisition_metrics,
                        public rcppsw::er::client<base_controller> {
 public:
  base_controller(void);
  ~base_controller(void) override;

  base_controller(const base_controller& other) = delete;
  base_controller& operator=(const base_controller& other) = delete;

  /* CCI_Controller overrides */
  void Init(ticpp::Element& node) override;
  void Reset(void) override;

  virtual std::type_index type_index(void) const = 0;

  /* movement metrics */
  double distance(void) const override;
  rmath::vector2d velocity(void) const override;

  /**
   * @brief By default controllers have no perception subsystem, and are
   * basically blind centipedes.
   */
  virtual const base_perception_subsystem* perception(void) const {
    return nullptr;
  }

  /**
   * @brief By default controllers have no perception subsystem, and are
   * basically blind centipedes.
   */
  virtual base_perception_subsystem* perception(void) { return nullptr; }

  /**
   * @brief Return the applied motion throttling for the robot. This is not
   * necessarily the same as the active/configured throttling.
   */
  double applied_motion_throttle(void) const;

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
  std::shared_ptr<repr::base_block> block(void) const { return m_block; }

  /**
   * @brief Set the block that the robot is carrying.
   */
  void block(const std::shared_ptr<repr::base_block>& block) {
    m_block = block;
  }

  void tv_init(const support::tv::tv_manager* tv_manager);

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

  const class block_manip_collator* block_manip_collator(void) const {
    return &m_block_manip;
  }
  class block_manip_collator* block_manip_collator(void) {
    return &m_block_manip;
  }

  /**
   * @brief Set the current location of the robot.
   *
   * This is a hack, as real world robot's would have to do their own
   * localization. This is far superior to that, in terms of ease of
   * programming. Plus it helps me focus in on my actual research. Ideally,
   * robots would calculate this from sensor values, rather than it being set by
   * the loop functions.
   */
  void position(const rmath::vector2d& loc);
  void discrete_position(const rmath::vector2u& loc);
  const rmath::vector2d& position(void) const;
  const rmath::vector2u& discrete_position(void) const;
  rmath::vector2d heading(void) const;

  /**
   * @brief Convenience function to add footbot ID to salient messages during
   * loop function execution (timestep is already there).
   */
  void ndc_push(void) { ER_NDC_PUSH("[" + GetId() + "]"); }

  /**
   * @brief Convenience function to add footbot ID+timestep to messages during
   * the control step.
   */
  void ndc_pusht(void);

  /**
   * @brief Remove the last NDC.
   */
  void ndc_pop(void) { ER_NDC_POP(); }

 protected:
  class saa_subsystem* saa_subsystem(void) {
    return m_saa.get();
  }
  const class saa_subsystem* saa_subsystem(void) const { return m_saa.get(); }

 private:
  void output_init(const struct params::output_params* params);
  void saa_init(const params::actuation_params* actuation_p,
                const params::sensing_params* sensing_p);

  /* clang-format off */
  const support::tv::tv_manager*             m_tv_manager{nullptr};
  bool                                       m_display_id{false};
  std::shared_ptr<repr::base_block>          m_block{nullptr};
  std::unique_ptr<controller::saa_subsystem> m_saa;
  class block_manip_collator                 m_block_manip{};
  /* clang-format on */
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_BASE_CONTROLLER_HPP_ */
