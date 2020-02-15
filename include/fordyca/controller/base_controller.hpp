/**
 * \file base_controller.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include <memory>
#include <string>
#include <typeindex>

#include "cosm/controller/irv_recipient_controller.hpp"
#include "cosm/pal/argos_controller2D_adaptor.hpp"
#include "cosm/metrics/config/output_config.hpp"

#include "fordyca/controller/block_manip_collator.hpp"
#include "fordyca/fordyca.hpp"
#include "fordyca/fsm/subsystem_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem::config {
struct actuation_subsystem2D_config;
struct sensing_subsystem2D_config;
} // namespace cosm::subsystem::config
namespace cosm::steer2D::config {
struct force_calculator_config;
}
namespace cosm::tv {
class robot_dynamics_applicator;
}
namespace cosm::repr {
class base_block2D;
}

namespace rcppsw::math::config {
struct rng_config;
} // namespace rcppsw::math::config

NS_START(fordyca);

namespace repr {
class line_of_sight;
} // namespace repr

NS_START(controller);
class base_perception_subsystem;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_controller
 * \ingroup controller
 *
 * \brief The base controller foraging class that all FORDYCA controllers derive
 * from. It holds all functionality common to all controllers, as well that some
 * that is stubbed out here, but overridden in derived classes which allows this
 * class to be used as the robot controller handle when rendering QT graphics
 * overlays.
 */
class base_controller : public cpal::argos_controller2D_adaptor,
                        public ccontroller::irv_recipient_controller,
                        public rer::client<base_controller> {
 public:
  base_controller(void) RCSW_COLD;
  ~base_controller(void) override RCSW_COLD;

  base_controller(const base_controller&) = delete;
  base_controller& operator=(const base_controller&) = delete;

  /* base_controller2D overrides */
  void init(ticpp::Element& node) override RCSW_COLD;
  void reset(void) override RCSW_COLD;
  rtypes::type_uuid entity_id(void) const override final;

  /* rda_recipient_controller overrides */
  double applied_movement_throttle(void) const override final;
  void irv_init(const ctv::robot_dynamics_applicator* rda) override final;

  /**
   * \brief By default controllers have no perception subsystem, and are
   * basically blind centipedes.
   */
  virtual const base_perception_subsystem* perception(void) const {
    return nullptr;
  }

  /**
   * \brief By default controllers have no perception subsystem, and are
   * basically blind centipedes.
   */
  virtual base_perception_subsystem* perception(void) { return nullptr; }

  /**
   * \brief If \c TRUE, the robot is currently at least most of the way in the
   * nest, as reported by the sensors.
   */
  bool in_nest(void) const;

  /**
   * \brief Return if the robot is currently carrying a block.
   */
  bool is_carrying_block(void) const { return nullptr != m_block; }

  /**
   * \brief Return the block robot is carrying, or NULL if the robot is not
   * currently carrying a block.
   */
  const crepr::base_block2D* block(void) const { return m_block.get(); }
  crepr::base_block2D* block(void) { return m_block.get(); }

  /**
   * \brief Release the held block as part of a drop operation.
   */
  std::unique_ptr<crepr::base_block2D> block_release(void);

  /**
   * \brief Set the block that the robot is carrying. We use a unique_ptr to
   * convey that the robot owns the block it picks up from a C++ point of
   * view. In actuality it gets a clone of the block in the arena map.
   */
  void block(std::unique_ptr<crepr::base_block2D> block);

  /**
   * \brief If \c TRUE, then the robot thinks that it is on top of a block.
   *
   * On rare occasions this may be a false positive, which is why it is also
   * checked in the loop functions before passing any events to the
   * controller. One such occasion that is known to occur is the first timestep,
   * because the sensors have not yet finished initializing, and will return the
   * values that are incidentally the same as those that correspond to a block
   * being found.
   */
  bool block_detected(void) const;

  const class block_manip_collator* block_manip_collator(void) const {
    return &m_block_manip;
  }
  class block_manip_collator* block_manip_collator(void) {
    return &m_block_manip;
  }

 protected:
  class crfootbot::footbot_saa_subsystem* saa(void) RCSW_PURE;
  const class crfootbot::footbot_saa_subsystem* saa(void) const RCSW_PURE;

 private:
  void saa_init(
      const csubsystem::config::actuation_subsystem2D_config* actuation_p,
      const csubsystem::config::sensing_subsystem2D_config* sensing_p);
  void output_init(const cmconfig::output_config* outputp);

  /* clang-format off */
  class block_manip_collator           m_block_manip{};
  std::unique_ptr<crepr::base_block2D> m_block;
  /* clang-format on */
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_BASE_CONTROLLER_HPP_ */
