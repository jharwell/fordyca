/**
 * \file mdpo_controller.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_D0_MDPO_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_D0_MDPO_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/cognitive/d0/dpo_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace config { namespace d0 { class mdpo_controller_repository; }}

NS_START(controller, cognitive);

class mdpo_perception_subsystem;

NS_START(d0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class mdpo_controller
 * \ingroup controller cognitive d0
 *
 * \brief A foraging controller that:
 *
 * - Models/tracks the state of the environment (empty, unknown, contains
 *   object, etc.)
 * - Models/tracks the seen objects in the environment.
 *
 * It shares the underlying FSM with the \ref dpo_controller so that the metrics
 * collection functions can be reused.
 */
class mdpo_controller : public dpo_controller,
                        public rer::client<mdpo_controller> {
 public:
  mdpo_controller(void) RCSW_COLD;
  ~mdpo_controller(void) override RCSW_COLD;

  /* foraging_controller overrides */
  void init(ticpp::Element& node) override RCSW_COLD;
  void control_step(void) override;
  std::type_index type_index(void) const override {return typeid(*this); }

  mdpo_perception_subsystem* mdpo_perception(void) RCSW_PURE;
  const mdpo_perception_subsystem* mdpo_perception(void) const RCSW_PURE;

  /**
   * \brief Initialization that derived classes may also need to perform, if the
   * want to use any of the following parts of this class's functionality as-is:
   *
   * - MDPO perception subsystem (\ref mdpo_perception_subsystem)
   * - Block selection matrix (\ref block_sel_matrix)
   */
  void shared_init(const config::d0::mdpo_controller_repository& config_repo) RCSW_COLD;

 private:
  /**
   * \brief Perform initialization private to this class:
   *
   * - DPO FSM
   *
   * Called after \ref shared_init() in \ref init().
   */
  void private_init(const config::d0::mdpo_controller_repository& config_repo) RCSW_COLD;
};

NS_END(cognitive, d0, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_D0_MDPO_CONTROLLER_HPP_ */
