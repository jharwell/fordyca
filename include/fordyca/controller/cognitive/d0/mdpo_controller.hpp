/**
 * \file mdpo_controller.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/cognitive/d0/dpo_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace fordyca::controller::config::d0 { class mdpo_controller_repository; }

NS_START(fordyca, controller, cognitive);

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
  using perception_subsystem_type = mdpo_perception_subsystem;


  mdpo_controller(void) RCPPSW_COLD;
  ~mdpo_controller(void) override RCPPSW_COLD;

  /* foraging_controller overrides */
  void init(ticpp::Element& node) override RCPPSW_COLD;
  void control_step(void) override;
  std::type_index type_index(void) const override {return typeid(*this); }

  /**
   * \brief Initialization that derived classes may also need to perform, if the
   * want to use any of the following parts of this class's functionality as-is:
   *
   * - MDPO perception subsystem (\ref mdpo_perception_subsystem)
   * - Block selection matrix (\ref block_sel_matrix)
   */
  void shared_init(const config::d0::mdpo_controller_repository& config_repo) RCPPSW_COLD;

 private:
  /**
   * \brief Perform initialization private to this class:
   *
   * - DPO FSM
   *
   * Called after \ref shared_init() in \ref init().
   */
  void private_init(const config::d0::mdpo_controller_repository& config_repo) RCPPSW_COLD;
};

NS_END(cognitive, d0, controller, fordyca);

