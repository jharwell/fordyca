/**
 * \file bitd_mdpo_controller.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/cognitive/d1/bitd_dpo_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class bitd_mdpo_controller
 * \ingroup controller cognitive d1
 *
 * \brief A controller defining the task allocation space via BIfurcating Task
 * Decomposition (BITD) and spliting the \ref generalist task into the \ref
 * harvester, and \ref collector tasks, according to dynamic changes in the
 * environment and/or execution/interface times of the tasks.
 *
 * Uses a Mapped DPO (MDPO) data store for tracking arena state and object
 * relavance.
 *
 * Note that this class does not inherit from \ref d0::mdpo_controller,
 * because that would mean duplicating all of the executive setup
 * logic/callbacks that are also present in the \ref d1::bitd_dpo_controller.
 * Cleaner to do it this way.
 */
class bitd_mdpo_controller : public d1::bitd_dpo_controller,
                           public rer::client<bitd_mdpo_controller> {
 public:
  bitd_mdpo_controller(void) RCPPSW_COLD;
  ~bitd_mdpo_controller(void) override RCPPSW_COLD;

  /* foraging_controller overrides */
  void init(ticpp::Element& node) override RCPPSW_COLD;
  void control_step(void) override;
  std::type_index type_index(void) const override { return typeid(*this); }

 protected:
  /**
   * \brief Initialization that derived classes may also need to perform, if
   * they want to use any of the following parts of this class's functionality
   * as-is:
   *
   * - Block selection matrix (\ref block_sel_matrix)
   * - Cache selection matrix (\ref cache_sel_matrix)
   * - Task executive (\ref cta::bi_tdgraph_executive)
   * - MDPO perception subsystem (\ref mdpo_perception_subsystem)
   *
   * \param config_repo Handle to parameter repository for this controller
   *                   (after parsing and validation).
   */
  void shared_init(const config::d1::controller_repository& config_repo) RCPPSW_COLD;

 private:
};

NS_END(cognitive, d1, controller, fordyca);

