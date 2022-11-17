/**
 * \file birtd_mdpo_controller.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/cognitive/d2/birtd_dpo_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class birtd_mdpo_controller
 * \ingroup controller cognitive d2
 *
 * \brief A controller defining the task allocation space via BIfurcating
 * Recursive Task Decomposition (BIRTD) and spliting the \ref generalist task
 * into the \ref harvester, and \ref collector tasks, and then each of the \ref
 * harvester and \ref collector tasks into two subtasks as well, according to
 * dynamic changes in the environment and/or execution/interface times of the
 * tasks.
 *
 * Uses a Mapped DPO data store for tracking arena state and object relevance.
 */
class birtd_mdpo_controller : public d2::birtd_dpo_controller,
                            public rer::client<birtd_mdpo_controller> {
 public:
  birtd_mdpo_controller(void) RCPPSW_COLD;
  ~birtd_mdpo_controller(void) override RCPPSW_COLD;

  /* foraging_controller overrides */
  void init(ticpp::Element& node) override RCPPSW_COLD;
  std::type_index type_index(void) const override { return typeid(*this); }

  void shared_init(const config::d2::controller_repository& config_repo) RCPPSW_COLD;
};

NS_END(cognitive, d2, controller, fordyca);

