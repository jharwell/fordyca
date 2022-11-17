/**
 * \file base_swarm_manager.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"

#include "cosm/pal/swarm_manager.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/support/config/loop_function_repository.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class base_swarm_manager
 * \ingroup support
 *
 * \brief The base loop functions in FORDYCA that all other loop functions
 * inherit from, regardless of the platform FORDYCA is built for.
 *
 * This class is not a functional set of loop functions, but it provides
 * functions needed across multiple derived classes.
 */
class base_swarm_manager : public cpal::swarm_manager,
                           public rer::client<base_swarm_manager> {
 public:
  base_swarm_manager(void) RCPPSW_COLD;
  ~base_swarm_manager(void) override RCPPSW_COLD;

  /* Not copy constructible/assignable by default */
  base_swarm_manager(const base_swarm_manager& s) = delete;
  base_swarm_manager& operator=(const base_swarm_manager& s) = delete;

  /* swarm manager overrides */
  void init(ticpp::Element&) override RCPPSW_COLD;
  void reset(void) override RCPPSW_COLD;
  void pre_step(void) override;
  void post_step(void) override;

  config::loop_function_repository* config(void) { return &m_config; }
  const config::loop_function_repository* config(void) const { return &m_config; }
  void config_parse(ticpp::Element& node) RCPPSW_COLD;

 private:
  /* clang-format off */
  config::loop_function_repository m_config{};
  /* clang-format on */
};

NS_END(support, fordyca);
