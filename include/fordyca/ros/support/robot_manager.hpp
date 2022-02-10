/**
 * \file robot_manager.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/config/rng_config.hpp"

#include "cosm/ros/robot_manager_adaptor.hpp"
#include "cosm/ros/config/xml/robot_manager_repository.hpp"
#include "cosm/ros/config/sierra_config.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::pal::config {
struct output_config;
} /* namespace cosm::metrics::config */

NS_START(fordyca, ros, support);


/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class robot_manager
 * \ingroup ros support
 *
 * \brief The base loop functions in FORDYCA that all other loop functions
 * inherit from when FORDYCA is built for ROS.
 */
class robot_manager : public cros::robot_manager_adaptor,
                      public rer::client<robot_manager> {
 public:
  robot_manager(const cros::config::sierra_config* config) RCPPSW_COLD;
  ~robot_manager(void) override RCPPSW_COLD;

  /* Not copy constructible/assignable by default */
  robot_manager(const robot_manager& s) = delete;
  robot_manager& operator=(const robot_manager& s) = delete;

  /* swarm manager overrides */
  void init(ticpp::Element&) override RCPPSW_COLD;
  void reset(void) override {}
  void pre_step(void) override;
  void post_step(void) override {}

  /* robot manager adaptor overrides */
  bool experiment_finished(void) const override;

 protected:
  const cros::config::xml::robot_manager_repository* config(void) const {
    return &m_config;
  }
  cros::config::xml::robot_manager_repository* config(void) {
    return &m_config;
  }
  void config_parse(ticpp::Element& node) RCPPSW_COLD;

 private:
  /* clang-format off */
  const cros::config::sierra_config           mc_sierra;
  cros::config::xml::robot_manager_repository m_config{};
  /* clang-format on */
};

NS_END(support, ros, fordyca);
