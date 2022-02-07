/**
 * \file swarm_manager.hpp
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

#ifndef INCLUDE_FORDYCA_ROS_SUPPORT_SWARM_MANAGER_HPP_
#define INCLUDE_FORDYCA_ROS_SUPPORT_SWARM_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/config/rng_config.hpp"

#include "cosm/pal/ros/swarm_manager_adaptor.hpp"
#include "cosm/pal/ros/config/xml/swarm_manager_repository.hpp"

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
 * \class swarm_manager
 * \ingroup ros support
 *
 * \brief The base loop functions in FORDYCA that all other loop functions
 * inherit from when FORDYCA is built for ROS. This code runs on the central ROS
 * node.
 */
class swarm_manager : public cpros::swarm_manager_adaptor,
                      public rer::client<swarm_manager> {
 public:
  swarm_manager(void) RCPPSW_COLD;
  ~swarm_manager(void) override RCPPSW_COLD;

  /* Not copy constructible/assignable by default */
  swarm_manager(const swarm_manager& s) = delete;
  swarm_manager& operator=(const swarm_manager& s) = delete;

  /* swarm manager overrides */
  void init(ticpp::Element&) override RCPPSW_COLD;
  void reset(void) override {}
  void pre_step(void) override {}
  void post_step(void) override {}

 protected:
  const cpros::config::xml::swarm_manager_repository* config(void) const {
    return &m_config;
  }
  cpros::config::xml::swarm_manager_repository* config(void) {
    return &m_config;
  }
  void config_parse(ticpp::Element& node) RCPPSW_COLD;

 private:
  /* clang-format off */
  cpros::config::xml::swarm_manager_repository m_config{};
  /* clang-format on */
};

NS_END(support, ros, fordyca);

#endif /* INCLUDE_FORDYCA_ROS_SUPPORT_SWARM_MANAGER_HPP_ */
