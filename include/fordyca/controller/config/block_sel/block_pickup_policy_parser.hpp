/**
 * \file block_pickup_policy_parser.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include <string>
#include <memory>

#include "rcppsw/config/xml/xml_config_parser.hpp"

#include "fordyca/controller/config/block_sel/block_pickup_policy_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config, block_sel);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_pickup_policy_parser
 * \ingroup controller config block_sel
 *
 * \brief Parses XML parameters for \ref block_pickup_policy_config at the
 * start of simulation.
 */
class block_pickup_policy_parser : public rer::client<block_pickup_policy_parser>,
                                   public rconfig::xml::xml_config_parser {
 public:
  using config_type = block_pickup_policy_config;

  block_pickup_policy_parser(void)
      : ER_CLIENT_INIT("fordyca.controller.config.block_sel.block_pickup_policy_parser") {}

  /**
   * \brief The root tag that all block sel matrix parameters should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "block_pickup_policy";

  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }
 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

NS_END(block_sel, config, controller, fordyca);

