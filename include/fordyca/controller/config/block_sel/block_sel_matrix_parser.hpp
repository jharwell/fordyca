/**
 * \file block_sel_matrix_parser.hpp
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
#include <string>
#include <memory>

#include "rcppsw/config/xml/xml_config_parser.hpp"

#include "fordyca/controller/config/block_sel/block_priorities_parser.hpp"
#include "fordyca/controller/config/block_sel/block_pickup_policy_parser.hpp"
#include "fordyca/controller/config/block_sel/block_sel_matrix_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config, block_sel);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_sel_matrix_parser
 * \ingroup config block_sel
 *
 * \brief Parses XML parameters for the \ref block_sel_matrix at the start
 * of simulation.
 */
class block_sel_matrix_parser final : public rer::client<block_sel_matrix_parser>,
                                      public rconfig::xml::xml_config_parser {
 public:
  using config_type = block_sel_matrix_config;

  block_sel_matrix_parser(void)
      : ER_CLIENT_INIT("fordyca.controller.config.block_sel.block_sel_matrix_parser") {}

  /**
   * \brief The root tag that all block sel matrix parameters should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "block_sel_matrix";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(pure, cold);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  block_priorities_parser      m_priorities{};
  block_pickup_policy_parser   m_pickup_policy{};
  /* clang-format on */
};

NS_END(block_sel, config, controller, fordyca);

