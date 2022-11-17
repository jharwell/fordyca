/**
 * \file block_pickup_policy_parser.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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

