/**
 * \file perception_parser.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#include "fordyca/subsystem/perception/config/perception_config.hpp"
#include "fordyca/subsystem/perception/config/dpo_parser.hpp"
#include "fordyca/subsystem/perception/config/mdpo_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, subsystem, perception, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class perception_parser
 * \ingroup subsystem perception config
 *
 * \brief Parses XML parameters for various perception subsystems into
 * \ref perception_config.
 */
class perception_parser final : public rer::client<perception_parser>,
                                public rconfig::xml::xml_config_parser {
 public:
  using config_type = perception_config;

  perception_parser(void)
      : ER_CLIENT_INIT("fordyca.subsystem.perception.config.perception_parser") {}

  /**
   * \brief The root tag that all perception  parameters should lie under in
   * the XML tree.
   */
  inline static std::string kXMLRoot = "perception";

  bool validate(void) const override RCPPSW_ATTR(pure, cold);
  void parse(const ticpp::Element& node) override RCPPSW_COLD;

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  dpo_parser                   m_dpo{};
  mdpo_parser                  m_mdpo{};
  /* clang-format on */
};

NS_END(config, perception, subsystem, fordyca);

