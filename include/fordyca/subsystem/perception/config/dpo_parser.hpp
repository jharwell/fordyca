/**
 * \file dpo_parser.hpp
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

#include "cosm/subsystem/perception/rlos/config/xml/rlos_parser.hpp"
#include "cosm/subsystem/perception/config/xml/pheromone_parser.hpp"

#include "fordyca/subsystem/perception/config/dpo_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, subsystem, perception, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dpo_parser
 * \ingroup subsystem perception config
 *
 * \brief Parses XML parameters relating to the DPO perception subsystem into
 * \ref dpo_config.
 */
class dpo_parser : public rer::client<dpo_parser>,
                   public rconfig::xml::xml_config_parser {
 public:
  using config_type = dpo_config;

  dpo_parser(void)
      : ER_CLIENT_INIT("fordyca.subsystem.perception.config.dpo_parser") {}

  /**
   * \brief The root tag that all dpo parameters should lie under in the
   * XML tree.
   */
  inline static const std::string kXMLRoot = "dpo";

  bool validate(void) const override RCPPSW_ATTR(pure, cold);
  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  std::string xml_root(void) const override { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

 private:
  /* clang-format off */
  std::shared_ptr<config_type>      m_config{nullptr};
  cspconfig::xml::pheromone_parser  m_pheromone{};
  csprlos::config::xml::rlos_parser m_rlos{};

  /* clang-format on */
};

NS_END(config, perception, subsystem, fordyca);
