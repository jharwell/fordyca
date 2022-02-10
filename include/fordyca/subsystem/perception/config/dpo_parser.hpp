/**
 * \file dpo_parser.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#include "cosm/subsystem/perception/config/xml/rlos_parser.hpp"
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
  std::shared_ptr<config_type>     m_config{nullptr};
  cspconfig::xml::pheromone_parser m_pheromone{};
  cspconfig::xml::rlos_parser      m_rlos{};

  /* clang-format on */
};

NS_END(config, perception, subsystem, fordyca);

