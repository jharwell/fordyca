/**
 * \file tv_manager_parser.hpp
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

#ifndef INCLUDE_FORDYCA_ARGOS_SUPPORT_TV_CONFIG_TV_MANAGER_PARSER_HPP_
#define INCLUDE_FORDYCA_ARGOS_SUPPORT_TV_CONFIG_TV_MANAGER_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "rcppsw/config/xml/xml_config_parser.hpp"

#include "cosm/tv/config/xml/population_dynamics_parser.hpp"

#include "fordyca/argos/support/tv/config/tv_manager_config.hpp"
#include "fordyca/argos/support/tv/config/env_dynamics_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, tv, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class tv_manager_parser
 * \ingroup argos support tv config
 *
 * \brief Parses XML parameters for \ref tv_manager into \ref tv_manager_config.
 */
class tv_manager_parser final : public rer::client<tv_manager_parser>,
                                public rconfig::xml::xml_config_parser {
 public:
  using config_type = tv_manager_config;

  tv_manager_parser(void)
      : ER_CLIENT_INIT("fordyca.argos.support.config.tv_manager_parser") {}

  /**
   * \brief The root tag that all temporal variance parameters should lie under
   * in the XML tree.
   */
  static inline const std::string kXMLRoot = "temporal_variance";

  void parse(const ticpp::Element& node) override;
  bool validate(void) const override RCPPSW_ATTR(const, cold);

  std::string xml_root(void) const override { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type>                 m_config{nullptr};
  ctv::config::xml::population_dynamics_parser m_popd{};
  env_dynamics_parser                          m_envd{};
  /* clang-format on */
};

NS_END(config, tv, support, argos, fordyca);

#endif /* INCLUDE_FORDYCA_ARGOS_SUPPORT_TV_CONFIG_TV_MANAGER_PARSER_HPP_ */
