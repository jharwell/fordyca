/**
 * \file env_dynamics_parser.hpp
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

#ifndef INCLUDE_FORDYCA_CONFIG_TV_ENV_DYNAMICS_PARSER_HPP_
#define INCLUDE_FORDYCA_CONFIG_TV_ENV_DYNAMICS_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "rcppsw/config/xml/xml_config_parser.hpp"

#include "cosm/tv/config/xml/temporal_penalty_parser.hpp"

#include "fordyca/config/tv/env_dynamics_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class env_dynamics_parser
 * \ingroup config tv
 *
 * \brief Parses XML parameters for \ref env_dynamics into \ref
 * env_dynamics_config.
 */
class env_dynamics_parser final : public rconfig::xml::xml_config_parser {
 public:
  using config_type = env_dynamics_config;

  env_dynamics_parser(void);

  /**
   * \brief The root tag that all temporal variance parameters should lie under
   * in the XML tree.
   */
  static inline const std::string kXMLRoot = "env_dynamics";

  void parse(const ticpp::Element& node) override;
  bool validate(void) const override RCPPSW_ATTR(const, cold);

  std::string xml_root(void) const override { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type>              m_config{nullptr};
  ctv::config::xml::temporal_penalty_parser m_motion{};
  ctv::config::xml::temporal_penalty_parser m_block_manip{};
  ctv::config::xml::temporal_penalty_parser m_block_carry{};
  ctv::config::xml::temporal_penalty_parser m_cache_usage{};
  /* clang-format on */
};

NS_END(tv, config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_TV_ENV_DYNAMICS_PARSER_HPP_ */
