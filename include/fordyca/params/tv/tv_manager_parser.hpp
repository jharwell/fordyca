/**
 * @file tv_manager_parser.hpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_PARAMS_TV_TV_CONTROLLER_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_TV_TV_CONTROLLER_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "fordyca/params/tv/tv_manager_params.hpp"
#include "rcppsw/control/waveform_xml_parser.hpp"
#include "rcppsw/params/xml_param_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class tv_manager_parser
 * @ingroup fordyca params tv
 *
 * @brief Parses XML parameters for \ref tv_manager into \ref tv_manager_params.
 */
class tv_manager_parser : public rcppsw::params::xml_param_parser {
 public:
  explicit tv_manager_parser(uint level)
      : xml_param_parser(level),
        m_block_manip(level + 1),
        m_block_carry(level + 1),
        m_cache_usage(level + 1) {}

  /**
   * @brief The root tag that all temporal variance parameters should lie under
   * in the XML tree.
   */
  static constexpr char kXMLRoot[] = "temporal_variance";

  void parse(const ticpp::Element& node) override;
  bool validate(void) const override;

  std::string xml_root(void) const override { return kXMLRoot; }

  std::shared_ptr<tv_manager_params> parse_results(void) const {
    return m_params;
  }

 private:
  std::shared_ptr<rcppsw::params::base_params> parse_results_impl(
      void) const override {
    return m_params;
  }

  /* clang-format off */
  std::shared_ptr<tv_manager_params> m_params{nullptr};
  rct::waveform_xml_parser              m_block_manip;
  rct::waveform_xml_parser              m_block_carry;
  rct::waveform_xml_parser              m_cache_usage;
  /* clang-format on */
};

NS_END(tv, params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_TV_TV_CONTROLLER_PARSER_HPP_ */
