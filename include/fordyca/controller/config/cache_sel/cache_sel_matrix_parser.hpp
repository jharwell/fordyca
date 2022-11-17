/**
 * \file cache_sel_matrix_parser.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#include "fordyca/controller/config/cache_sel/cache_sel_matrix_config.hpp"
#include "fordyca/controller/config/cache_sel/cache_pickup_policy_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config, cache_sel);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_sel_matrix_parser
 * \ingroup controller config cache_sel
 *
 * \brief Parses XML parameters for the \ref cache_sel_matrix at the start
 * of simulation.
 */
class cache_sel_matrix_parser final : public rer::client<cache_sel_matrix_parser>,
                                      public rconfig::xml::xml_config_parser {
 public:
  using config_type = cache_sel_matrix_config;

  cache_sel_matrix_parser(void)
      : ER_CLIENT_INIT("fordyca.controller.config.cache_sel.cache_sel_matrix_parser") {}

  /**
   * \brief The root tag that all cache sel matrix parameters should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "cache_sel_matrix";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }
  bool validate(void) const override RCPPSW_ATTR(pure, cold);

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  cache_pickup_policy_parser   m_pickup_policy{};
  /* clang-format on */
};

NS_END(cache_sel, config, controller, fordyca);

