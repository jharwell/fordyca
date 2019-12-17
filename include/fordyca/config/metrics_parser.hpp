/**
 * \file metrics_parser.hpp
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

#ifndef INCLUDE_FORDYCA_CONFIG_METRICS_PARSER_HPP_
#define INCLUDE_FORDYCA_CONFIG_METRICS_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <string>

#include "cosm/cosm.hpp"
#include "cosm/pal/config/xml/metrics_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class metrics_parser
 * \ingroup fordyca config
 *
 * \brief Parses XML parameters related to metric collection into \ref
 * metrics_config (actually the parent class does this, this cass just fulfills
 * a small interface).
 */
class metrics_parser final : public cpconfig::xml::metrics_parser {
 public:
  using config_type = metrics_parser::config_type;

  /**
   * \brief The XML attributes specifying supported metric collectors.
   */
  std::list<std::string> xml_attr(void) const override {
    auto base = cpconfig::xml::metrics_parser::xml_attr();
    std::list<std::string> ret = {
        "cache_acq_counts",
        "cache_acq_locs",
        "cache_acq_explore_locs",
        "cache_acq_vector_locs",
        "cache_utilization",
        "cache_lifecycle",
        "cache_locations",
        "cache_site_selection",

        "task_execution_generalist",
        "task_execution_collector",
        "task_execution_harvester",
        "task_execution_cache_starter",
        "task_execution_cache_finisher",
        "task_execution_cache_transferer",
        "task_execution_cache_collector",

        "task_tab_generalist",
        "task_tab_harvester",
        "task_tab_collector",

        "task_distribution",

        "perception_mdpo",
        "perception_dpo",

        "tv_environment",
    };
    std::for_each(base.begin(), base.end(), [&](const auto& s) {
      ret.push_back(s);
    });
    return ret;
  };
};

NS_END(config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_METRICS_PARSER_HPP_ */
