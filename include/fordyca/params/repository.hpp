/**
 * @file repository.hpp
 *
 * Handles parsing of all XML parameters at runtime.
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_PARAMS_REPOSITORY_HPP_
#define INCLUDE_FORDYCA_PARAMS_REPOSITORY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <map>
#include "rcppsw/common/common.hpp"
#include "fordyca/params/params.hpp"
#include "fordyca/params/base_param_parser.hpp"
#include "rcppsw/common/er_client.hpp"
#include "rcppsw/patterns/factory/releasing_factory.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);
namespace factory = rcppsw::patterns::factory;
/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class repository {
 public:
  repository(void) : m_parsers(), m_factory() {}

  void parse_all(argos::TConfigurationNode& node);
  const struct base_params* get_params(const std::string& name) {
    return m_parsers[name]->get_results();
  }
  void show_all(std::ostream& stream);

 protected:
  factory::releasing_factory<base_param_parser>& factory(void) {
    return m_factory;
  }
  std::map<std::string, base_param_parser*>& parsers(void) {
    return m_parsers;
  }

 private:
  std::map<std::string, base_param_parser*> m_parsers;
  factory::releasing_factory<base_param_parser> m_factory;
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_REPOSITORY_HPP_ */
