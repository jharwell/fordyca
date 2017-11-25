/**
 * @file base_repository.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_BASE_REPOSITORY_HPP_
#define INCLUDE_FORDYCA_PARAMS_BASE_REPOSITORY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <map>
#include "rcppsw/common/common.hpp"
#include "rcppsw/common/base_params.hpp"
#include "fordyca/params/base_parser.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/factory/sharing_factory.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);
namespace factory = rcppsw::patterns::factory;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class base_repository {
 public:
  base_repository(void) : m_parsers(), m_factory() {}

  void parse_all(argos::TConfigurationNode& node);
  const struct rcppsw::common::base_params* get_params(const std::string& name) {
    return m_parsers[name]->get_results();
  }
  void show_all(std::ostream& stream);

 protected:
  factory::sharing_factory<base_parser>& factory(void) {
    return m_factory;
  }
  std::map<std::string, base_parser*>& parsers(void) {
    return m_parsers;
  }

 private:
  std::map<std::string, base_parser*> m_parsers;
  factory::sharing_factory<base_parser> m_factory;
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_BASE_REPOSITORY_HPP_ */
