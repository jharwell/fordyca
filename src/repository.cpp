/**
 * @file repository.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
 *
 * This file is part of RCPPSW.
 *
 * RCPPSW is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * RCPPSW is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * RCPPSW.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/params/repository.hpp"
#include <algorithm>
#include "rcsw/common/fpc.h"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void repository::init(std::shared_ptr<rcppsw::common::er_server> server) {
  deferred_init(server);
  insmod("params");
  server_handle()->dbglvl(rcppsw::common::er_lvl::OFF);
  server_handle()->loglvl(rcppsw::common::er_lvl::NOM);
} /* init() */

status_t repository::add_category(const std::string& name, base_param_parser* parser) {
  FPC_CHECK(ERROR, m_parsers.find(name) == m_parsers.end());

  m_parsers.insert(std::pair<std::string, base_param_parser*>(name, parser));
  return OK;
} /* add_category() */

status_t repository::parse_all(argos::TConfigurationNode& node) {
  std::for_each(m_parsers.begin(), m_parsers.end(), [&](std::pair<const std::string, base_param_parser*>& pair) {
      pair.second->parse(node);
    });
  return OK;
} /* parse_all() */

void repository::show_all(void) {
  std::for_each(m_parsers.begin(), m_parsers.end(), [&](std::pair<const std::string, base_param_parser*>& pair) {
      pair.second->show(server_handle()->log_stream());
    });
} /* show_all() */

NS_END(params, fordyca);
