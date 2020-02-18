/**
 * \file entities_oracle.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/oracle/entities_oracle.hpp"

#include <numeric>

#include "cosm/foraging/repr/base_cache.hpp"
#include "cosm/repr/base_block2D.hpp"

#include "fordyca/config/oracle/entities_oracle_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, oracle);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
entities_oracle::entities_oracle(
    const config::oracle::entities_oracle_config* const config)
    : ER_CLIENT_INIT("fordyca.support.entities_oracle"),
      mc_blocks(config->blocks),
      mc_caches(config->caches) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
boost::optional<entities_oracle::variant_vector_type> entities_oracle::ask(
    const std::string& query) const {
  if (std::string::npos != query.find("entities.blocks")) {
    return m_blocks;
  } else if (std::string::npos != query.find("entities.caches")) {
    return m_caches;
  }
  return boost::optional<variant_vector_type>();
} /* ask() */

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
std::string entities_oracle::result_to_string(const variant_vector_type& v) {
  auto lambda = [&](const std::string& a, const auto& ent) {
    if (0 == ent.which()) {
      return a + "b" +
             rcppsw::to_string(
                 boost::get<std::shared_ptr<crepr::base_block2D>>(ent)->id()) +
             ",";
    } else if (1 == ent.which()) {
      return a + "c" +
             rcppsw::to_string(
                 boost::get<std::shared_ptr<cfrepr::base_cache>>(ent)->id()) +
             ",";
    } else {
      return a + "unknown entity type,";
    }
  };

  return std::accumulate(v.begin(), v.end(), std::string(), lambda);
} /* result_to_string() */

NS_END(oracle, support, fordyca);
