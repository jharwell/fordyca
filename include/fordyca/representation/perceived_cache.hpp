/**
 * @file perceived_cache.hpp
 * @ingroup representation
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_CACHE_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_CACHE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>
#include "rcppsw/common/common.hpp"
#include "rcppsw/swarm/pheromone_density.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);
class base_cache;

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * @struct perceived_cache
 * @ingroup representation
 *
 * @brief A representation of a "virtual" cache in the arena, which has a
 * pheromone density/relevance associated with it.
 */
struct perceived_cache {
  perceived_cache(void) : ent(nullptr), density() {}
  perceived_cache(const std::shared_ptr<base_cache>& c,
                  const rcppsw::swarm::pheromone_density& d)
      : ent(c), density(d) {}

  std::shared_ptr<base_cache> ent;
  rcppsw::swarm::pheromone_density density;
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRSENTATION_PERCEIVED_CACHE_HPP_ */
