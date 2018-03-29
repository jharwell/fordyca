/**
 * @file perceived_block.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_BLOCK_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_BLOCK_HPP_

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

class block;

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * @struct perceived_block
 * @ingroup representation
 *
 * @brief A representation of a "virtual" block in the arena, which has a
 * pheromone density/relevance associated with it.
 */
struct perceived_block {
  perceived_block(void) : ent(nullptr), density() {}
  perceived_block(const std::shared_ptr<block>& b,
                  const rcppsw::swarm::pheromone_density& d)
      : ent(b), density(d) {}

  std::shared_ptr<block> ent;
  rcppsw::swarm::pheromone_density density;
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRSENTATION_PERCEIVED_BLOCK_HPP_ */
