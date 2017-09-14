/**
 * @file cache.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_CACHE_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_CACHE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <utility>

#include "fordyca/representation/cell_entity.hpp"
#include "fordyca/representation/block.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief A representation of a cache within the arena map. Caches do not have
 * state, and if/when a cache becomes empty, it needs to be deleted by an
 * enclosing class. Caches have both real (where they actually live in the
 * world) and discretized locations (where they are mapped to within the arena
 * map).
 */
class cache : public cell_entity,
              public rcppsw::patterns::visitor::visitable<cache> {
 public:
  explicit cache(double dimension, std::pair<block*, block*> blocks) :
      cell_entity(dimension, dimension, argos::CColor::BLUE), m_blocks() {
    m_blocks.push_back(blocks.first);
    m_blocks.push_back(blocks.second);
  }

  void block_add(block* block) { m_blocks.push_back(block); }
  void block_remove(block* block) { m_blocks.remove(block); }
  size_t n_blocks(void) const { return m_blocks.size(); }

 private:
  std::list<block*> m_blocks;
};

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
typedef std::pair<const cache*, double> perceived_cache;

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRSENTATION_CACHE_HPP_ */
