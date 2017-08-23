/**
 * @file line_of_sight.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_LINE_OF_SIGHT_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_LINE_OF_SIGHT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/multi_array.hpp>
#include <list>
#include "fordyca/representation/grid2D.hpp"
#include "fordyca/representation/cell2D.hpp"
#include "fordyca/representation/block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);


/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class line_of_sight {
  typedef std::pair<size_t, size_t> discrete_coord;
 public:
  explicit line_of_sight(const grid_view<cell2D*> view, const discrete_coord& center) :
      m_center(center), m_view(view) {}
  std::list<const representation::block*> blocks(void);
  size_t size(void) const { return m_view.num_elements(); }
  cell2D& cell(size_t i, size_t j) { return *m_view[i][j]; }
  discrete_coord& center(void) const { return m_center; }

 private:
  const discrete_coord& m_center;
  grid_view<cell2D*> m_view;
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_LINE_OF_SIGHT_HPP_ */
