/**
 * @file perceived_grid2d.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_GRID2D_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_GRID2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/multi_array.hpp>
#include <list>
#include <argos/core/utility/math/vector2.h>
#include "rcppsw/common/common.hpp"
#include "fordyca/representation/dynamic_grid2D.hpp"
#include "fordyca/representation/perceived_cell2D.hpp"
#include "fordyca/params/params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class perceived_grid2D : public dynamic_grid2D {
 public:
  explicit perceived_grid2D(const perceived_grid_params* params);

  std::list<const dynamic_cell2D*> with_blocks(void);

 private:
  std::shared_ptr<const perceived_grid_params> mc_params;
  boost::multi_array<perceived_cell2D, 2> m_cells;
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_GRID2D_HPP_ */
