/**
 * \file block_vector.hpp
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

#ifndef INCLUDE_FORDYCA_DS_BLOCK_VECTOR_HPP_
#define INCLUDE_FORDYCA_DS_BLOCK_VECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <vector>

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class base_block2D;
}
NS_START(fordyca, ds);

using block_vector_type = std::shared_ptr<crepr::base_block2D>;

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * \class block_vector
 * \ingroup ds
 *
 * \brief Specialization of \ref std::vector with an additional \ref to_str()
 * method.
 */
class block_vector : public std::vector<block_vector_type> {
 public:
  using std::vector<block_vector_type>::vector;
  using value_type = std::vector<block_vector_type>::value_type;

  /**
   * \brief Get a string representation of the vector contents.
   */
  std::string to_str(void) const;
};

NS_END(ds, fordyca);

#endif /* INCLUDE_FORDYCA_DS_BLOCK_VECTOR_HPP_ */
