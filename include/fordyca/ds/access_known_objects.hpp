/**
 * \file access_known_objects.hpp
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

#ifndef INCLUDE_FORDYCA_DS_ACCESS_KNOWN_OBJECTS_HPP_
#define INCLUDE_FORDYCA_DS_ACCESS_KNOWN_OBJECTS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ds/block3D_vector.hpp"
#include "cosm/arena/ds/cache_vector.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, ds);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class access_known_objects
 * \ingroup ds
 *
 * \brief Defines the interface for extracting information about currently
 * known objects, without tracking information (i.e., "raw" tracked objects).
 */
class access_known_objects {
 public:
  access_known_objects(void) = default;
  virtual ~access_known_objects(void) = default;

  /**
   * \brief Get all known blocks the robot is currently aware of, sans tracking
   * information.
   */
  virtual cds::block3D_vectorno known_blocks(void) const = 0;

  /**
   * \brief Get all known caches the robot is currently aware of, sans tracking
   * information.
   */
  virtual cads::bcache_vectorno known_caches(void) const = 0;
};

NS_END(ds, fordyca);

#endif /* INCLUDE_FORDYCA_DS_ACCESS_KNOWN_OBJECTS_HPP_ */
