/**
 * \file tracked_objects_accessor.hpp
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

#ifndef INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_TRACKED_OBJECTS_ACCESSOR_HPP_
#define INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_TRACKED_OBJECTS_ACCESSOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, subsystem, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class tracked_objects_accessor
 * \ingroup subsystem perception
 *
 * \brief Defines the interface for extracting information about currently
 * access_tracked objects, including tracking information about each object.
 */
template <typename TBlockContainer, typename TCacheContainer>
class tracked_objects_accessor {
 public:
  using cache_container_type = TCacheContainer;
  using block_container_type = TBlockContainer;

  using tracked_cache_type = typename cache_container_type::value_type;
  using tracked_block_type = typename block_container_type::value_type;

  tracked_objects_accessor(void) = default;
  virtual ~tracked_objects_accessor(void) = default;

  /**
   * \brief Get all tracked blocks the robot is currently aware of.
   */
  virtual const block_container_type& tracked_blocks(void) const = 0;
  virtual block_container_type& tracked_blocks(void) = 0;

  /**
   * \brief Get all tracked caches the robot is currently aware of.
   */
  virtual const cache_container_type& tracked_caches(void) const = 0;
  virtual cache_container_type& tracked_caches(void) = 0;
};

NS_END(perception, subsystem, fordyca);

#endif /* INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_TRACKED_OBJECTS_ACCESSOR_HPP_ */
