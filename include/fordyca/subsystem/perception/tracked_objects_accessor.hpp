/**
 * \file tracked_objects_accessor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

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

