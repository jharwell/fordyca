/**
 * \file known_objects_accessor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>

#include "rcppsw/math/vector2.hpp"

#include "cosm/ds/block3D_vector.hpp"
#include "cosm/arena/ds/cache_vector.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, subsystem, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class known_objects_accessor
 * \ingroup subsystem perception
 *
 * \brief Defines the interface for extracting information about currently
 * known objects, without tracking information (i.e., "raw" tracked objects).
 */
class known_objects_accessor {
 public:
  known_objects_accessor(void) = default;
  virtual ~known_objects_accessor(void) = default;

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

  boost::optional<rmath::vector2d> last_block_loc(void) const {
    return m_last_block_loc;
  }
  boost::optional<rmath::vector2d> last_cache_loc(void) const {
    return m_last_cache_loc;
  }

 protected:
  void last_block_loc(const rmath::vector2d& loc) { m_last_block_loc = loc; }
  void last_cache_loc(const rmath::vector2d& loc) { m_last_cache_loc = loc; }

 private:
  /* clang-format off */
  boost::optional<rmath::vector2d> m_last_block_loc{};
  boost::optional<rmath::vector2d> m_last_cache_loc{};
  /* clang-format on */
};

NS_END(perception, subsystem, fordyca);

