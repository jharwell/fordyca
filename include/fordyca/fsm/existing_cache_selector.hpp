/**
 * \file existing_cache_selector.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/types/timestep.hpp"

#include "fordyca/subsystem/perception/perception_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::repr {
class base_cache;
} /* namespace cosm::arena::repr */

NS_START(fordyca);

namespace controller::cognitive {
class cache_sel_matrix;
} // namespace controller::cognitive

NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class existing_cache_selector
 * \ingroup fsm
 *
 * \brief Selects from among known caches (which are presumed to still exist at
 * this point, although that may not be true as a robot's knowledge of the arena
 * is imperfect), using an internal utility function.
 */
class existing_cache_selector : public rer::client<existing_cache_selector> {
 public:
  existing_cache_selector(bool is_pickup,
                          const controller::cognitive::cache_sel_matrix* matrix,
                          const fspds::dp_cache_map* cache_map);

  ~existing_cache_selector(void) override = default;
  existing_cache_selector&
  operator=(const existing_cache_selector& other) = delete;
  existing_cache_selector(const existing_cache_selector&) = delete;

  /**
   * \brief Given a list of existing caches that a robot knows about (i.e. have
   * not faded into an unknown state), compute which is the "best", for use in
   * deciding which cache to go to and attempt to pickup from.
   *
   * \return The "best" existing cache.
   */
  const carepr::base_cache* operator()(const fspds::dp_cache_map& existing_caches,
                                       const rmath::vector2d& position,
                                       const rtypes::timestep& t);

 private:
  /**
   * \brief Determine if the specified cache is excluded from being considered
   * for selection because:
   *
   * - The robot is currently inside it.
   * - It is on the exception list.
   *
   * \return \c TRUE if the cache should be excluded, \c FALSE otherwise.
   */
  bool cache_is_excluded(const rmath::vector2d& position,
                         const carepr::base_cache* cache) const;

  /* clang-format off */
  const bool                                           mc_is_pickup;
  const controller::cognitive::cache_sel_matrix* const mc_matrix;
  const fspds::dp_cache_map* const                     mc_cache_map;
  /* clang-format on */
};

NS_END(fsm, fordyca);
