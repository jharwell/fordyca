/**
 * \file d0_loop_functions.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_D0_D0_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_D0_D0_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/ds/type_map.hpp"
#include "rcppsw/ds/grid2D_overlay.hpp"

#include "cosm/controller/operations/grid_los_update.hpp"
#include "cosm/controller/operations/metrics_extract.hpp"
#include "cosm/hal/robot.hpp"

#include "fordyca/repr/forager_los.hpp"
#include "fordyca/support/base_loop_functions.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena {
class caching_arena_map;
} /* namespace cosm::foraging::ds */

NS_START(fordyca, support, d0);
namespace detail {
struct functor_maps_initializer;
} /* namespace detail */
class d0_metrics_manager;

template<typename Controller, typename TArenaMap>
class robot_arena_interactor;

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class d0_loop_functions
 * \ingroup support d0
 *
 * \brief Contains the simulation support functions for d0 foraging, such
 * as:
 *
 * - Metric collection from robots
 * - Robot arena interactions
 */
class d0_loop_functions : public base_loop_functions,
                              public rer::client<d0_loop_functions> {
 public:
  d0_loop_functions(void) RCPPSW_COLD;
  ~d0_loop_functions(void) override RCPPSW_COLD;

  /* swarm manager overrides */
  void init(ticpp::Element& node) override RCPPSW_COLD;
  void pre_step(void) override;
  void post_step(void) override;
  void reset(void) override RCPPSW_COLD;
  void destroy(void) override RCPPSW_COLD;

 protected:
  /**
   * \brief Initialize d0 support to be shared with derived classes:
   *
   * - Depth0 metric collection
   */
  void shared_init(ticpp::Element& node) RCPPSW_COLD;

private:
  using interactor_map_type = rds::type_map<
    rmpl::typelist_wrap_apply<controller::d0::typelist,
                              robot_arena_interactor,
                              carena::caching_arena_map>::type
    >;
  using los_updater_map_type = rds::type_map<
    rmpl::typelist_wrap_apply<controller::d0::typelist,
                              ccops::grid_los_update,
                              rds::grid2D_overlay<cds::cell2D>,
                              repr::forager_los>::type>;

  using metric_extraction_map_type = rds::type_map<
    rmpl::typelist_wrap_apply<controller::d0::typelist,
                              ccops::metrics_extract,
                              d0_metrics_manager>::type>;
  /**
   * \brief These are friend classes because they are basically just pieces of
   * the loop functions pulled out for increased clarity/modularity, and are not
   * meant to be used in other contexts.
   *
   * Doing things this way rather than passing 8 parameters to the functors
   * seemed much cleaner.
   */
  friend detail::functor_maps_initializer;

  /**
   * \brief Initialize d0 support not shared with derived classes:
   *
   * - Robot interactions with arena
   * - Various maps mapping controller types to metric collection, controller
   *   initialization, and arena interaction maps (reflection basically).
   */
  void private_init(void) RCPPSW_COLD;

  /**
   * \brief Process a single robot on a timestep, before running its controller:
   *
   * - Set its new position, time from ARGoS and send it its LOS.
   *
   * \note These operations are done in parallel for all robots (lock free).
   */
  void robot_pre_step(chal::robot& robot);

  /**
   * \brief Process a single robot on a timestep, after running its controller.
   *
   * - Have it interact with the environment.
   * - Collect metrics from it.
   *
   * \note These operations are done in parallel for all robots (with mutual
   *       exclusion as needed).
   */
  void robot_post_step(chal::robot& robot);

  /* clang-format off */
  std::unique_ptr<d0_metrics_manager>      m_metrics_manager;
  std::unique_ptr<interactor_map_type>        m_interactor_map;
  std::unique_ptr<metric_extraction_map_type> m_metrics_map;
  std::unique_ptr<los_updater_map_type>       m_los_update_map;
  /* clang-format on */
};

NS_END(d0, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_D0_D0_LOOP_FUNCTIONS_HPP_ */
