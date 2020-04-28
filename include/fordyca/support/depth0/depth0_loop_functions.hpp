/**
 * \file depth0_loop_functions.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH0_DEPTH0_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH0_DEPTH0_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include "rcppsw/ds/type_map.hpp"
#include "rcppsw/ds/grid2D_overlay.hpp"

#include "cosm/controller/operations/robot_los_update.hpp"
#include "cosm/controller/operations/metrics_extract.hpp"
#include "fordyca/repr/forager_los.hpp"

#include "fordyca/support/base_loop_functions.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena {
class caching_arena_map;
} /* namespace cosm::foraging::ds */

NS_START(fordyca, support, depth0);
namespace detail {
struct functor_maps_initializer;
} /* namespace detail */
class depth0_metrics_aggregator;

template<typename ControllerType, typename TArenaMapType>
class robot_arena_interactor;

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class depth0_loop_functions
 * \ingroup support depth0
 *
 * \brief Contains the simulation support functions for depth0 foraging, such
 * as:
 *
 * - Metric collection from robots
 * - Robot arena interactions
 */
class depth0_loop_functions : public base_loop_functions,
                              public rer::client<depth0_loop_functions> {
 public:
  depth0_loop_functions(void) RCSW_COLD;
  ~depth0_loop_functions(void) override RCSW_COLD;

  /* swarm manager overrides */
  void init(ticpp::Element& node) override RCSW_COLD;
  void pre_step(void) override;
  void post_step(void) override;
  void reset(void) override RCSW_COLD;
  void destroy(void) override RCSW_COLD;

 protected:
  /**
   * \brief Initialize depth0 support to be shared with derived classes:
   *
   * - Depth0 metric collection
   */
  void shared_init(ticpp::Element& node) RCSW_COLD;

 private:
  using interactor_map_type = rds::type_map<
    rmpl::typelist_wrap_apply<controller::depth0::typelist,
                              robot_arena_interactor,
                              carena::caching_arena_map>::type
    >;
  using los_updater_map_type = rds::type_map<
    rmpl::typelist_wrap_apply<controller::depth0::typelist,
                              ccops::robot_los_update,
                              rds::grid2D_overlay<cds::cell2D>,
                              repr::forager_los>::type>;

  using metric_extraction_typelist = rmpl::typelist<
    ccops::metrics_extract<controller::depth0::crw_controller,
                           depth0_metrics_aggregator>,
    ccops::metrics_extract<controller::depth0::dpo_controller,
                           depth0_metrics_aggregator>,
    ccops::metrics_extract<controller::depth0::odpo_controller,
                           depth0_metrics_aggregator>,
    ccops::metrics_extract<controller::depth0::mdpo_controller,
                           depth0_metrics_aggregator>,
    ccops::metrics_extract<controller::depth0::omdpo_controller,
                           depth0_metrics_aggregator>
    >;

  using metric_extraction_map_type = rds::type_map<metric_extraction_typelist>;

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
   * \brief Initialize depth0 support not shared with derived classes:
   *
   * - Robot interactions with arena
   * - Various maps mapping controller types to metric collection, controller
   *   initialization, and arena interaction maps (reflection basically).
   */
  void private_init(void) RCSW_COLD;

  /**
   * \brief Process a single robot on a timestep, before running its controller:
   *
   * - Set its new position, time from ARGoS and send it its LOS.
   *
   * \note These operations are done in parallel for all robots (lock free).
   */
  void robot_pre_step(argos::CFootBotEntity& robot);

  /**
   * \brief Process a single robot on a timestep, after running its controller.
   *
   * - Have it interact with the environment.
   * - Collect metrics from it.
   *
   * \note These operations are done in parallel for all robots (with mutual
   *       exclusion as needed).
   */
  void robot_post_step(argos::CFootBotEntity& robot);

  argos::CColor GetFloorColor(const argos::CVector2& plane_pos) override RCSW_PURE;

  /* clang-format off */
  std::unique_ptr<depth0_metrics_aggregator>  m_metrics_agg;
  std::unique_ptr<interactor_map_type>        m_interactor_map;
  std::unique_ptr<metric_extraction_map_type> m_metrics_map;
  std::unique_ptr<los_updater_map_type>       m_los_update_map;
  /* clang-format on */
};

NS_END(depth0, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH0_DEPTH0_LOOP_FUNCTIONS_HPP_ */
