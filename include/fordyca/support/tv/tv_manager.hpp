/**
 * @file tv_manager.hpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_TV_TV_MANAGER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_TV_TV_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <tuple>
#include <typeindex>
#include <list>
#include <map>

#include <boost/variant.hpp>
#include <boost/container/map.hpp>

#include "fordyca/metrics/temporal_variance_metrics.hpp"
#include "fordyca/support/tv/block_op_penalty_handler.hpp"
#include "fordyca/support/tv/cache_op_penalty_handler.hpp"
#include "fordyca/support/tv/motion_throttling_handler.hpp"
#include "fordyca/support/tv/block_op_src.hpp"
#include "fordyca/support/tv/cache_op_src.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Macros
 ******************************************************************************/
#define BLOCK_HANDLERS                                                  \
  std::unique_ptr<block_op_penalty_handler<controller::depth0::crw_controller>> , \
    std::unique_ptr<block_op_penalty_handler<controller::depth0::dpo_controller>>, \
    std::unique_ptr<block_op_penalty_handler<controller::depth0::mdpo_controller>>, \
    std::unique_ptr<block_op_penalty_handler<controller::depth1::gp_dpo_controller>>, \
    std::unique_ptr<block_op_penalty_handler<controller::depth1::gp_mdpo_controller>>,\
    std::unique_ptr<block_op_penalty_handler<controller::depth2::grp_dpo_controller>>, \
    std::unique_ptr<block_op_penalty_handler<controller::depth2::grp_mdpo_controller>>

#define EXISTING_CACHE_HANDLERS                                                  \
  std::unique_ptr<cache_op_penalty_handler<controller::depth1::gp_dpo_controller>>, \
    std::unique_ptr<cache_op_penalty_handler<controller::depth1::gp_mdpo_controller>>,\
    std::unique_ptr<cache_op_penalty_handler<controller::depth2::grp_dpo_controller>>, \
    std::unique_ptr<cache_op_penalty_handler<controller::depth2::grp_mdpo_controller>>

#define FREE_BLOCK_DROP_HANDLERS                                                  \
  std::unique_ptr<block_op_penalty_handler<controller::depth2::grp_dpo_controller>>, \
    std::unique_ptr<block_op_penalty_handler<controller::depth2::grp_mdpo_controller>>

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca);

namespace controller {
namespace depth0 {
class crw_controller;
class dpo_controller;
class mdpo_controller;
}
namespace depth1 {
class gp_dpo_controller;
class gp_mdpo_controller;
}
namespace depth2 {
class grp_dpo_controller;
class grp_mdpo_controller;
}
} /* namespace controller */
namespace params { namespace tv { struct tv_manager_params; }}
namespace ds { class arena_map; }
NS_START(support);

class base_loop_functions;

NS_START(tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class tv_manager
 * @ingroup fordyca support tv
 * @brief Orchestrates all application of temporal variance to robot interations
 * with the environment/robotic mechanical functioning.
 */

class tv_manager : public rcppsw::er::client<tv_manager>,
                   public metrics::temporal_variance_metrics {
 public:
  template<typename T>
  using filter_status = typename block_op_filter<T>::filter_status;

  template<typename T>
  using penalty_handler_list = std::list<tv::temporal_penalty_handler<T>*>;

  tv_manager(const params::tv::tv_manager_params* params,
                const support::base_loop_functions* lf,
                ds::arena_map* map);

  tv_manager(const tv_manager& other) = delete;
  const tv_manager& operator=(const tv_manager& other) = delete;

  /* temporal variance metrics */
  double swarm_motion_throttle(void) const override;
  double env_block_manipulation(void) const override;
  double env_cache_usage(void) const override;

  /**
   * @brief Return non-owning reference to a penalty handler for the specified
   * type of block operation; scope of usage must not exceed that of the
   * instance of this class used to generate the reference.
   */
  template<class T>
  block_op_penalty_handler<T>* penalty_handler(
      const block_op_src& src) const {
    switch (src) {
      case block_op_src::kSrcFreePickup:
        return boost::get<std::unique_ptr<block_op_penalty_handler<T>>>(
            m_fb_pickup.at(typeid(T))).get();
      case block_op_src::kSrcNestDrop:
        return boost::get<std::unique_ptr<block_op_penalty_handler<T>>>(
            m_nest_drop.at(typeid(T))).get();
      case block_op_src::kSrcNewCacheDrop:
        return boost::get<std::unique_ptr<block_op_penalty_handler<T>>>(
            m_new_cache.at(typeid(T))).get();
      case block_op_src::kSrcCacheSiteDrop:
        return boost::get<std::unique_ptr<block_op_penalty_handler<T>>>(
            m_cache_site.at(typeid(T))).get();
      default:
        ER_FATAL_SENTINEL("Bad penalty source %d", src);
    } /* switch() */
    return nullptr;
  }

  /**
   * @brief Return non-owning reference to a penalty handler for the specified
   * type of cache operation; scope of usage must not exceed that of the
   * instance of this class used to generate the reference.
   */
  template<class T>
  cache_op_penalty_handler<T>* penalty_handler(const cache_op_src& src) const {
    switch (src) {
      case cache_op_src::kSrcExistingCachePickup:
      case cache_op_src::kSrcExistingCacheDrop:
        return boost::get<std::unique_ptr<cache_op_penalty_handler<T>>>(
            m_existing_cache.at(typeid(T))).get();
        break;
      default:
        ER_FATAL_SENTINEL("Bad penalty source %d", src);
    } /* switch() */
    return nullptr;
  }

  /*
   * Not all handlers are valid/needed for all controllers, and if you don't use
   * SFINAE to control what templates are instantiated for a given controller,
   * you get a bunch of errors from boost::variant at runtime because you are
   * trying to get a handler for cache pickup/drop for a \ref crw_controller
   * (for example).
   */
  template<typename T,
           RCPPSW_SFINAE_REQUIRE(std::is_same<T,
                                 controller::depth0::crw_controller>::value ||
                                 std::is_same<T,
                                 controller::depth0::dpo_controller>::value ||
                                 std::is_same<T,
                                 controller::depth0::mdpo_controller>::value)
           >
  penalty_handler_list<T> all_penalty_handlers(void) {
    return {penalty_handler<T>(block_op_src::kSrcFreePickup),
          penalty_handler<T>(block_op_src::kSrcNestDrop)};
  }
  template<typename T,
           RCPPSW_SFINAE_REQUIRE(std::is_same<T,
                                 controller::depth1::gp_dpo_controller>::value ||
                                 std::is_same<T,
                                 controller::depth1::gp_mdpo_controller>::value)
           >
  penalty_handler_list<T> all_penalty_handlers(void) {
    return {penalty_handler<T>(block_op_src::kSrcFreePickup),
          penalty_handler<T>(block_op_src::kSrcNestDrop),
          penalty_handler<T>(cache_op_src::kSrcExistingCacheDrop)};
  }

  template<typename T,
           RCPPSW_SFINAE_REQUIRE(std::is_same<T,
                                 controller::depth2::grp_dpo_controller>::value ||
                                 std::is_same<T,
                                 controller::depth2::grp_mdpo_controller>::value)
           >
  penalty_handler_list<T> all_penalty_handlers(void) {
    return {penalty_handler<T>(block_op_src::kSrcFreePickup),
          penalty_handler<T>(block_op_src::kSrcNestDrop),
          penalty_handler<T>(cache_op_src::kSrcExistingCacheDrop),
          penalty_handler<T>(block_op_src::kSrcCacheSiteDrop),
          penalty_handler<T>(block_op_src::kSrcNewCacheDrop),
          };
  }

  const motion_throttling_handler* movement_throttling_handler(int robot_id) const {
    return &m_motion_throttling.at(robot_id);
  }

  /**
   * @brief Register a robot controller to the temporal variance controller so
   * that all necessary handlers for all possible types of variance that could
   * be applied to a given controller are setup.
   *
   * @param robot_id The ID of the robot controller, assumed to be unique (not
   * checked).
   */
  void register_controller(int robot_id);

  /**
   * @brief Update the state of applied variance:
   *
   * - Status of all motion throttling handlers, conditioned on whether or not a
   *   given robot is currently carrying a block.
   *
   * Should be called once per timestep.
   */
  void update(void);

 private:
  void nest_drop_init(const params::tv::tv_manager_params* params,
                      ds::arena_map* map);
  void fb_pickup_init(const params::tv::tv_manager_params* params,
                      ds::arena_map* map);
  void existing_cache_init(const params::tv::tv_manager_params* params,
                           ds::arena_map* map);
  void cache_site_init(const params::tv::tv_manager_params* params,
                       ds::arena_map* map);

  using block_variant = boost::variant<BLOCK_HANDLERS>;
  using existing_cache_variant = boost::variant<EXISTING_CACHE_HANDLERS>;
  using fb_drop_variant = boost::variant<FREE_BLOCK_DROP_HANDLERS>;

  /* clang-format off */
  const support::base_loop_functions* const mc_lf;
  const rct::waveform_params                mc_motion_throttle_params;

  boost::container::map<std::type_index, block_variant>          m_fb_pickup{};
  boost::container::map<std::type_index, block_variant>          m_nest_drop{};
  boost::container::map<std::type_index, existing_cache_variant> m_existing_cache{};
  boost::container::map<std::type_index, fb_drop_variant>        m_new_cache{};
  boost::container::map<std::type_index, fb_drop_variant>        m_cache_site{};

  std::map<int, motion_throttling_handler>                       m_motion_throttling{};
  /* clang-format on */
};

NS_END(tv, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_TV_TV_MANAGER_HPP_ */
