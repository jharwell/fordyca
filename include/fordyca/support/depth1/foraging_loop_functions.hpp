/**
 * @file foraging_loop_functions.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_FORAGING_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH1_FORAGING_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include "fordyca/support/depth0/stateful_foraging_loop_functions.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace metrics { namespace collectors {
class task_collector;
namespace robot_metrics {
class depth1_metrics_collector;
}}}

NS_START(support, depth1);

class cache_usage_penalty;
namespace robot_collectors = metrics::collectors::robot_metrics;

/*******************************************************************************
 * Classes
 ******************************************************************************/
class foraging_loop_functions : public depth0::stateful_foraging_loop_functions {
 public:
  foraging_loop_functions(void);
  virtual ~foraging_loop_functions(void);

  void Init(argos::TConfigurationNode& node) override;
  void PreStep() override;
  void Destroy(void) override;
  void Reset(void) override;

 protected:
  template<typename T>
  bool handle_cache_block_drop(argos::CFootBotEntity& robot);
  template<typename T>
  bool init_cache_usage_penalty(argos::CFootBotEntity& robot);
  template<typename T>
  bool cache_usage_penalty_satisfied(argos::CFootBotEntity& robot);
  template<typename T>
  void finish_cached_block_pickup(argos::CFootBotEntity& robot);
  int robot_on_cache(const argos::CFootBotEntity& robot);
  template<typename T>
  bool robot_serving_cache_penalty(argos::CFootBotEntity& robot);

 private:
  void pre_step_final(void) override;
  void pre_step_iter(argos::CFootBotEntity& robot);
  argos::CColor GetFloorColor(const argos::CVector2& plane_pos) override;

  foraging_loop_functions(const foraging_loop_functions& s) = delete;
  foraging_loop_functions& operator=(const foraging_loop_functions& s) = delete;

  uint mc_cache_penalty;
  double mc_cache_respawn_scale_factor;
  std::unique_ptr<robot_collectors::depth1_metrics_collector> m_depth1_collector;
  std::unique_ptr<metrics::collectors::task_collector> m_task_collector;
  std::list<cache_usage_penalty*> m_penalty_list;
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_FORAGING_LOOP_FUNCTIONS_HPP_ */
