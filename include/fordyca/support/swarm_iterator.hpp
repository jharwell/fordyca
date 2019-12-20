/**
 * \file swarm_iterator.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_SWARM_ITERATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_SWARM_ITERATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <vector>

#include "rcppsw/common/common.hpp"

#include "fordyca/controller/base_controller.hpp"
#include "fordyca/support/base_loop_functions.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * \struct swarm_iterator
 * \ingroup support
 *
 * \brief Iterate over the swarm (robots or controllers), and perform an action
 * within the ARGoS simulator.
 *
 * The selected action can be specified to be performed in static order
 * (single-threaded execution), or in dynamic order (multi-threaded execution)
 * for speed.
 */
struct swarm_iterator {
  using static_order = std::true_type;
  using dynamic_order = std::false_type;

  /**
   * \brief Iterate through controllers using static ordering.
   *
   * \tparam TRobotType The type of the robot within the ::argos namespace of
   *                    the robots in the swarm.
   * \tparam TOrdering \ref static_order or \ref dynamic_order.
   * \tparam TFunction Type of the lambda callback to use (inferred).
   *
   * \param sm Handle to the \ref cpal::swarm_manager.
   * \param cb Function to run on each robot in the swarm.
   * \param entity_name Name associated with the robot type within ARGoS.
   */
  template <typename TRobotType,
            typename TOrdering,
            typename TFunction,
            RCPPSW_SFINAE_FUNC(std::is_same<typename TOrdering::type,
                                            static_order::type>::value)>
  static void controllers(const cpal::swarm_manager* const sm,
                          const TFunction& cb,
                          const std::string& entity_name) {
    for (auto& [name, robotp] : sm->GetSpace().GetEntitiesByType(entity_name)) {
      auto* robot = ::argos::any_cast<TRobotType*>(robotp);
      auto* controller = dynamic_cast<controller::base_controller*>(
          &robot->GetControllableEntity().GetController());
      cb(controller);
    } /* for(...) */
  }

  /**
   * \brief Iterate through robots using dynamic ordering (OpenMP
   * implementation).
   *
   * \tparam TRobotType The type of the robot within the ::argos namespace of
   *                    the robots in the swarm.
   * \tparam TOrdering \ref static_order or \ref dynamic_order.
   * \tparam TFunction Type of the lambda callback (inferred).
   *
   * \param sm Handle to the \ref cpal::swarm_manager.
   * \param cb Function to run on each robot in the swarm.
   * \param entity_name Name associated with the robot type within ARGoS.
   */
  template <typename TRobotType,
            typename TOrdering,
            typename TFunction,
            RCPPSW_SFINAE_FUNC(std::is_same<typename TOrdering::type,
                                            dynamic_order::type>::value)>
  static void controllers(const cpal::swarm_manager* const sm,
                          const TFunction& cb,
                          const std::string& entity_name) {
    auto& ents = sm->GetSpace().GetEntitiesByType(entity_name);
    std::vector<::argos::CAny> robots;
    robots.reserve(ents.size());
    for (auto& [name, robotp] : ents) {
      robots.push_back(robotp);
    } /* for(&e..) */

#pragma omp parallel for
    for (size_t i = 0; i < robots.size(); ++i) {
      auto* robot = ::argos::any_cast<TRobotType*>(robots[i]);
      auto* controller = dynamic_cast<controller::base_controller*>(
          &robot->GetControllableEntity().GetController());
      cb(controller);
    } /* for(i..) */
  }

  /**
   * \brief Iterate through robots using static ordering.
   *
   * \tparam TRobotType The type of the robot within the ::argos namespace of
   *                    the robots in the swarm.
   * \tparam TOrdering \ref static_order or \ref dynamic_order.
   * \tparam TFunction Type of the lambda callback (inferred).
   *
   * \param sm Handle to the \ref cpal::swarm_manager.
   * \param cb Function to run on each robot in the swarm.
   * \param entity_name Name associated with the robot type within ARGoS.
   */
  template <typename TRobotType,
            typename TOrdering,
            typename TFunction,
            RCPPSW_SFINAE_FUNC(std::is_same<typename TOrdering::type,
                                            static_order::type>::value)>
  static void robots(const cpal::swarm_manager* const sm,
                     const TFunction& cb,
                     const std::string& entity_name) {
    for (auto& [name, robotp] : sm->GetSpace().GetEntitiesByType(entity_name)) {
      auto* robot = ::argos::any_cast<TRobotType*>(robotp);
      cb(robot);
    } /* for(...) */
  }

  /**
   * \brief Iterate through robots using dynamic ordering (OpenMP
   * implementation).
   *
   * \tparam TRobotType The type of the robot within the ::argos namespace of
   *                    the robots in the swarm.
   * \tparam TOrdering \ref static_order or \ref dynamic_order.
   * \tparam TFunction Type of the lambda callback (inferred).
   *
   * \param sm Handle to the \ref cpal::swarm_manager.
   * \param cb Function to run on each robot in the swarm.
   * \param entity_name Name associated with the robot type within ARGoS.
   */
  template <typename TRobotType,
            typename TOrdering,
            typename TFunction,
            RCPPSW_SFINAE_FUNC(std::is_same<typename TOrdering::type,
                                            dynamic_order::type>::value)>
  static void robots(const cpal::swarm_manager* const sm,
                     const TFunction& cb,
                     const std::string& entity_name) {
    auto& ents = sm->GetSpace().GetEntitiesByType(entity_name);
    std::vector<::argos::CAny> robots;
    robots.reserve(ents.size());
    for (auto& [name, robotp] : ents) {
      robots.push_back(robotp);
    } /* for(&e..) */

#pragma omp parallel for
    for (size_t i = 0; i < robots.size(); ++i) {
      auto* robot = ::argos::any_cast<TRobotType*>(robots[i]);
      cb(robot);
    } /* for(i..) */
  }
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_SWARM_ITERATOR_HPP_ */
