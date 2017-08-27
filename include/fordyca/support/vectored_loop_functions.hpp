/**
 * @file vectored_loop_functions.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_VECTORED_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_VECTORED_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <vector>
#include "fordyca/support/base_loop_functions.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
class vectored_loop_functions : public base_loop_functions {
 public:
  vectored_loop_functions() {}
  virtual ~vectored_loop_functions(void) {}

  virtual void Init(argos::TConfigurationNode& node);
  virtual void PreStep();

 private:
  vectored_loop_functions(const vectored_loop_functions& s) = delete;
  vectored_loop_functions& operator=(const vectored_loop_functions& s) = delete;
  void set_robot_los(argos::CFootBotEntity& robot);
  void set_robot_tick(argos::CFootBotEntity& robot);
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_VECTORED_LOOP_FUNCTIONS_HPP_ */
