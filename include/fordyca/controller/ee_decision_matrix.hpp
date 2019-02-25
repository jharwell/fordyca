/**
 * @file ee_decision_matrix.hpp
 *
 * @copyright 2019 Anthony Chen/John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_EE_DECISION_MATRIX_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_EE_DECISION_MATRIX_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/params/energy_params.hpp"

#include <boost/variant.hpp>
#include <map>
#include <string>
#include <vector>

#include "rcppsw/common/common.hpp"
#include "rcppsw/math/vector2.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
NS_START(controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class ee_decision_matrix
 * @ingroup controller
 *
 * @brief Energy Efficieny Decision Matrix holds the threshold values for the
 * robot to use the appropriate amount of energy.
 *
 *
 * This class may be separated into those components in the future if it makes
 * sense. For now, it is cleaner to have all three uses be in the same class.
 */

class ee_decision_matrix {
  public:
    float e_lowerT;
    float e_higherT;
    bool liu;
    uint Th;

    explicit ee_decision_matrix(const struct params::energy_params* const params);

    void setData(float eLow, float eHigh);
    void setForageTime(uint newTh);


};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_EE_DECISION_MATRIX_HPP_ */
