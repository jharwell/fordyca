/**
 * @file depth0/robot_configurer.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH0_ROBOT_CONFIGURER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH0_ROBOT_CONFIGURER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include <boost/variant/static_visitor.hpp>
#include "fordyca/params/visualization_params.hpp"
#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/controller/oracular_info_receptor.hpp"
#include "fordyca/support/oracle/entities_oracle.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, depth0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @struct robot_configurer
 * @ingroup fordyca support depth0
 *
 * @brief Functor to perform controller configuration during initialization.
 */
template<typename T>
class robot_configurer : public boost::static_visitor<void> {
 public:
  using controller_type = T;
  robot_configurer(const params::visualization_params* const params,
                   oracle::entities_oracle* const oracle)
      : mc_params(params),
        m_oracle(oracle) {}

  template<typename U = T,
           RCPPSW_SFINAE_TYPELIST_REJECT(controller::depth0::oracular_typelist,
                                         U)>
  void operator()(controller_type* const c) const {
    if (nullptr != mc_params) {
      c->display_los(mc_params->robot_los);
      c->display_id(mc_params->robot_id);
    }
  }
  template<typename U = T,
           RCPPSW_SFINAE_TYPELIST_REQUIRE(controller::depth0::oracular_typelist,
                                          U)>
  void operator()(controller_type* const c) const {
    if (nullptr != mc_params) {
      c->display_los(mc_params->robot_los);
      c->display_id(mc_params->robot_id);
    }
    if (nullptr != m_oracle) {
      auto receptor = rcppsw::make_unique<controller::oracular_info_receptor>(
          nullptr, m_oracle);
      c->oracle_init(std::move(receptor));
    }
  }

 private:
  /* clang-format off */
  const params::visualization_params * const mc_params;
  oracle::entities_oracle *                  m_oracle;
  /* clang-format on */
};

NS_END(depth0, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH0_ROBOT_CONFIGURER_HPP_ */
