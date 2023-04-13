/**
 * \file crw_controller.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/patterns/fsm/base_fsm.hpp"

#include "fordyca/controller/foraging_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace fsm { namespace d0 { class crw_fsm; }}

NS_START(controller, reactive, d0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class crw_controller
 * \ingroup controller reactive d0
 *
 * \brief The most basic form of a foraging controller: roam around randomly
 * until you find a block, and then bring it back to the nest; repeat.
 */
class crw_controller : public rer::client<crw_controller>,
                       public foraging_controller {
 public:
  crw_controller(void) RCPPSW_COLD;
  ~crw_controller(void) override RCPPSW_COLD;

  /* foraging_controller overrides */
  void init(ticpp::Element& node) override RCPPSW_COLD;
  void control_step(void) override;
  void reset(void) override RCPPSW_COLD;
  std::type_index type_index(void) const override { return typeid(*this); }

  /* goal acquisition metrics */
  bool is_vectoring_to_goal(void) const override { return false; }
  RCPPSW_WRAP_DECL_OVERRIDE(exp_status, is_exploring_for_goal, const);
  RCPPSW_WRAP_DECL_OVERRIDE(bool, goal_acquired, const);
  RCPPSW_WRAP_DECL_OVERRIDE(csmetrics::goal_acq_metrics::goal_type,
                            acquisition_goal,
                            const);
  RCPPSW_WRAP_DECL_OVERRIDE(boost::optional<rmath::vector3z>, acquisition_loc3D, const);
  RCPPSW_WRAP_DECL_OVERRIDE(boost::optional<rmath::vector3z>, explore_loc3D, const);
  RCPPSW_WRAP_DECL_OVERRIDE(boost::optional<rmath::vector3z>, vector_loc3D, const);
  RCPPSW_WRAP_DECL_OVERRIDE(rtypes::type_uuid, entity_acquired_id, const);

  /* block transportation */
  RCPPSW_WRAP_DECL_OVERRIDE(fsm::foraging_transport_goal,
                            block_transport_goal,
                            const);
  bool is_phototaxiing_to_goal(bool include_ca) const override RCPPSW_PURE;

  /* block carrying controller */
  const cssblocks::drop::base_drop* block_drop_strategy(void) const override;

  const fsm::d0::crw_fsm* fsm(void) const { return m_fsm.get(); }
  fsm::d0::crw_fsm* fsm(void) { return m_fsm.get(); }

 private:
  /* clang-format off */
  std::unique_ptr<fsm::d0::crw_fsm> m_fsm{nullptr};
  /* clang-format on */
};

NS_END(d0, reactive, controller, fordyca);
