#ifndef INCLUDE_FORDYCA_CONTROLLER_COMMUNICATION_SUBSYSTEM_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_COMMUNICATION_SUBSYSTEM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/config/communication_config.hpp"
#include "fordyca/controller/mdpo_perception_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/controller/block_sel_matrix.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
NS_START(controller);
/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class communication_subsystem
 * @ingroup fordyca controller
 *
 * @brief 
 */
class communication_subsystem
{
  public:
    const std::string kRANDOM = "random";
    const std::string kUTILITY = "utility";
    /**
    * @brief Initialize the base sensing subsystem.
    *
    * @param config Subsystem parameters.
    */
    communication_subsystem(controller::base_perception_subsystem * perception,
                            controller::saa_subsystem *saa,
                            controller::block_sel_matrix *block_sel_matrix);

    /**
    * @brief Checks for available messages from nearby robots, and
    * probabilistically integrates their contents. Also probabilistically
    * sends messages to nearby robots.
    *
    * Called from \ref ControlStep().
    */
    void communication_check(void);

    /**
    * @brief Calls get_most_valuable_cell, and fills a rab_wifi_packet with
    * information representing the current state of that cell.
    *
    * Called iteratively from \ref communication_check().
    */
    void fill_packet(void);

    /**
    * @brief Integrate a received communication packet's contents with the
    * robot's internal environmental mapping.
    *
    * Called from \ref communication_check().
    */
    void integrate_recieved_packet(std::vector<uint8_t> packet_data);

    /**
    * @brief Returns a vector containing the (X,Y) coordinates of the cell deamed
    * most valuable according to block count, distance from the nest, and current
    * pheromon levels.
    *
    * Called from \ref fill_packet().
    */
    rcppsw::math::vector2u get_most_valuable_cell(void);

    std::vector<std::vector<uint8_t>> validate_messages(std::vector<rrhal::sensors::rab_wifi_sensor::rab_wifi_packet> readings);

    void set_communication_parameters(const struct config::communication_config *params) {
      m_communication_params = *params;
    }

  private:
    mdpo_perception_subsystem *mdpo_perception(void) {
      return static_cast<mdpo_perception_subsystem *>(m_perception.get());
    }

    class saa_subsystem* saa_subsystem(void) {
      return m_saa.get();
    }

    class block_sel_matrix* block_sel_matrix(void) {
      return m_block_sel_matrix;
    }


    /* clang-format off */
    struct config::communication_config                    m_communication_params;
    std::shared_ptr<controller::base_perception_subsystem> m_perception;
    std::shared_ptr<controller::saa_subsystem>             m_saa;
    class block_sel_matrix*                                m_block_sel_matrix;
    /* clang-format on */
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_COMMUNICATION_SUBSYSTEM_HPP_ */