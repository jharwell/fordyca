/**
 * @file cache_penalty_generator.hpp
 *
 * @copyright 2018 John Harwell/Anthony Chen, All rights reserved.
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

 #ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_CACHE_PENALTY_GENERATOR_HPP_
 #define INCLUDE_FORDYCA_SUPPORT_DEPTH1_CACHE_PENALTY_GENERATOR_HPP_

 /*******************************************************************************
  * Classes
  ******************************************************************************/
 /**
  * @class cache_penalty_generator
  * @ingroup support depth1
  *
  * @brief Generates the temporal function that creates the penalty
  *
  * @param pen_func String that represents a temporal function
  *
  *
  */

class cache_penalty_generator{
  public:
    cache_penalty_generator(const char* pen_func, const int amp, const int per,
                            const int phase, const int square,
                            const int step, const int saw){
      switch(pen_func){
        case "sine":
            penalty_func = &sine_func;
            break;
        case "square":
            penalty_func = &square_func;
            break;
        case "step":
            penalty_func = &step_func;
            break;
        case "saw":
            penalty_func = &sawtooth_func;
            break;
      }
      amplitude = amp;
      period = per;
      phase_shift = phase;
      square_length = square;
      step_length = step;
      saw_length = saw;
    }

    cache_penalty_generator(){
      amplitude = 4;
      period = 1
      phase_shift = 1;
      square_length = 10;
      step_length = 20;
      saw_length = 10;
    }

    /**
     * @brief sine temporal penalty function
     *
     * @param timestep The current timestep.
     */
    static uint sine_func(uint timestep) {
      return (uint) (amplitude *(sin(timestep) + phase_shift));
    }
    /**
     * @brief square temporal penalty function
     *
     * @param timestep The current timestep.
     */
    static uint square_func(uint timestep) {
      uint time_ones = timestep % square_length;
      if(time_ones >= 0 && time_ones < (square_length/2)) {
        return 0;
      } else if(time_ones >= (square_length/2) && time_ones < square_length) {
        return 1;
      }
     }
    /**
     * @brief step temporal penalty function
     *
     * @param timestep The current timestep.
     */
     static uint step_func(uint timestep) {
      return (timestep/step_length);
     }
     /**
      * @brief sawtooth temporal penalty function
      *
      * @param timestep The current timestep.
      */
    static uint sawtooth_func(uint timestep){
      return (timestep % saw_length);
    }
    /**
     * @brief the function pointer for temporal function
     *
     */
    uint (cache_penalty_generator::*penalty_func)(uint);

  private:
    int amplitude;
    int period;
    int phase_shift;
    int square_length;
    int step_length;
    int saw_length;
