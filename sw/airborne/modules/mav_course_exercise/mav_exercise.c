/*
 * Copyright (C) 2021 Matteo Barbera <matteo.barbera97@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "mav_exercise.h"
#include "modules/core/abi.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include "autopilot_static.h"
#include <stdio.h>
#include <time.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define PRINT(string, ...) fprintf(stderr, "[mav_exercise->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

uint8_t increase_nav_heading(float incrementDegrees);
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_SAFE_HEADING,
  OUT_OF_BOUNDS,
};

// define and initialise global variables
float oa_color_count_frac = 0.18f;
enum navigation_state_t navigation_state = SEARCH_SAFE_HEADING;
int32_t color_count = 0;               // orange color count from color filter for obstacle detection
float divergence_size = 0;             // divergence size from optical flow
float divergence_threshold = 0.003;      // threshold for the divergence value for optical flow object detection
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
float moveDistance = 0.6;                 // waypoint displacement [m]
float oob_heading_increment = 10.f;      // heading angle increment if out of bounds [deg]
const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free


// needed to receive output from a separate module running on a parallel process
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, 
                               int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width,
                               int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, 
                               int16_t __attribute__((unused)) extra) 
{
  color_count = quality;
}

// Receive ABI message from opticflow_module.c, where the divergence value is of interest
#ifndef OPTICAL_FLOW_ID
#define OPTICAL_FLOW_ID ABI_BROADCAST
#endif
static abi_event optical_flow_ev;
static void optical_flow_cb(uint8_t __attribute__((unused)) sender_id,
                            uint32_t __attribute__((unused)) stamp, 
                            int32_t __attribute__((unused)) flow_x,
                            int32_t __attribute__((unused)) flow_y,
                            int32_t __attribute__((unused)) flow_der_x,
                            int32_t __attribute__((unused)) flow_der_y,
                            float __attribute__((unused)) quality, 
                            float size_divergence) 
{
  divergence_size = size_divergence;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    ////// RUN AUTOPILOT INIT FUNCTION //////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mav_exercise_init(void) {
  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);

  // bind the optical flow callbacks to receive the divergence values
  AbiBindMsgOPTICAL_FLOW(OPTICAL_FLOW_ID, &optical_flow_ev, optical_flow_cb);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    ////// RUN AUTOPILOT PERIODIC FUNCTION //////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mav_exercise_periodic(void) {
  // only evaluate our state machine if we are flying
  if (!autopilot_in_flight()) {
    return;
  }

  ////// COMPUTE CURRENT COLOR THRESHOLDS //////
  int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h; // front_camera defined in airframe xml, with the video_capture module

  ////// PRINT DETECTION VALUES //////
  PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state); // Print visual detection pixel colour values and navigation state
  PRINT("Divergence size: %lf Divergence threshold: %lf \n", divergence_size, divergence_threshold); // Print optical flow divergence size


  ////// DETERMINE OBSTACLE FREE CONFIDENCE //////
  // Update our safe confidence using color threshold
  if (color_count < color_count_threshold) {
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  // Bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  ////// NAVIGATION STATE MACHINE //////
  switch (navigation_state) {
    case SAFE:
      moveWaypointForward(WP_TRAJECTORY, 1.9f * moveDistance);
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence == 0 && divergence_size > divergence_threshold) {
        navigation_state = OBSTACLE_FOUND;
      } else {
        moveWaypointForward(WP_GOAL, moveDistance);
      }
      break;
    case OBSTACLE_FOUND:
      // Stop as soon as obstacle is found
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);

      navigation_state = SEARCH_SAFE_HEADING;
      break;
    case SEARCH_SAFE_HEADING:
      increase_nav_heading(oob_heading_increment);
    
      if (obstacle_free_confidence >= 3){
        navigation_state = SAFE;
      }
      break; 
    case OUT_OF_BOUNDS:
      // stop
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);

      increase_nav_heading(oob_heading_increment);
      moveWaypointForward(WP_TRAJECTORY, 2.1f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        // add offset to head back into arena
        increase_nav_heading(oob_heading_increment);
        obstacle_free_confidence = 0;
        navigation_state = SEARCH_SAFE_HEADING;
      }
      break;
    default:
      break;
  }
  return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees) {
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading
  nav_heading = ANGLE_BFP_OF_REAL(new_heading);

  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters) {
  float heading = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor) {
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters) {
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}
