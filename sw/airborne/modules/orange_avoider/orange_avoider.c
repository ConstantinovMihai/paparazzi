/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of Paparazzi
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

/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * @edit by Group 5 for the course (AE4317) Autonomous Flight of Micro Air Vehicles 
 * @year 2022/2023  
 *
 * This module is used in combination with a color filter (cv_detect_color_object.c), optic flow (opticflow_module.c), 
 * and the navigation mode of the autopilot.
 *
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn. The same principle is applied but then 
 * using the divergence difference value between left and right of image (div_diff) and the divergence threshold (divergence_threshold).
 *
 */

#include "modules/orange_avoider/orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    ////// DEFINE SETTINGS AND VARIABLES //////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////// BUILT-IN FUNCTIONS DEFINITION //////
static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
uint8_t chooseRandomIncrementAvoidance(void);

////// STATES AND VARIABLE DEFINITION AND INITIALISATION //////
enum navigation_state_t {
  SAFE,
  SEARCH_SAFE_HEADING,
  TURN_AROUND,
  OUT_OF_BOUNDS,
};

// Add sub state for search safe heading to make it clear which detection method called the state
enum search_safe_heading_state_t {
  ORANGE,
  GREEN,
};

enum navigation_state_t navigation_state = SEARCH_SAFE_HEADING;
enum search_safe_heading_state_t search_safe_heading_state = ORANGE;

// Orange avoider:
int32_t color_count = 0;                                 // orange color count from color filter for obstacle detection
float oa_color_count_frac = 0.18f;

// // Green detector:
// int32_t floor_count = 0;                                 // green color count from color filter for floor detection
float oa_floor_count_frac = 0.065f;                       // floor detection threshold as a fraction of total of image

int32_t safe_heading_green = 0;                          // safe heading for green detection

int16_t obstacle_free_confidence_orange = 0;             // a measure of how certain we are that the way ahead is safe for orange detection
const int16_t max_trajectory_confidence_orange = 5;      // number of consecutive negative object detections to be sure we are obstacle free for orange detection

// Distances:
float maxDistance = 0.8;                                 // max waypoint displacement [m]
float moveDistance = 0.f;                                // waypoint displacement [m] 
// float heading_increment_CCW = -8.f;                     // CCW heading angle increment [deg]
// float heading_increment_TurnAround = 8.f;               // Turn 180 [deg] CW in 45 [deg] increments to go back

// Heading:
float heading_increment = 0.f;                          // heading angle increment [deg]
const float heading_magnitude = 15.f;                        // heading angle magnitude [deg]
float heading_increment_OOB = 1.f * heading_magnitude; // CW heading angle increment [deg]

// COUNTER:
int16_t C_ROBUSTNESS = 0;

// Debugging:
enum navigation_state_t previous_state = SAFE;           // previous state

/*
 * This next section defines an ABI messaging event (http://wiki.paparazziuav.org/wiki/ABI), necessary
 * any time data calculated in another module needs to be accessed. Including the file where this external
 * data is defined is not enough, since modules are executed parallel to each other, at different frequencies,
 * in different threads. The ABI event is triggered every time new data is sent out, and as such the function
 * defined in this file does not need to be explicitly called, only bound in the init function
 */

////// ABI MESSAGES //////
//// Receive ABI message from cv_detect_color_object.c, where the quality value is of interest
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
                               int32_t direction, 
                               int32_t __attribute__((unused)) floor_color_count_img_segment,
                               int16_t __attribute__((unused)) extra)
{
  color_count = quality;
  // PRINT("[ORANGE] color_count: %d", color_count);
  safe_heading_green = direction;
  // PRINT("RAN CB ORANGE");
}

// //// Receive ABI message from opticflow_module.c, where the divergence value is of interest
// #ifndef OPTICAL_FLOW_ID
// #define OPTICAL_FLOW_ID ABI_BROADCAST
// #endif
// static abi_event optical_flow_ev;
// static void optical_flow_cb(uint8_t __attribute__((unused)) sender_id,
//                             uint32_t __attribute__((unused)) stamp, 
//                             int32_t __attribute__((unused)) flow_x,
//                             int32_t __attribute__((unused)) flow_y,
//                             int32_t __attribute__((unused)) flow_der_x,
//                             int32_t __attribute__((unused)) flow_der_y,
//                             float __attribute__((unused)) quality, 
//                             float size_divergence,
//                             double diff_divergence) 
// {
//   div_size = size_divergence;
//   div_diff = diff_divergence;
// }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    ////// RUN AUTOPILOT INIT FUNCTION //////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void orange_avoider_init(void)
{
  //// Initialise random values
  srand(time(NULL));

  //// Set the initial heading increment
  chooseRandomIncrementAvoidance();
  
  //// Bind the colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);

  // //// Bind the optical flow callbacks to receive the divergence values
  AbiBindMsgOPTICAL_FLOW(OPTICAL_FLOW_ID, &optical_flow_ev, optical_flow_cb);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    ////// RUN AUTOPILOT PERIODIC FUNCTION //////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void orange_avoider_periodic(void)
{
  //// Only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    return;
  }

  // Compute colour thresholds:
  int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h; // Front_camera defined in airframe xml, with the video_capture module

  ////// DETERMINE OBSTACLE FREE CONFIDENCE //////
  // Orange avoider
  if (color_count < color_count_threshold) {
    obstacle_free_confidence_orange++;
  } else {
    obstacle_free_confidence_orange -= 2; // Be more cautious with positive obstacle detections
  }

  // Bound obstacle_free_confidence_orange
  Bound(obstacle_free_confidence_orange, 0, max_trajectory_confidence_orange);

  ////// PRINT DETECTION VALUES //////
  PRINT("[ORANGE] Obstacle Free Confidence: %d; Orange Count: %d; Orange Threshold: %d \n", obstacle_free_confidence_orange, color_count, color_count_threshold); // Print visual detection pixel colour values and navigation state
  // PRINT("[GREEN] Counter No Green: %d; Green Count: %d; Green Threshold: %d \n", C_ROBUST_NO_GREEN, floor_count, floor_count_threshold); // Print visual detection pixel colour values.

  // Print safe heading green
  PRINT("[GREEN] SafeSafe Heading: %d \n", safe_heading_green); // Print visual detection pixel colour values and navigation state

  // Print obstacle free confidence orange
  //PRINT("[ORANGE] Obstacle Free Confidence: %d \n", obstacle_free_confidence_orange); // Print visual detection pixel colour values and navigation state

  // Print current state and possible safe heading caller
  if (previous_state != navigation_state) {
    //PRINT("[STATE] State: %d \n", navigation_state); // Print visual detection pixel colour values and navigation state
    previous_state = navigation_state;
  }
  // PRINT("[STATE] State: %d \n", navigation_state); // Print visual detection pixel colour values and navigation state

  // Distance waypoint moves ahead of drone -> compares both orange avoider and optical flow confidences followed by the maxDistance comparison and takes the min value out of all of them
  moveDistance = maxDistance; // fminf(maxDistance, 0.2f * obstacle_free_confidence_orange);

  ////// NAVIGATION STATE MACHINE //////
  switch (navigation_state) {
    case SAFE:
      moveWaypointForward(WP_TRAJECTORY, 1.9f * moveDistance);
      
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) 
      {
        navigation_state = OUT_OF_BOUNDS;
      } else if (safe_heading_green == 404) {
        navigation_state = TURN_AROUND;
      } else if (obstacle_free_confidence_orange == 0) {
        navigation_state = SEARCH_SAFE_HEADING;
        search_safe_heading_state = ORANGE;
      } else if (abs(safe_heading_green) == 1) 
      {
        navigation_state = SEARCH_SAFE_HEADING;
        search_safe_heading_state = GREEN;
      } else 
      {
        moveWaypointForward(WP_GOAL, moveDistance);
      }

      // Increment the counter for the number of times the drone has been in the safe state
      C_ROBUSTNESS++;
      // PRINT("ROBUSTNESS (SAFE): %d \n", C_ROBUSTNESS);

      // If next state is to search for a safe heading, set the heading increment
      if (navigation_state == SEARCH_SAFE_HEADING) {
        // if (C_ROBUSTNESS < 1) {
          // navigation_state = SAFE;
        //} 
        // else {
          // PRINT("Robustness exceeded 10, going to search safe heading called by %d \n", search_safe_heading_state);
          chooseRandomIncrementAvoidance();
          // C_ROBUSTNESS = 0;
        // }
      }

      break;
    case SEARCH_SAFE_HEADING:
      if (previous_state != navigation_state) {
        PRINT("[STATE] Search Safe Heading [CALLED BY] %d", search_safe_heading_state);
      }

      // Stop
      guidance_h_set_body_vel(0, 0);

      switch (search_safe_heading_state) {
        case ORANGE:
          // Stop
          guidance_h_set_body_vel(0, 0);
          /////////////// ORANGE AVOIDER ///////////////
          if (obstacle_free_confidence_orange >= 3){
            navigation_state = SAFE;
          } else {
            // Turn if there is an obstacle detected with orange avoider
            increase_nav_heading(heading_increment);
          }
          break;
        case GREEN:
          // Stop
          guidance_h_set_body_vel(0, 0);
          /////////////// NO-GREEN DETECTOR ///////////////
          increase_nav_heading(safe_heading_green * heading_magnitude);
          // Turn if there is an obstacle detected with green detector
          if (abs(safe_heading_green) == 0) 
          {
            navigation_state = SAFE;
          }
          break;
        default:
          break;
      }
      break;
    case TURN_AROUND:
      // Stop
      guidance_h_set_body_vel(0, 0);
      
      // Turn CW
      increase_nav_heading(90.f);

      if (safe_heading_green != 404) {
        navigation_state = SAFE;
      }

      break;
    case OUT_OF_BOUNDS:
      // Stop
      guidance_h_set_body_vel(0, 0);
      
      // Turn CW
      increase_nav_heading(heading_increment_OOB);
      
      // Move waypoint forward
      moveWaypointForward(WP_TRAJECTORY, 2.1f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        // Add offset to head back into arena
        increase_nav_heading(heading_increment_OOB);
        // Set confidence to 0
        obstacle_free_confidence_orange = 0;
        // Set robustness counter to 0
        C_ROBUSTNESS = 0;
        // Assume there is an obstacle to be safe (just in case)
        navigation_state = SAFE;
        // search_safe_heading_state = ORANGE;
        // Set the heading increment to CW
        heading_increment = heading_increment_OOB;
        //PRINT("[STATE] SAFE [CALLED BY] OOB");
      }
      break;
    default:
      break; 
  }
  return;
}

////// INCREASE NAV HEADING //////
/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  // for performance reasons the navigation variables are stored and processed in Binary Fixed-Point format
  nav_heading = ANGLE_BFP_OF_REAL(new_heading);

  // VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}

////// WAYPOINT FORWARD POSITION MOVEMENT //////
/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

////// FORWARD POSITION CALCULATION //////
/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  // VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
  //               POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
  //               stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

////// RE-ASSIGN WAYPOINT TO FORWARD POSITION //////
/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  //VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Sets the variable 'heading_increment' randomly positive/negative
 */

uint8_t chooseRandomIncrementAvoidance()
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    heading_increment = 1.f * heading_magnitude;
  } else {
    heading_increment = -1.f * heading_magnitude;
  }
  // PRINT("New heading increment: %f\n", heading_increment);
  return false;
}
