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
 * using the divergence difference value between left and right of image (div_size_difference_mean) and the divergence threshold (divergence_threshold).
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

////// STATES AND VARIABLE DEFINITION AND INITIALISATION //////
enum navigation_state_t {
  SAFE,
  TURN,
  //RETURN,
  OUT_OF_BOUNDS,
};

enum navigation_state_t navigation_state = SAFE;

int orange_detection = 0;                                // used for debugging, 0=not detected, 1=detected
int div_diff_detection = 0;                     // used for debugging, 0=not detected, 1=detected
int div_size_detection = 0;                      // used for debugging, 0=not detected, 1=detected
// int opticalflow_detection_Zero = 0;                      // triggers detection when divergence difference is 0
// int32_t opticalflow_detection_counter_Zero = 0;          // counter for when divergence difference is 0
int out_of_bounds_detection = 0;                         // used for debugging, 0=not detected, 1=detected

int32_t color_count = 0;                                 // orange color count from color filter for obstacle detection
float oa_color_count_frac = 0.18f;

int32_t safe_heading_g;
int32_t safe_heading_confidence_g;

float div_size = 0.f;                                    // divergence size -> see size_divergence.c
float div_diff = 0.f;                                    // divergence difference between right and left half of image to determine whether there is an obstacle in left or right half of image -> see size_divergence.c
int32_t divergence_threshold = 0.3;                        // threshold for the divergence value for optical flow object detection
int32_t divergence_difference_threshold = 1.0;             // threshold for the divergence difference value for optical flow object detection

int16_t obstacle_free_confidence_orange = 0;             // a measure of how certain we are that the way ahead is safe for orange detection
int16_t obstacle_free_confidence_div_diff = 0;
int16_t obstacle_free_confidence_div_size = 0;
const int16_t max_trajectory_confidence_orange = 5;      // number of consecutive negative object detections to be sure we are obstacle free for orange detection
const int16_t max_trajectory_confidence_div_diff = 5;      // number of consecutive negative object detections to be sure we are obstacle free for orange detection
const int16_t max_trajectory_confidence_div_size = 5;      // number of consecutive negative object detections to be sure we are obstacle free for orange detection

float maxDistance = 0.8;                                 // max waypoint displacement [m]
float moveDistance = 0.f;                                // waypoint displacement [m]
float heading_increment_CW = 12.f;                       // CW heading angle increment [deg]
float heading_increment_TurnAround = 90.f;              // Turn 180 [deg] CW to go back

float Kp = 1.0;

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
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, 
                               int32_t safe_heading, 
                               int32_t safe_heading_confidence,
                               int16_t __attribute__((unused)) extra)
{
  color_count = quality;
  safe_heading_g = safe_heading;
  safe_heading_confidence_g = safe_heading_confidence;
}

//// Receive ABI message from opticflow_module.c, where the divergence value is of interest
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
                            float size_divergence,
                            float diff_divergence) 
{
  div_size = size_divergence;
  div_diff = diff_divergence;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    ////// RUN AUTOPILOT INIT FUNCTION //////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void orange_avoider_init(void)
{
  //// Initialise random values
  srand(time(NULL));
  
  //// Bind the colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);

  //// Bind the optical flow callbacks to receive the divergence values
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

  ////// COMPUTE CURRENT COLOR THRESHOLDS //////
  int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h; // Front_camera defined in airframe xml, with the video_capture module

  ////// PRINT DETECTION VALUES //////
  //VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state); // Print visual detection pixel colour values and navigation state
  // VERBOSE_PRINT("Divergence size: %lf Divergence threshold: %lf \n", div_size, divergence_threshold); // Print optical flow divergence size
  // VERBOSE_PRINT("Divergence difference: %lf Divergence threshold: %lf \n", div_diff, divergence_threshold); // Print optical flow divergence difference
  // VERBOSE_PRINT("Out Of Bounds Detection: %d Orange Detection: %d Div Diff Detection: %d Div Size Detection: %d \n", out_of_bounds_detection, orange_detection, div_diff_detection, div_size_detection); // Print optical flow and orange detection
  //VERBOSE_PRINT("Obstacle Free Confidence Orange: %d Obstacle Free Confidence Div Diff: %d Obstacle Free Confidence Div Size: %d \n", obstacle_free_confidence_orange, obstacle_free_confidence_div_diff, obstacle_free_confidence_div_size);
  //VERBOSE_PRINT("Div Diff: %lf Div Size: %lf \n", div_diff, div_size);
  //VERBOSE_PRINT("State: %d \n", navigation_state);
  VERBOSE_PRINT("safe_heading: %d  safe_heading_confidence: %d state: %d \n", safe_heading_g, safe_heading_confidence_g, navigation_state);
  
  ////// DETERMINE OBSTACLE FREE CONFIDENCE //////
  if (color_count < color_count_threshold) {
    obstacle_free_confidence_orange++;
  } else {
    obstacle_free_confidence_orange -= 2; // Be more cautious with positive obstacle detections
  }

  // if (fabs(div_diff) < divergence_difference_threshold) {
  //   obstacle_free_confidence_div_diff++;
  // } else {
  //   obstacle_free_confidence_div_diff -= 2;  
  // }

  // if (div_size < divergence_threshold) {
  //   obstacle_free_confidence_div_size++;
  // } else {
  //   obstacle_free_confidence_div_size -= 2;  
  // }

  // Bound obstacle_free_confidence_orange
  Bound(obstacle_free_confidence_orange, 0, max_trajectory_confidence_orange);

  // // Bound obstacle_free_confidence_div_diff
  // Bound(obstacle_free_confidence_div_diff, 0, max_trajectory_confidence_div_diff);

  // // Bound obstacle_free_confidence_div_size
  // Bound(obstacle_free_confidence_div_size, 0, max_trajectory_confidence_div_size);

  // Distance waypoint moves ahead of drone -> compares both orange avoider and optical flow confidences followed by the maxDistance comparison and takes the min value out of all of them
  float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence_orange);
  // float moveDistance_temp2 = fminf(maxDistance, 0.2f * obstacle_free_confidence_opticalflow_right);
  // float moveDistance_temp3 = fminf(maxDistance, 0.2f * obstacle_free_confidence_opticalflow_left);


  // if (moveDistance_temp1 < moveDistance_temp2) {
  //   moveDistance = moveDistance_temp1;
  // } else if (moveDistance_temp1 < moveDistance_temp3) {
  //   moveDistance = moveDistance_temp1;
  // } else if (moveDistance_temp2 < moveDistance_temp1) {
  //   moveDistance = moveDistance_temp2;  
  // } else if (moveDistance_temp2 < moveDistance_temp3) {
  //   moveDistance = moveDistance_temp2;  
  // } else if (moveDistance_temp3 < moveDistance_temp1) {
  //   moveDistance = moveDistance_temp3;  
  // } else if (moveDistance_temp3 < moveDistance_temp2) {
  //   moveDistance = moveDistance_temp3;  
  // } else {
  //   moveDistance = maxDistance;         
  // }

  ////// NAVIGATION STATE MACHINE //////
  switch (navigation_state) {
    case SAFE:
      moveWaypointForward(WP_TRAJECTORY, 1.9f * moveDistance);
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        out_of_bounds_detection = 1;
        navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence_orange == 0) {
        orange_detection = 1;
        navigation_state = TURN;
      // } else if (obstacle_free_confidence_div_diff == 0) {
      //   div_diff_detection = 1;
      //   navigation_state = TURN;
      // } else if (obstacle_free_confidence_div_size == 0) {
      //   div_size_detection = 1;
      //   navigation_state = RETURN;
      // } 
      } else {
        out_of_bounds_detection = 0;
        orange_detection = 0;
        // div_diff_detection = 0;
        // div_size_detection = 0;
        moveWaypointForward(WP_GOAL, moveDistance);
      }
      break;
    case TURN:
      // Stop
      guidance_h_set_body_vel(0, 0);

      if (obstacle_free_confidence_orange >= 3){
        navigation_state = SAFE;
      } else {
        increase_nav_heading(heading_increment_CW);
      }

      // if (obstacle_free_confidence_div_diff >= 3) {
      //   navigation_state = SAFE;
      // } else {
      //   increase_nav_heading(div_diff * Kp);
      // }
      break; 
    // case RETURN:
    //   // Stop
    //   guidance_h_set_body_vel(0, 0);
      
    //   // if (obstacle_free_confidence_div_size >= 3) {
    //   //   navigation_state = SAFE;
    //   // } else {
    //   //   increase_nav_heading(heading_increment_TurnAround);
    //   // }

    //   break;
    case OUT_OF_BOUNDS:
      // Stop
      guidance_h_set_body_vel(0, 0);

      increase_nav_heading(heading_increment_CW);
      
      moveWaypointForward(WP_TRAJECTORY, 2.1f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        // Add offset to head back into arena
        increase_nav_heading(heading_increment_CW);
        obstacle_free_confidence_orange = 0;
        obstacle_free_confidence_div_diff = 0;
        obstacle_free_confidence_div_size = 0;
        navigation_state = SAFE;
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