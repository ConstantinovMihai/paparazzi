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
 * @edit by Group 5 (2022-2023) for the course (AE4317) Autonomous Flight of Micro Air Vehicles 
 * @year 2022/2023  
 *
 * This module is used in combination with a color filter (cv_detect_color_object.c), optic flow (opticflow_module.c, 
 * opticflow_calculator.c, and size_divergence.c), and the navigation mode of the autopilot.
 *
 * The main avoidance strategy is to detect the colour pixels of the green floor and avoid obstacles using that (cv_detect_color_object.c). 
 * This is supplemented with the detection of orange colour pixels (cv_detect_color_object.c), the detection of green colour pixels of 
 * the plant (cv_detect_color_object.c), and the divergence size (size_divergence.c and opticflow_calculator.c).
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

// Print function for debugging
#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    ////// DEFINE SETTINGS AND VARIABLES //////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////// FUNCTIONS DEFINITION //////
static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
uint8_t chooseRandomIncrementAvoidance(void);

////// STATES AND VARIABLE DEFINITIONS AND INITIALISATION //////
// State machine:
// SAFE: Continue flying until obstacle detected 
// SEARCH_SAFE_HEADING: Obstacle detected, search for safe heading until it is safe to fly again
// TURN_AROUND: Obstacle detected, turn around until it is safe to fly again
// OUT_OF_BOUNDS: Out of bounds, turn until in bounds again
enum navigation_state_t {
  SAFE,
  SEARCH_SAFE_HEADING,
  TURN_AROUND,
  OUT_OF_BOUNDS,
};

// Sub state for search safe heading to make it clear which detection method called the state
enum search_safe_heading_state_t {
  ORANGE,
  GREEN,
};

// Sub state for turn around to make it clear which detection method called the state
enum turn_around_state_t {
  GREEN_TURN_AROUND,
  DIV_SIZE,
};

// Initialising the state machine, the state is set to SEARCH_SAFE_HEADING, ORANGE, and GREEN
enum navigation_state_t navigation_state = SEARCH_SAFE_HEADING;
enum search_safe_heading_state_t search_safe_heading_state = ORANGE;
enum turn_around_state_t turn_around_state = GREEN;

// Orange detector:
int32_t color_count = 0;                                 // Orange color count from color filter for obstacle detection
float oa_color_count_frac = 0.18f;                       // Percentage of image that is considered orange for the threshold

// Green detector:
int32_t safe_heading_green = 0;                          // Safe heading for green detection received from cv_detect_color_object.c

// Optical flow detector:
float div_size = 0;                                      // Divergence size for obstacle detection
float divergence_threshold = 0.065f;                     // Threshold for the divergence size 

int16_t obstacle_free_confidence_orange = 0;             // Measure of how certain we are that the way ahead is safe for orange detection
int16_t obstacle_free_confidence_floor = 0;              // Measure of how certain we are that the way ahead is safe for floor detection
int16_t obstacle_free_confidence_div_size = 0;           // Measure of how certain we are that the way ahead is safe for divergence size detection
const int16_t max_trajectory_confidence_orange = 5;      // Number of consecutive negative object detections to be sure we are obstacle free for orange detection
const int16_t max_trajectory_confidence_floor = 5;       // Number of consecutive negative object detections to be sure we are obstacle free for floor detection
const int16_t max_trajectory_confidence_opticalflow = 5; // Number of consecutive negative object detections to be sure we are obstacle free for divergence size detection

// Distances:
float maxDistance = 0.5;                                 // Max waypoint displacement [m]
float moveDistance = 0.0f;                               // Waypoint displacement [m] 

// Heading:
const float heading_magnitude = 5.f;                     // Heading angle magnitude [deg]
float heading_increment = 1.f * heading_magnitude;       // Heading angle increment [deg]
const float heading_magnitude_OOB = 15.f;                // OUT_OF_BOUNDS heading angle magnitude [deg]
float heading_increment_OOB = 1.f * heading_magnitude;   // OUT_OF_BOUNDS heading angle increment [deg]
const float heading_magnitude_TA = 30.f;                 // TURN_AROUND heading angle magnitude [deg]
float heading_increment_TA = 1.f * heading_magnitude_TA; // TURN_AROUND heading angle increment [deg]

// Counters:
int16_t C_DIV_SIZE_MAX_HEADING = 0;                      // Counter to allow for a max heading angle of 90 [deg] for divergence size

/*
 * This next section defines an ABI messaging event (http://wiki.paparazziuav.org/wiki/ABI), necessary
 * any time data calculated in another module needs to be accessed. Including the file where this external
 * data is defined is not enough, since modules are executed parallel to each other, at different frequencies,
 * in different threads. The ABI event is triggered every time new data is sent out, and as such the function
 * defined in this file does not need to be explicitly called, only bound in the init function
 */

////// ABI MESSAGES //////
//// Receive ABI message from cv_detect_color_object.c, where the quality (orange detection) and safe heading direction  (floor colour detection) values are of interest
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
  color_count = quality;           // Orange color count from color filter for obstacle detection  
  safe_heading_green = direction;  // Safe heading for green detection received from cv_detect_color_object.c
}

//// Receive ABI message from opticflow_module.c, where the divergence size value is of interest
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
  div_size = size_divergence;      // Divergence size for obstacle detection
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    ////// RUN AUTOPILOT INIT FUNCTION //////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void orange_avoider_init(void)
{
  //// Initialise random values
  srand(time(NULL));

  //// Set the initial heading increment:
  chooseRandomIncrementAvoidance();
  
  //// Bind the colorfilter callbacks to receive the color filter outputs:
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);

  //// Bind the optical flow callbacks to receive the divergence size values:
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
  ////// ONLY EVALUATE STATE MACHINE IF FLYING //////
  if(!autopilot_in_flight()){
    return;
  }

  ////// COMPUTE COLOUR THRESHOLDS FOR ORANGE //////
  int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h; // Front_camera defined in airframe xml, with the video_capture module

  ////// DETERMINE OBSTACLE FREE CONFIDENCE //////
  // Orange detection:
  if (color_count < color_count_threshold) {
    obstacle_free_confidence_orange++;
  } else {
    obstacle_free_confidence_orange -= 2; // Be more cautious with positive obstacle detections
  }

  // Floor/Green detection (for velocity setpoint):
  if (abs(safe_heading_green) == 0) {
    obstacle_free_confidence_floor++;
  } else {
    obstacle_free_confidence_floor -= 2; // Be more cautious with positive obstacle detections
  }

  // Divergence size detection:
  if (fabs(div_size) < divergence_threshold) {
    obstacle_free_confidence_div_size++;
  } else {
    obstacle_free_confidence_div_size -= 1; // Be more cautious with positive obstacle detections
  }

  ////// BIND OBSTACLE FREE CONFIDENCE VALUES TO A RANGE //////
  // Orange detection:
  Bound(obstacle_free_confidence_orange, 0, max_trajectory_confidence_orange);          // Bound obstacle_free_confidence_orange

  // Floor detection:
  Bound(obstacle_free_confidence_floor, 0, max_trajectory_confidence_floor);            // Bound obstacle_free_confidence_floor

  // Divergence size detection:
  Bound(obstacle_free_confidence_div_size, 0, max_trajectory_confidence_opticalflow);   // Bound obstacle_free_confidence_div_size

  ////// PRINT DETECTION VALUES //////
  // Orange detection:
  // Print obstacle free confidence for orange and visual detection pixel colour values
  // PRINT("[ORANGE] Obstacle Free Confidence: %d; Orange Count: %d; Orange Threshold: %d \n", obstacle_free_confidence_orange, color_count, color_count_threshold); 
  
  // Floor/Green detection:
  // PRINT("[GREEN] Obstacle Free Confidence: %d; Safe Heading: %d \n", obstacle_free_confidence_floor, safe_heading_green); // Print safe heading green

  // Divergence size detection:
  // PRINT("[DIV_SIZE] Obstacle Free Confidence: %d; Size: %lf; Threshold: %f \n", obstacle_free_confidence_div_size, div_size, divergence_threshold); // Print div_size

  ////// SETTING VELOCITY/MOVEMENT //////
  // The distance the waypoint moves ahead of drone -> compares both orange detection and floor detection confidences followed by a comparison with divergence size detection, which is then compared to the 
  // maxDistance and it takes the min value out of all of them  
  float moveDistance_temp = fminf(0.2f * obstacle_free_confidence_orange, 0.2f * obstacle_free_confidence_floor);
  float moveDistance_temp2 = fminf(moveDistance_temp, 0.2f * obstacle_free_confidence_div_size);
  
  moveDistance = fminf(maxDistance, moveDistance_temp2);

  ////// NAVIGATION STATE MACHINE //////
  switch (navigation_state) {
    case SAFE:
      // Move TRAJECTORY waypoint forwards:
      moveWaypointForward(WP_TRAJECTORY, 1.9f * moveDistance);
      
      // OUT OF BOUNDS CHECK:
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) 
      {
        navigation_state = OUT_OF_BOUNDS; // If the waypoint is out of bounds, then change state to out of bounds
      } 
      
      // ORANGE OBSTACLE DETECTION:
      else if (obstacle_free_confidence_orange == 0) 
      {
        navigation_state = SEARCH_SAFE_HEADING;
        search_safe_heading_state = ORANGE;
      } 

      // FLOOR OBSTACLE DETECTION:
      else if (abs(safe_heading_green) == 1) {
        navigation_state = SEARCH_SAFE_HEADING;
        search_safe_heading_state = GREEN;
      } 
      
      // FLOOR "ERROR 404" (Does not understand what is happening so turn around):
      else if (safe_heading_green == 404) 
      {
        navigation_state = TURN_AROUND;
        turn_around_state = GREEN_TURN_AROUND;
      } 

      // DIVERGENCE SIZE DETECTION:
      else if (obstacle_free_confidence_div_size == 0) 
      {
        navigation_state = TURN_AROUND;
        turn_around_state = DIV_SIZE;
      } 
      
      // IF NO ABOVE CONDITION MET -> Move GOAL waypoint forwards
      else 
      {
        moveWaypointForward(WP_GOAL, moveDistance);
      }
      
      // If next state is to search for a safe heading, set the heading increment to random (only for orange detections)
      if (navigation_state == SEARCH_SAFE_HEADING) 
      {
        chooseRandomIncrementAvoidance();
      }

      break;
    
    // Search for a safe heading to move to (either orange or floor detection)
    case SEARCH_SAFE_HEADING:
      // Stop
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);

      switch (search_safe_heading_state) {
        //// Orange detection ////
        case ORANGE:
        // Stop
        waypoint_move_here_2d(WP_GOAL);
        waypoint_move_here_2d(WP_TRAJECTORY);

          // If confident there is no obstacle -> set navigation state to SAFE
          // If not confident and an obstacle is still detected, keep on turning
          if (obstacle_free_confidence_orange >= 3)
          {
            navigation_state = SAFE;
          } else 
          {
            increase_nav_heading(heading_increment);
          }

          break;
        
        //// Floor/No-Green detection ////
        case GREEN:
        // Stop
        waypoint_move_here_2d(WP_GOAL);
        waypoint_move_here_2d(WP_TRAJECTORY);

          // Continuosly turn using safe_heading_green (-1, 0, 1) until no obstacle is detected anymore
          increase_nav_heading(safe_heading_green * heading_magnitude);
          
          // Obstacle not found in the middle (right in front of the drone)
          if (abs(safe_heading_green) == 0) 
          {
            navigation_state = SAFE;
          }

        break;
      default:
        break;
      }
      break;
    // Turn around if "ERROR 404" or divergence size detection
    case TURN_AROUND:
      // Stop
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);
      
      switch (turn_around_state) {
        
        //// Floor/No-Green detection -> "ERROR 404" ////
        case GREEN_TURN_AROUND:
          // Stop
          waypoint_move_here_2d(WP_GOAL);
          waypoint_move_here_2d(WP_TRAJECTORY);

          // Turn CW
          increase_nav_heading(heading_increment_TA);

          // "Error 404" has ended so return to SAFE state
          if (safe_heading_green != 404) 
          {
            navigation_state = SAFE;
          }

        break;

        //// Divergence Size Detection ////
        case DIV_SIZE:
          // Stop
          waypoint_move_here_2d(WP_GOAL);
          waypoint_move_here_2d(WP_TRAJECTORY);

          // Turn CW
          increase_nav_heading(heading_increment_TA);
          
          // Max allowed turn is 180 [deg] or C_DIV_SIZE_MAX_HEADING = 6 (for an increment of 30 [deg])
          C_DIV_SIZE_MAX_HEADING++;

          // If confident there is no obstacle -> set navigation state to SAFE
          // If not confident and an obstacle is still detected, keep on turning until 180 [deg] turn is made (max limit)
          if (obstacle_free_confidence_div_size >= 3 || C_DIV_SIZE_MAX_HEADING < 6)
          { 
            navigation_state = SAFE;
            C_DIV_SIZE_MAX_HEADING = 0;
          }
        
        break;
        
      default:
        break;
      }
      break;
    
    // Out of bounds detection
    case OUT_OF_BOUNDS:
      // Stop
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);
      
      // Turn CW
      increase_nav_heading(heading_increment_OOB);
      
      // Move waypoint forward
      moveWaypointForward(WP_TRAJECTORY, 2.1f);

      // If waypoint is back in bounds, set navigation state to SAFE
      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        // Add offset to head back into arena
        increase_nav_heading(heading_increment_OOB);
        // Set confidence to 0
        obstacle_free_confidence_orange = 0;
        obstacle_free_confidence_div_size = 0;
        // Set navigation state to SAFE
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
  
  return false;
}

////// RE-ASSIGN WAYPOINT TO FORWARD POSITION //////
/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Sets the variable 'heading_increment' randomly positive/negative
 */

uint8_t chooseRandomIncrementAvoidance()
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) 
  {
    heading_increment = 1.f * heading_magnitude;
  } else 
  {
    heading_increment = -1.f * heading_magnitude;
  }
  
  return false;
}
