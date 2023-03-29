/*
 * Copyright (C) 2019 Kirk Scheper <kirkscheper@gmail.com>
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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/cv_detect_object.h
 * Assumes the object consists of a continuous color and checks
 * if you are over the defined object or not
 */

// Own header
#include "modules/computer_vision/cv_detect_color_object.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef COLOR_OBJECT_DETECTOR_FPS1
#define COLOR_OBJECT_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif
#ifndef COLOR_OBJECT_DETECTOR_FPS2
#define COLOR_OBJECT_DETECTOR_FPS2 0 ///< Default FPS (zero means run at camera fps)
#endif

// Filter Settings
uint8_t cod_lum_min1 = 0;
uint8_t cod_lum_max1 = 0;
uint8_t cod_cb_min1 = 0;
uint8_t cod_cb_max1 = 0;
uint8_t cod_cr_min1 = 0;
uint8_t cod_cr_max1 = 0;

uint8_t cod_lum_min2 = 0;
uint8_t cod_lum_max2 = 0;
uint8_t cod_cb_min2 = 0;
uint8_t cod_cb_max2 = 0;
uint8_t cod_cr_min2 = 0;
uint8_t cod_cr_max2 = 0;

bool cod_draw1 = false;
bool cod_draw2 = false;

// Module settings new to include the green floor detection
uint8_t cod_lum_min1_floor;
uint8_t cod_lum_max1_floor;
uint8_t cod_cb_min1_floor;
uint8_t cod_cb_max1_floor;
uint8_t cod_cr_min1_floor;
uint8_t cod_cr_max1_floor;

// Module settings new to include the green thin plant detection
uint8_t cod_lum_min1_plant;
uint8_t cod_lum_max1_plant;
uint8_t cod_cb_min1_plant;
uint8_t cod_cb_max1_plant;
uint8_t cod_cr_min1_plant;
uint8_t cod_cr_max1_plant;

////PART1/3: DEFINE THE SIGNATURE OF THE FUNCTION
// define global variables
struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  int32_t direction;
  int32_t floor_color_count_img_segment;
  bool updated;
};
struct color_object_t global_filters[2];


////// (NEW) define global variables for determining the direction in which the drone should go, straight ahead. left, right or go back
//struct direction_decision_object_t {
//    int32_t direction;
//    int32_t floor_color_count_img_segment;
//    bool updated;
//};
//struct direction_decision_object_t global_filters_floor[2];



/// General function to detect the object centroid, initially implemented in cv_detect_color_object
/// used later in the implementation of orange_avoider_guided (kept in case we decide to use later)
/// Function (detects object centroid, color count of an object +
/// improved with floor count + image segmentation to detect the direction(front, left right, back)
/// if green color count is too low
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              int32_t *direction, int32_t *floor_color_count_img_segment,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max,
                              uint8_t lum_min_floor, uint8_t lum_max_floor,
                              uint8_t cb_min_floor, uint8_t cb_max_floor,
                              uint8_t cr_min_floor, uint8_t cr_max_floor,
                              uint8_t lum_min_plant, uint8_t lum_max_plant,
                              uint8_t cb_min_plant, uint8_t cb_max_plant,
                              uint8_t cr_min_plant ,uint8_t cr_max_plant);



/// Decide which camera to use
/*
 * object_detector
 * @param img - input image to process
 * @param filter - which detection filter to process
 * @return img
 */
/// FUNCTION signature to detect and object, to be called later on
/// two different times once for the orange detector and once for the green dectector
static struct image_t *object_detector(struct image_t *img, uint8_t filter)
{
  uint8_t lum_min, lum_max;
  uint8_t cb_min, cb_max;
  uint8_t cr_min, cr_max;
  bool draw;
  uint8_t lum_min_floor, lum_max_floor;
  uint8_t cb_min_floor, cb_max_floor;
  uint8_t cr_min_floor, cr_max_floor;

  uint8_t lum_min_plant, lum_max_plant;
  uint8_t cb_min_plant, cb_max_plant;
  uint8_t cr_min_plant, cr_max_plant;

  switch (filter){
    case 1:
      lum_min = cod_lum_min1;
      lum_max = cod_lum_max1;
      cb_min = cod_cb_min1;
      cb_max = cod_cb_max1;
      cr_min = cod_cr_min1;
      cr_max = cod_cr_max1;
      //draw = cod_draw1;

      lum_min_floor = cod_lum_min1_floor;
      lum_max_floor = cod_lum_max1_floor;
      cb_min_floor = cod_cb_min1_floor;
      cb_max_floor = cod_cb_max1_floor;
      cr_min_floor = cod_cr_min1_floor;
      cr_max_floor = cod_cr_max1_floor;

      lum_min_plant = cod_lum_min1_plant;
      lum_max_plant = cod_lum_max1_plant;
      cb_min_plant = cod_cb_min1_plant;
      cb_max_plant = cod_cb_max1_plant;
      cr_min_plant = cod_cr_min1_plant;
      cr_max_plant = cod_cr_max1_plant;
      
      draw = cod_draw1;
      
      break;
    case 2:
      lum_min = cod_lum_min2;
      lum_max = cod_lum_max2;
      cb_min = cod_cb_min2;
      cb_max = cod_cb_max2;
      cr_min = cod_cr_min2;
      cr_max = cod_cr_max2;
      draw = cod_draw2;

      lum_min_floor = cod_lum_min1_floor;
      lum_max_floor = cod_lum_max1_floor;
      cb_min_floor = cod_cb_min1_floor;
      cb_max_floor = cod_cb_max1_floor;
      cr_min_floor = cod_cr_min1_floor;
      cr_max_floor = cod_cr_max1_floor;

      lum_min_plant = cod_lum_min1_plant;
      lum_max_plant = cod_lum_max1_plant;
      cb_min_plant = cod_cb_min1_plant;
      cb_max_plant = cod_cb_max1_plant;
      cr_min_plant = cod_cr_min1_plant;
      cr_max_plant = cod_cr_max1_plant;
      break;
    default:
      return img;
  };

  int32_t x_c, y_c;
  /// (NEW) initialize the two variables needed
  int32_t direction = 0;
  int32_t floor_color_count_img_segment;

  // Filter and find centroid
  uint32_t count = find_object_centroid(img, &x_c, &y_c, draw, &direction, &floor_color_count_img_segment, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max, lum_min_floor, lum_max_floor, cb_min_floor, cb_max_floor, cr_min_floor, cr_max_floor, lum_min_plant, lum_max_plant, cb_min_plant, cb_max_plant, cr_min_plant, cr_max_plant);
  VERBOSE_PRINT("Color count %d: %u, threshold %u, x_c %d, y_c %d\n", camera, object_count, count_threshold, x_c, y_c);
  VERBOSE_PRINT("centroid %d: (%d, %d) r: %4.2f a: %4.2f\n", camera, x_c, y_c,
        hypotf(x_c, y_c) / hypotf(img->w * 0.5, img->h * 0.5), RadOfDeg(atan2f(y_c, x_c)));

  pthread_mutex_lock(&mutex);
  global_filters[filter-1].color_count = count;
  global_filters[filter-1].x_c = x_c;
  global_filters[filter-1].y_c = y_c;
  global_filters[filter-1].direction = direction;
  global_filters[filter-1].floor_color_count_img_segment = floor_color_count_img_segment;
  global_filters[filter-1].updated = true;
  pthread_mutex_unlock(&mutex);

  return img;
}

//// FRONT CAMERA INFORMATION
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 1);
}

//// BOTTOM CAMERA INFORMATION (not used in this implementation)
struct image_t *object_detector2(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector2(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 2);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////
                                //// PART2/3: DEFINE THE ACTUALLY ////
///////////////////////////////////////////////////////////////////////////////////////////////////////
void color_object_detector_init(void) {
    memset(global_filters, 0, 2 * sizeof(struct color_object_t));
    pthread_mutex_init(&mutex, NULL);
#ifdef COLOR_OBJECT_DETECTOR_CAMERA1
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN1
    cod_lum_min1 = COLOR_OBJECT_DETECTOR_LUM_MIN1;
    cod_lum_max1 = COLOR_OBJECT_DETECTOR_LUM_MAX1;
    cod_cb_min1 = COLOR_OBJECT_DETECTOR_CB_MIN1;
    cod_cb_max1 = COLOR_OBJECT_DETECTOR_CB_MAX1;
    cod_cr_min1 = COLOR_OBJECT_DETECTOR_CR_MIN1;
    cod_cr_max1 = COLOR_OBJECT_DETECTOR_CR_MAX1;

    cod_lum_min1_floor = COLOR_OBJECT_DETECTOR_LUM_MIN2;
    cod_lum_max1_floor = COLOR_OBJECT_DETECTOR_LUM_MAX2;
    cod_cb_min1_floor = COLOR_OBJECT_DETECTOR_CB_MIN2;
    cod_cb_max1_floor = COLOR_OBJECT_DETECTOR_CB_MAX2;
    cod_cr_min1_floor = COLOR_OBJECT_DETECTOR_CR_MIN2;
    cod_cr_max1_floor = COLOR_OBJECT_DETECTOR_CR_MAX2;

    cod_lum_min1_plant = COLOR_OBJECT_DETECTOR_LUM_MIN2_PLANT;
    cod_lum_max1_plant = COLOR_OBJECT_DETECTOR_LUM_MAX2_PLANT;
    cod_cb_min1_plant = COLOR_OBJECT_DETECTOR_CB_MIN2_PLANT;
    cod_cb_max1_plant = COLOR_OBJECT_DETECTOR_CB_MAX2_PLANT;
    cod_cr_min1_plant = COLOR_OBJECT_DETECTOR_CR_MIN2_PLANT;
    cod_cr_max1_plant = COLOR_OBJECT_DETECTOR_CR_MAX2_PLANT;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW1
    cod_draw1 = COLOR_OBJECT_DETECTOR_DRAW1;
#endif

    cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, object_detector1, COLOR_OBJECT_DETECTOR_FPS1, 0);
#endif

//// (NEW) Comment in if you want Bottom Cmaera
#ifdef COLOR_OBJECT_DETECTOR_CAMERA2
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN2
    cod_lum_min2 = COLOR_OBJECT_DETECTOR_LUM_MIN2;
    cod_lum_max2 = COLOR_OBJECT_DETECTOR_LUM_MAX2;
    cod_cb_min2 = COLOR_OBJECT_DETECTOR_CB_MIN2;
    cod_cb_max2 = COLOR_OBJECT_DETECTOR_CB_MAX2;
    cod_cr_min2 = COLOR_OBJECT_DETECTOR_CR_MIN2;
    cod_cr_max2 = COLOR_OBJECT_DETECTOR_CR_MAX2;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW2
    cod_draw2 = COLOR_OBJECT_DETECTOR_DRAW2;
#endif

    cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA2, object_detector2, COLOR_OBJECT_DETECTOR_FPS2, 1);
#endif
}


/*
 * find_object_centroid
 *
 * Finds the centroid of pixels in an image within filter bounds.
 * Also returns the amount of pixels that satisfy these filter bounds.
 *
 * @param img - input image to process formatted as YUV422.
 * @param p_xc - x coordinate of the centroid of color object
 * @param p_yc - y coordinate of the centroid of color object
 * @param lum_min - minimum y value for the filter in YCbCr colorspace
 * @param lum_max - maximum y value for the filter in YCbCr colorspace
 * @param cb_min - minimum cb value for the filter in YCbCr colorspace
 * @param cb_max - maximum cb value for the filter in YCbCr colorspace
 * @param cr_min - minimum cr value for the filter in YCbCr colorspace
 * @param cr_max - maximum cr value for the filter in YCbCr colorspace
 * @param draw - whether or not to draw on image
 * @return number of pixels of image within the filter bounds.
 */
uint32_t find_object_centroid(struct image_t *img, int32_t *p_xc, int32_t *p_yc, bool draw,
                                int32_t *direction, int32_t *floor_color_count_img_segment,
                                uint8_t lum_min, uint8_t lum_max,
                                uint8_t cb_min, uint8_t cb_max,
                                uint8_t cr_min, uint8_t cr_max,
                                uint8_t lum_min_floor, uint8_t lum_max_floor,
                                uint8_t cb_min_floor, uint8_t cb_max_floor,
                                uint8_t cr_min_floor, uint8_t cr_max_floor, 
                                uint8_t lum_min_plant, uint8_t lum_max_plant,
                                uint8_t cb_min_plant, uint8_t cb_max_plant,
                                uint8_t cr_min_plant ,uint8_t cr_max_plant)
{
    uint32_t cnt_orange = 0; /// color count for obstacle i.e. orange
    uint32_t tot_x = 0;
    uint32_t tot_y = 0;
    uint8_t *buffer = img->buf;

    /// (NEW) define the new varialbes
    //*direction ; /// default 0; middle; negative = left; positive = right (to be scaled later)
    //  int32_t cnt_green = 288;
    int32_t img_segments = 3;
    int32_t mid_segment = 1;
    int32_t cnt_green = 0;
    int32_t cnt_green_plant = 0;
    float cropped_image_factor = 0.1f;
    
    // define settings
    //float oag_color_count_frac = 0.18f;       // obstacle detection threshold as a fraction of total of image
    float oag_floor_count_frac = 0.05f;       // floor detection threshold as a fraction of total of image
    int32_t color_count_per_img_segment_arr[img_segments]; /// array storing number of green pixels for each of the 5 segments of the photo
    int32_t color_count_per_img_segment_arr_plant[img_segments]; /// array storing number of green pixels for each of the 5 segments of the photo
    int64_t floor_threshold_per_segment_arr[img_segments];

    // ORANGE
    // Go through all the pixels
    for (uint16_t y = 0; y < img->h; y++) {
        for (uint16_t x = 0; x < img->w; x++) {
        // Check if the color is inside the specified values
        uint8_t *yp_orange, *up_orange, *vp_orange;
        if (x % 2 == 0) {
            // Even x
            up_orange = &buffer[y * 2 * img->w + 2 * x];      // U
            yp_orange = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
            vp_orange = &buffer[y * 2 * img->w + 2 * x + 2];  // V
            //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
        } else {
            // Uneven x
            up_orange = &buffer[y * 2 * img->w + 2 * x - 2];  // U
            //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
            vp_orange = &buffer[y * 2 * img->w + 2 * x];      // V
            yp_orange = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
        }
        
        // if pixel is orange count it for the total and make it white
        if ((*yp_orange >= lum_min) && (*yp_orange <= lum_max) &&
            (*up_orange >= cb_min) && (*up_orange <= cb_max) &&
            (*vp_orange >= cr_min) && (*vp_orange <= cr_max)) {
            cnt_orange++;
            // tot_x += x;
            // tot_y += y;
            // if (draw) {
            //     *yp_orange = 255;  // make pixel brighter in image
            // }
            }
        }
    }
    // Compute centroid for orange detection
    if (cnt_orange > 0) {
        *p_xc = (int32_t) roundf(tot_x / ((float) cnt_orange) - img->w * 0.5f);
        *p_yc = (int32_t) roundf(img->h * 0.5f - tot_y / ((float) cnt_orange));
    } else {
        *p_xc = 0;
        *p_yc = 0;
    }

    //GREEN
    // iterate over all possible headings from the image
    for (int i = 0; i < img_segments; i++)
    {
        // Determine the start and end of each of the possible heading sections
        uint32_t start_y = (uint16_t) roundf(i * img->h / (float) img_segments);
        uint32_t end_y = (uint16_t) roundf((i + 1) * img->h / (float) img_segments);
        uint32_t small_w = (uint16_t) roundf(img->w * (float) cropped_image_factor);
        // Reinitialize the green count per section
        cnt_green = 0;

        // Go through all the pixels
        for (uint32_t y = start_y; y < end_y; y++) {
            for (uint32_t x = 0; x < small_w; x++) {
                // Check if the color is inside the specified values
                uint8_t *yp_green, *up_green, *vp_green;
                if (x % 2 == 0) {
                    // Even x (Grayscale)
                    up_green = &buffer[y * 2 * img->w + 2 * x];      // U
                    yp_green = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
                    vp_green = &buffer[y * 2 * img->w + 2 * x + 2];  // V
                    //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
                } else {
                    // Odd x (Colour)
                    up_green = &buffer[y * 2 * img->w + 2 * x - 2];  // U
                    //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
                    vp_green = &buffer[y * 2 * img->w + 2 * x];      // V
                    yp_green = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
                }

                /// GREEN/FLOOR
                // if pixel is green count it for the total per section and make it black; maybe make it black
                if ((*yp_green >= lum_min_floor) && (*yp_green <= lum_max_floor) &&
                    (*up_green >= cb_min_floor) && (*up_green <= cb_max_floor) &&
                    (*vp_green >= cr_min_floor) && (*vp_green <= cr_max_floor)) {
                    cnt_green++;
                    // TODO: remove the following line when debugging is over
                    *yp_green = 0;  // make pixel dark in the image
                }

                // GREEN/PLANT
                // if pixel is green count it for the total per section and make it black; maybe make it black
                if ((*yp_green >= lum_min_plant) && (*yp_green <= lum_max_plant) &&
                    (*up_green >= cb_min_plant) && (*up_green <= cb_max_plant) &&
                    (*vp_green >= cr_min_plant) && (*vp_green <= cr_max_plant)) {
                    cnt_green_plant++;
                    *yp_green = 255;  // make pixel bright in the image
                }
            }
        }
        /// Compute the green threshold per section
        /// (NEW) WAY OF COMPUTING FLOOR THRESHOLD; adjust the orange_avoider_guided_threshold
        floor_threshold_per_segment_arr[i] = oag_floor_count_frac * (end_y-start_y) * small_w;
        //Compute the green color count per each of the segments of the photo
        color_count_per_img_segment_arr[i] = cnt_green;
        color_count_per_img_segment_arr_plant[i] = cnt_green_plant;
    }
  


    // after you iterated over all image segments
    /// Check if the default heading(2 - middle) is still safe or we need to change it
    /// assume max is mid segment
    /// check if it's actually the mid segment and if it's above the threshold
    /// get the direction with max value above threshold ig you cant find return 404
    float detectionTolerance = 2.0f; // 1.95f
    int32_t plant_greenThreshold = 300;
    int32_t maxValue = color_count_per_img_segment_arr[mid_segment];
    int32_t minValue = color_count_per_img_segment_arr[mid_segment];
    int32_t maxIndex = 1;
    bool atLeastOneAboveTh = false;
    int32_t margin_between_min_max = 60;
    bool plant_detection;
    bool above_th_arr[img_segments];

    // Check if the plant is detected
    plant_detection = color_count_per_img_segment_arr_plant[mid_segment] >= plant_greenThreshold; 
        
    // Check if the maximum color count is above the threshold and find the direction with the maximum color count
    for (int i = 0; i < img_segments; i++) {
        above_th_arr[i] = color_count_per_img_segment_arr[i] >= floor_threshold_per_segment_arr[i]*detectionTolerance;
    }

    for (int i = 0; i < img_segments; i++) {
        if (above_th_arr[i] == true) {
            atLeastOneAboveTh = true;
        }
        if (above_th_arr[i] == true && color_count_per_img_segment_arr[i] > maxValue){
            maxValue = color_count_per_img_segment_arr[i];
            maxIndex = i;
        } else if (atLeastOneAboveTh == false && color_count_per_img_segment_arr[i] > maxValue) {
            maxValue = color_count_per_img_segment_arr[i];
            maxIndex = i;
        }
        if (color_count_per_img_segment_arr[i] <= minValue) {
            minValue = color_count_per_img_segment_arr[i];
        }
    }
    // If the margin between min and max is too low, then set direction = 404
    if (atLeastOneAboveTh == false && (maxValue - minValue) < margin_between_min_max) {
        maxIndex = 3; // corresponds to 404 direction value (other than 0, 1 and 2)
    }

    if (plant_detection == true) {
        maxIndex = 3;
    }

    PRINT("[PLANT] Colour count: %ld; Threshold: %d; Detection: %d \n", color_count_per_img_segment_arr_plant[mid_segment], plant_greenThreshold, plant_detection);
    //PRINT("S-1 %ld, S0 %ld, S1 %ld \n", color_count_per_img_segment_arr[0], color_count_per_img_segment_arr[1], color_count_per_img_segment_arr[2]);
   // PRINT("TH-1 %ld, TH0 %ld, TH1 %ld \n", floor_threshold_per_segment_arr[0], floor_threshold_per_segment_arr[1], floor_threshold_per_segment_arr[2]);
    // set output variables
    if (maxIndex == 0) {
        *direction = -1;
    } else if (maxIndex == 1) {
        *direction = 0;
    } else if (maxIndex == 2) {
        *direction = 1;
    } else {
        *direction = 404; // error turn back
    }
    *floor_color_count_img_segment = maxValue;




    return cnt_orange;

}

//// PART 3/3: SEND MESSAGE TO NAVIGATION
void color_object_detector_periodic(void)
{
  static struct color_object_t local_filters[2];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  if(local_filters[0].updated){
        // PRINT("[CV_DETECT] direction: %d  floor_color_count_img_segment: %d\n", local_filters[0].direction, local_filters[0].floor_color_count_img_segment);

        AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
                                   0, 0, local_filters[0].color_count, local_filters[0].direction, local_filters[0].floor_color_count_img_segment, 0);
        local_filters[0].updated = false;
  }

  /// Comment IN if you want bottom camera info
  if(local_filters[1].updated){
        AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, local_filters[1].x_c, local_filters[1].y_c,
                                   0, 0, local_filters[1].color_count,local_filters[1].direction, local_filters[1].floor_color_count_img_segment, 1);
        local_filters[1].updated = false;
  }

}
