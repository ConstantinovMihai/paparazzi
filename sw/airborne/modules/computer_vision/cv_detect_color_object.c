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
                              uint8_t cr_min_floor, uint8_t cr_max_floor);



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
      break;
    default:
      return img;
  };

  int32_t x_c, y_c;
  /// (NEW) initialize the two variables needed
  int32_t direction;
  int32_t floor_color_count_img_segment;

  // Filter and find centroid
  uint32_t count = find_object_centroid(img, &x_c, &y_c, draw, &direction, &floor_color_count_img_segment, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max, lum_min_floor, lum_max_floor, cb_min_floor, cb_max_floor, cr_min_floor, cr_max_floor);
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
                                  uint8_t cr_min_floor, uint8_t cr_max_floor)
{
    uint32_t cnt = 0; /// color count for obstacle i.e. orange
    uint32_t tot_x = 0;
    uint32_t tot_y = 0;
    uint8_t *buffer = img->buf;

    /// (NEW) define the new varialbes
    // *direction = 0; /// default 0; middle; negative = left; positive = right (to be scaled later)
    //  int32_t cnt_green = 288;
    int32_t img_segments = 3;
    int32_t left_segment = 0;
    int32_t mid_segment = 1;
    int32_t right_segment = 2;
    int32_t cnt_green = 0;
    int32_t maxValue;
    float oag_floor_count_frac = 0.05f;       // floor detection threshold as a fraction of total of image
    int32_t color_count_per_img_segment_arr[img_segments]; /// array storing number of green pixels for each of the 3 segments of the photo
    int64_t floor_threshold_per_segment_arr[img_segments];

    // iterate over all possible headings from the image
    for (int i = 0; i < img_segments; i++)
    {
        // Determine the start and end of each of the possible heading sections
        uint16_t start_x = (uint16_t) roundf(i * img->h / (float) 3);
        uint16_t end_x = (uint16_t) roundf((i + 1) * img->h / (float) 3);
        
        cnt_green = 0;
        
        // Go through all the pixels
        for (uint32_t y = 0; y < img->w; y++)
        {
            for (uint16_t x = start_x; x < end_x; x++)
            {
                // Check if the color is inside the specified values
                uint8_t *yp, *up, *vp;
                if (x % 2 == 0) {
                    // Even xh
                    up = &buffer[y * 2 * img->h + 2 * x];      // U
                    yp = &buffer[y * 2 * img->h + 2 * x + 1];  // Y1
                    vp = &buffer[y * 2 * img->h + 2 * x + 2];  // V
                    //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
                } else {
                    // Uneven x
                    up = &buffer[y * 2 * img->h + 2 * x - 2];  // U
                    //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
                    vp = &buffer[y * 2 * img->h + 2 * x];      // V
                    yp = &buffer[y * 2 * img->h + 2 * x + 1];  // Y2
                }

                /// GREEN/FLOOR/
                // if pixel is green count it for the total per section and make it black; maybe make it black
                if ((*yp >= lum_min_floor) && (*yp <= lum_max_floor) &&
                    (*up >= cb_min_floor) && (*up <= cb_max_floor) &&
                    (*vp >= cr_min_floor) && (*vp <= cr_max_floor)) {
                    cnt_green++;
                    /// Comment in if you want to DRAW white over detected pixels
                    // tot_x += x;
                    // tot_y += y;
                    // if (draw) {
                    //    *yp = 0;  // make pixel dark in the image
                    //}
                }
                /// Compute the green threshold per section
                /// (NEW) WAY OF COMPUTING FLOOR THRESHOLD; adjust the orange_avoider_guided_threshold
                // define settings
                //float oag_color_count_frac = 0.18f;       // obstacle detection threshold as a fraction of total of image
                // floor_threshold_per_segment = 
                


                // TO DO ADDITIONAL SECTION
                // DETERMINE IF CURRENT MIDDLE SECTION = 2 HAS THE DESIRED THRESHOLD
                // IF YES PASS ON THAT HEADING + ASSOCIATED COLOR COUNT
                // IF NOT COMPARE WITH LEFT AND RIGHT
                // DECIDE IN WHICH SIDE YOU WANT TO SAMPLE FURTHER
                // PASS THE NEW HEADING + ASSOCIATED COLOR COUNT

                /// ORANGE
                // if pixel is orange count it for the total and make it white
                if ((*yp >= lum_min) && (*yp <= lum_max) &&
                    (*up >= cb_min) && (*up <= cb_max) &&
                    (*vp >= cr_min) && (*vp <= cr_max)) {
                    cnt++;
                    tot_x += x;
                    tot_y += y;
                    if (draw) {
                        *yp = 255;  // make pixel brighter in image
                    }
                }
            }
        }
        floor_threshold_per_segment_arr[i] = oag_floor_count_frac * img->w * (end_x - start_x);
        //Compute the green color count per each of the segments of the photo
        color_count_per_img_segment_arr[i] = cnt_green;

        // Compute centroid for orange detection
        if (cnt > 0) {
            *p_xc = (int32_t) roundf(tot_x / ((float) cnt) - img->w * 0.5f);
            *p_yc = (int32_t) roundf(img->h * 0.5f - tot_y / ((float) cnt));
        } else {
            *p_xc = 0;
            *p_yc = 0;
        }
    }

    // after you iterated over all image segments
 /// Check if the default heading(2 - middle) is still safe or we need to change it
    if (color_count_per_img_segment_arr[mid_segment] <= floor_threshold_per_segment_arr[mid_segment]) { // * (1.05f) && color_count_per_img_segment_arr[mid_segment] > floor_threshold_per_segment_arr[mid_segment] * (0.95f)) {
        maxValue = color_count_per_img_segment_arr[mid_segment];
        if (color_count_per_img_segment_arr[right_segment] > maxValue) { //&& color_count_per_img_segment_arr[right_segment] > floor_threshold_per_segment_arr[right_segment])  {
            // PRINT("[IF STAMENT FOR SAFE_HEADING=1]");
            //maxValue = color_count_per_img_segment_arr[0];
            *direction = 1; // change the index to center it around 0 with negative values = left; positive = right
            *floor_color_count_img_segment = color_count_per_img_segment_arr[right_segment];
        } else if (color_count_per_img_segment_arr[left_segment] > maxValue) { // && color_count_per_img_segment_arr[left_segment] > floor_threshold_per_segment_arr[left_segment]) { // && color_count_per_img_segment_arr[left_segment] > floor_threshold_per_segment_arr[left_segment] * 105 / 100) {
            //maxValue = color_count_per_img_segment_arr[0];
            *direction = -1; // change the index to center it around 0 with negative values = left; positive = right
            *floor_color_count_img_segment = color_count_per_img_segment_arr[left_segment];
        } else {
            *direction = 404; // return back (180 degrees)
            *floor_color_count_img_segment = color_count_per_img_segment_arr[mid_segment];
        }
    }
        // PRINT("TEST IN cv_detect_color_object");
        // PRINT("direction: %d  floor_color_count_img_segment: %d\n", direction, floor_color_count_img_segment)
        
    // else if (color_count_per_img_segment_arr[mid_segment] < floor_threshold_per_segment_arr[1]) {
    //   *direction = 404; // error turn back
    //   *floor_color_count_img_segment = color_count_per_img_segment_arr[mid_segment];
    // } 
    else {
      *direction = 0; // return the values just for the straight heading
      *floor_color_count_img_segment = color_count_per_img_segment_arr[mid_segment];
    }

    return cnt;

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
