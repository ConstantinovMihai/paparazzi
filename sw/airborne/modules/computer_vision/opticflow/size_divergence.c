/*
 * Copyright (C) 2015 Guido de Croon <guido.de.croon@gmail.com>
 *
 * From:
 * Characterization of Flow Field Divergence for Vertical Landing Control of MAVs
 * by H.W. Ho and G.C.H.E. de Croon (submitted)
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

/**
 * @file modules/computer_vision/opticflow/size_divergence.c
 * @brief Calculate divergence from flow vectors by looking at line sizes between the points.
 *
 * Uses optical flow vectors as determined with a corner tracker and Lucas Kanade to estimate divergence.
 */

#include "firmwares/rotorcraft/navigation.h"
#include "size_divergence.h"
#include <stdlib.h>
#define PRINT(string,...) fprintf(stderr, "[size_divergence->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

float moving_average_filter_div_diff(float current_value, float old_value);
bool vectorInROI(uint32_t vectorPosX, uint32_t vectorPosY, int32_t x1, int32_t x2, int32_t y1, int32_t y2);

/**
 * Filter all optic flow vectors by their position in a specific region of interest.
 * @param[in] original_vectors    The optical flow vectors
 * @param[in] count      The number of optical flow vectors 
 * @return filtered optic flow vectors
*/

bool vectorInROI(uint32_t vectorPosX, uint32_t vectorPosY, int32_t x1, int32_t x2, int32_t y1, int32_t y2) {
    
    if (vectorPosX >= (uint32_t)x1 && vectorPosX <= (uint32_t)x2 && vectorPosY >= (uint32_t)y1 && vectorPosY <= (uint32_t)y2) {
        return true;
    } else {
        return false;
    }
}

int filter_vectors(struct flow_t *original_vectors, int count, struct flow_t *filtered_vectors)
{
    // Define factors for cropping in the horizontal and vertical axes
    float h_factor = 0.3;
    float w_factor = 0.5;
    float scale = 100;

    // Get all points of cropped image in original bounds -> Region of Interest
    // Horizontal Region of Interest
    int32_t top_left_horizontal = (0.5f * front_camera.output_size.h * (1.f - w_factor)) * scale;
    int32_t top_right_horizontal = top_left_horizontal + (w_factor*front_camera.output_size.h * scale);

    // Vertical Region of Interest
    int32_t top_left_vertical = (0.5f * front_camera.output_size.w * (1.f - h_factor)) * scale;
    int32_t bottom_left_vertical = top_left_vertical + (h_factor*front_camera.output_size.w * scale);

    int32_t i;
    int32_t filteredCount = 0;
    struct flow_t tV; // tempVector

    // Go through all vectors and keep count of how many remain after filtering
    for (i = 0; i < count; i++) {
        // Get vector i
        tV = original_vectors[i];
        
        // Check if vector is in region of interest
        if (vectorInROI(tV.pos.x, tV.pos.y, top_left_horizontal, top_right_horizontal, 
            top_left_vertical, bottom_left_vertical)) 
        {   
            filtered_vectors[filteredCount] = tV;
            filteredCount++;
        }
    }
    return 0;
}

/**
 * Get divergence from optical flow vectors based on line sizes between corners
 * @param[in] vectors    The optical flow vectors
 * @param[in] count      The number of optical flow vectors
 * @param[in] n_samples  The number of line segments that will be taken into account. 0 means all line segments will be considered.
 * @return divergence
 */

float get_size_divergence(struct flow_t *vectors, int count, int n_samples)
{
    float distance_1, distance_2;
    float divs_sum = 0.f;
    uint32_t used_samples = 0;
    float dx, dy;
    int32_t i, j;

    int32_t max_samples = (count * count - count) / 2;

    if (count < 2) {
        return 0.f;
    } else if (count >= max_samples) {
        n_samples = 0;
    }

    if (n_samples == 0) {
        // go through all possible lines:
        for (i = 0; i < count; i++) {
            for (j = i + 1; j < count; j++) {
                // distance in previous image:
                dx = (float)vectors[i].pos.x - (float)vectors[j].pos.x;
                dy = (float)vectors[i].pos.y - (float)vectors[j].pos.y;
                distance_1 = sqrtf(dx * dx + dy * dy);

                if (distance_1 < 1E-5) {
                    continue;
                }

                // distance in current image:
                dx = (float)vectors[i].pos.x + (float)vectors[i].flow_x - (float)vectors[j].pos.x - (float)vectors[j].flow_x;
                dy = (float)vectors[i].pos.y + (float)vectors[i].flow_y - (float)vectors[j].pos.y - (float)vectors[j].flow_y;
                distance_2 = sqrtf(dx * dx + dy * dy);

                divs_sum += (distance_2 - distance_1) / distance_1;
                used_samples++;
            }
        }
    } else {
        // take random samples:
        for (uint16_t sample = 0; sample < n_samples; sample++) {
            // take two random indices:
            i = rand() % count;
            j = rand() % count;
            // ensure it is not the same index:
            while (i == j) {
                j = rand() % count;
            }

            // distance in previous image:
            dx = (float)vectors[i].pos.x - (float)vectors[j].pos.x;
            dy = (float)vectors[i].pos.y - (float)vectors[j].pos.y;
            distance_1 = sqrtf(dx * dx + dy * dy);

            if (distance_1 < 1E-5) {
                continue;
            }

            // distance in current image:
            dx = (float)vectors[i].pos.x + (float)vectors[i].flow_x - (float)vectors[j].pos.x - (float)vectors[j].flow_x;
            dy = (float)vectors[i].pos.y + (float)vectors[i].flow_y - (float)vectors[j].pos.y - (float)vectors[j].flow_y;
            distance_2 = sqrtf(dx * dx + dy * dy);

            divs_sum += (distance_2 - distance_1) / distance_1;
            used_samples++;
        }
    }

    if (used_samples < 1){
        return 0.f;
    }

    // return the calculated mean divergence:
    return divs_sum / used_samples;
}

float moving_average_filter_div_diff(float current_value, float old_value)
{
    return ((0.4*current_value) + (1-0.4)*old_value);
}

// Keep track of the old divergence difference for the moving average filter
float divs_sum_difference_old = 0.f;
bool F_FIRST_TIME_MOVING_FILTER = true;

double get_difference_divergence(struct flow_t *vectors, int count, int n_samples)
{
    // computes the difference between the normalised divergence on the left and right sides of the image
    // the two normalisations are:
    // 1. normalise for the linear expansion of optic flow vectors situated further from the center of the image
    // 2. normalise for the amount of optic flow vectors on each half of the image

    double flow_norm;
    double coeff_norm;                // normalisation coefficient

    double divs_sum_left = 0.f;       // Divergence in left part of image
    double divs_sum_left_mean = 0.f;  // Mean divergence in left part of image
    double divs_sum_right = 0.f;      // Divergence in right part of image
    double divs_sum_right_mean = 0.f; // Mean divergence in right part of image
    double divs_sum_difference = 0.f; // Difference in divergence used to determine which side has the larger divergence
    uint32_t used_samples = 0;
    uint32_t used_samples_left = 0;
    uint32_t used_samples_right = 0;
    double dx, dy;
    int32_t i;
    int32_t image_width_half = (front_camera.output_size.h/2) * 100; // Width of captured image (maybe needs a header file)
    int32_t image_height_half = (front_camera.output_size.w/2) * 100;

    // apply the random consensus method if n_samples != 0
    // TODO: apply random consensus method
    for (i = 0; i < count; i++) {
        // Distance in previous image:
        dx = (double)vectors[i].flow_x;
        dy = (double)vectors[i].flow_y;

        // this is the linear normalisation coefficient
        coeff_norm = sqrtf(
                ((double) vectors[i].pos.x - image_width_half) * ((double) vectors[i].pos.x - image_width_half) +
                ((double) vectors[i].pos.y - image_height_half) * ((double) vectors[i].pos.y - image_height_half));

        // Compute the norm of the flow vector and normalise it (normalisation 1)
        flow_norm = sqrtf(dx * dx + dy * dy) / coeff_norm;

        // Decide whether the optic flow vector is on the left or on the right
        if ((double) vectors[i].pos.x < image_width_half) {
            divs_sum_left += flow_norm; // Left part of image considered
            used_samples_left++;
        } else {
            divs_sum_right += flow_norm; // Right part of image considered
            used_samples_right++;
        }
        used_samples++;
    }

    if (used_samples_left < 1 || used_samples_right < 1){
        return 0.f;
    }

    // normalise the two divergences with the number of optic flow vectors in the corresponding part of the image
    divs_sum_left_mean = divs_sum_left / used_samples_left;
    divs_sum_right_mean = divs_sum_right / used_samples_right;

    divs_sum_difference = divs_sum_left_mean - divs_sum_right_mean;

    // PRINT("divs_sum_left_mean: %f; divs_sum_right_mean: %f; divs_sum_difference: %f", divs_sum_left_mean, divs_sum_right_mean, divs_sum_difference);

    // If this is the first time the function is called, set the old value to the new value
    if (F_FIRST_TIME_MOVING_FILTER) {
        divs_sum_difference_old = divs_sum_difference;
        F_FIRST_TIME_MOVING_FILTER = false;
    }

    // Average out divs_sum_difference values using a moving average filter, x is new value, y is old value
    divs_sum_difference = moving_average_filter_div_diff(divs_sum_difference, divs_sum_difference_old);
    divs_sum_difference_old = divs_sum_difference;


    return divs_sum_difference;
}
