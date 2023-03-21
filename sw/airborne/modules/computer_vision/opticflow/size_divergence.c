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

// TODO: test this function, and send an ABI message to the Navigator to make decision based on the divergence difference
float get_difference_divergence(struct flow_t *vectors, int count, int n_samples)
{
    // computes the difference between the normalised divergence on the left and right sides of the image
    // the two normalisations are:
    // 1. normalise for the linear expansion of optic flow vectors situated further from the center of the image
    // 2. normalise for the amount of optic flow vectors on each half of the image

    float flow_norm;
    float coeff_norm;                // normalisation coefficient

    float divs_sum_left = 0.f;       // Divergence in left part of image
    float divs_sum_left_mean = 0.f;  // Mean divergence in left part of image
    float divs_sum_right = 0.f;      // Divergence in right part of image
    float divs_sum_right_mean = 0.f; // Mean divergence in right part of image
    float divs_sum_difference = 0.f; // Difference in divergence used to determine which side has the larger divergence
    uint32_t used_samples = 0;
    uint32_t used_samples_left = 0;
    uint32_t used_samples_right = 0;
    float dx, dy;
    int32_t i;
    int32_t image_width_half = (front_camera.output_size.h/2) * 100; // Width of captured image (maybe needs a header file)
    int32_t image_height_half = (front_camera.output_size.w/2) * 100;
    PRINT("image_height_half: %d; image_width_half: %d \n", image_height_half, image_width_half);

    // apply the random consensus method if n_samples != 0
    // TODO: apply random consensus method
    for (i = 0; i < count; i++) {
        // distance in previous image:
        dx = (float)vectors[i].flow_x;
        dy = (float)vectors[i].flow_y;

        PRINT("dx: %f; dy: %f \n", dx, dy);

        // this is the linear normalisation coefficient
        coeff_norm = sqrtf(
                ((float) vectors[i].pos.x - image_width_half) * ((float) vectors[i].pos.x - image_width_half) +
                ((float) vectors[i].pos.y - image_height_half) * ((float) vectors[i].pos.y - image_height_half));

        PRINT("coeff_norm: %f \n", coeff_norm);

        // compute the norm of the flow vector and normalise it (normalisation 1)
        flow_norm = sqrtf(dx * dx + dy * dy) / coeff_norm;
        PRINT("flow_norm: %f \n", flow_norm);

        PRINT("pos_x: %d; image_width_half: %d \n", vectors[i].pos.x, image_width_half);
        // decide whether the optic flow vector is on the left or on the right
        if ((float) vectors[i].pos.x < image_width_half) {
            divs_sum_left += flow_norm; // Left part of image considered
            used_samples_left++;
        } else {
            divs_sum_right += flow_norm; // Right part of image considered
            used_samples_right++;
        }
        used_samples++;
        PRINT("used_samples_left: %d; used_samples_right: %d \n", used_samples_left, used_samples_right);
        PRINT("used_samples: %d \n", used_samples);
    }

    if (used_samples_left < 1 || used_samples_right < 1){
        return 0.f;
    }

    // normalise the two divergences with the number of optic flow vectors in the corresponding part of the image
    divs_sum_left_mean = divs_sum_left / used_samples_left;
    divs_sum_right_mean = divs_sum_right / used_samples_right;
    PRINT("divs_sum_left_mean: %f \n", divs_sum_left_mean);
    PRINT("divs_sum_right_mean: %f \n", divs_sum_right_mean);

    divs_sum_difference = divs_sum_left_mean - divs_sum_right_mean;
    PRINT("div_diff difference : %f \n", divs_sum_difference);
    return divs_sum_difference;
}
