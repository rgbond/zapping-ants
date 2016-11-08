/* 
 * Copyright 2016 Robert Bond
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <iostream>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "opencv2/core/core.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;
using namespace cv::gpu;

#include "hw.h"
#include "neuro.h"
#include "util.h"
#include "blobs.h"
#include "ants.h"

// globals from units.cpp
extern int frame_index;
extern double average_frame_time;
extern uint64_t frame_ticks;
extern double tps;
extern running_average laser_frame_lag;

// options
extern bool neural_class;
extern bool show_mog;
extern bool take_snapshots;
extern bool verbose;

// Sizes of ants in sq pixels
int min_ant_size;
int max_ant_size;
uint16_t ant_pix[PIX_TBL_WIDTH][PIX_TBL_HEIGHT];
const int ant_thresh = 100;

// Finds a score for this blob
void ants::ant_score(struct rec_list *pn)
{
    if (neural_class) {
        std::vector<float> image_type;
        image_type = pclass->get_image_type(pframe, Point(pn->xc, pn->yc));
        pn->score = (int)round(image_type[ant_index] * 15.0);
        DPRINTF("neural ant_score: %d %d %d\n", pn->xc, pn->yc, pn->score);
    } else {
        int scale = xpix / pfg->cols;
        uint64_t color_tot = 0; 
        assert(pfg->cols == pframe->cols);
        pn->score = 0;

        // Ant sizes vary a lot by distance from camera
        int ideal_count = get_ant_size(pn->xc, pn->yc) / (scale * scale);
        int range = ideal_count / 2;
        if (range == 0)
            range = 1;
        int max = ideal_count + range;
        int min = ideal_count - range;
        if (min < 3)
            min = 3;

        if (pn->npix < min) {
            DPRINTF("%4d %4d %3d %3d too small\n", pn->xc, pn->yc, pn->npix, min);
            return;
        }

        if (pn->npix > max) {
            DPRINTF("%4d %4d %3d %3d too big\n", pn->xc, pn->yc, pn->npix, max);
            return;
        }

        DPRINTF("ant_score: processing %d %d %d\n", pn->xc, pn->yc, pn->npix);

        // Blob size
        pn->score += 5;
        // Blob aspect ratio
        double ratio = (double)pn->rect.width / (double)pn->rect.height;
        if (ratio < 1.0)
            ratio = 1.0 / ratio;
        if (ratio < ant_len * 1.1 / ant_width)
            pn->score += 4;
        // Ant color
        int ystart = pn->rect.y/scale;
        int yend = ystart + pn->rect.height/scale;
        int xstart = pn->rect.x/scale;
        int xend = xstart + pn->rect.width/scale;
        int cc = 0;

        for (int y = ystart; y < yend; y++) {
            for (int x = xstart; x < xend; x++) {
                if (pfg->at<uchar>(y, x) == ant_thresh &&
                    pframe->at<uchar>(y, x) < ant_color) {
                        cc++;
                }
                if (pfg->at<uchar>(y, x) == ant_thresh) {
                    DPRINTF("%3d ", pframe->at<uchar>(y, x));
                } else {
                    DPRINTF("999 ");
                }
            }
            DPRINTF("\n");
        }

        // Close black pixel counts are a good indicator
        range = ideal_count / 8;
        max = ideal_count + range;
        min = ideal_count - range;
        if (cc >= min && cc <= max)
            pn->score += 10;

        DPRINTF("ant_score: ideal: %d min: %d max: %d cc: %d\n",
               ideal_count, min, max, cc);
    }
}

void ants::score_ants(struct rec_list *precs)
{
    if (!precs)
        return;
    ant_score(precs);
    score_ants(precs->pnext);
}

ants::ants(hw *phw, Mat *pframe, Mat *pfg, Mat *phalf_fg,
           snapshots *psnap, image_classifier *pclass)
{
    this->phw = phw;
    this->pframe = pframe;
    this->pfg = pfg;
    this->phalf_fg = phalf_fg;
    this->psnap = psnap;
    this->pclass = pclass;
    pants = NULL;

    // Set up pixel size table
    min_ant_size = 1000;
    max_ant_size = 0;
    for (int px = 0; px < PIX_TBL_WIDTH; px++) 
        for (int py = 0; py < PIX_TBL_HEIGHT; py++) {
            double pixels_per_mm = 1.0/phw->mm_per_pixel(px*PTG , py*PTG);
            double ant_sq_pix = ant_len * pixels_per_mm;
            ant_sq_pix *= ant_width * pixels_per_mm;
            assert(ant_sq_pix + 0.5 < 65536.0);
            uint16_t value = (uint16_t)(ant_sq_pix + 0.5);
            ant_pix[px][py] = value;
            if (value > max_ant_size)
                max_ant_size = value;
            if (value < min_ant_size)
                min_ant_size = value;
            // DPRINTF("ant_pix[%d, %d]: %u\n", px, py, value);
    }
    DPRINTF("min_ant_size: %u max_ant_size: %u\n", min_ant_size, max_ant_size);
}

struct ant_list* ants::all_ants(void)
{
    return pants;
}

void ants::predict_next_pos(struct ant_list *pant, int *px, int *py)
{
    Point pred;

    double lag = laser_frame_lag.average();
    double aspeed = pant->avg_speed.average();
    Point2d uv = pant->uv.average();
    if (aspeed > 0.1) {
        DPRINTF("id %d predict_next_pos 0 %d %d\n", pant->id, pant->last.x, pant->last.y);
        // 1st guess is a minimum count of frames based on the state machine
        pred.x = pant->last.x + uv.x * aspeed * lag * average_frame_time;
        pred.y = pant->last.y + uv.y * aspeed * lag * average_frame_time;
        DPRINTF("id %d predict_next_pos 1 %d %d lag: %5.2lf\n",
                 pant->id, pred.x, pred.y, lag);
        assert(pred.x + pred.y != 0);
        // Next guess adds how far the ant moves while the mirrors
        // move rounded up to a full frame time 
        double dt = phw->move_time(pred.x, pred.y);
        if (dt > 0.0) {
            double move_frames = trunc(dt/average_frame_time + 0.9);
            pred.x += uv.x * aspeed * move_frames * average_frame_time;
            pred.y += uv.y * aspeed * move_frames * average_frame_time;
            DPRINTF("id %d predict_next_pos 2 %d %d move: %5.2lf\n", 
                    pant->id, pred.x, pred.y, move_frames);
        }
        if (!Rect(0, 0, xpix, ypix).contains(pred)) {
            pred.x = pant->last.x;
            pred.y = pant->last.y;
        }
    } else {
        pred.x = pant->last.x;
        pred.y = pant->last.y;
    }
    *px = pred.x;
    *py = pred.y;
}

void ants::match_blobs_to_ant(struct rec_list *precs, struct ant_list *pant)
{
    struct rec_list *pn;
    struct rec_list *candidate = NULL;
    double closest = 1000000.0;
    for (pn = precs; pn; pn = pn->pnext) {
        if (pn->score == 0)
            continue;
        Point w(pn->xc - pant->pred.x, pn->yc - pant->pred.y);
        double dist = sqrt((double)(w.x * w.x + w.y * w.y));
        if (dist > close_blob)
            continue;
        if (dist < closest) {
            closest = dist;
            candidate = pn;
        }
    }
    if (candidate) {
        candidate->claimed = pant;
    } else {
        // For plot_predicitons
        pant->pred = Point(0, 0);
    }
}

void ants::match_blobs_to_ants(struct rec_list *precs, struct ant_list *pants)
{
    struct ant_list *pant;
    for (pant = pants; pant; pant = pant->next) {
        double aspeed = pant->avg_speed.average();
        Point2d uv = pant->uv.average();
        int frames = frame_index - pant->last_frame;
        pant->pred.x = pant->last.x + uv.x * aspeed * frames * average_frame_time;
        pant->pred.y = pant->last.y + uv.y * aspeed * frames * average_frame_time;
        match_blobs_to_ant(precs, pant);
    }
}

void ants::process_ant(struct rec_list *pn)
{
    Point v;
    struct ant_list *pant = pn->claimed;
    v.x = pn->xc - pant->last.x;
    v.y = pn->yc - pant->last.y;
    double dist = sqrt((double)(v.x * v.x + v.y * v.y));
    if (dist != 0.0)
        pant->uv.add_item(Point2d((double)v.x / dist, (double)v.y / dist));
    double dt = (double)(frame_ticks - pant->last_frame_ticks)/tps;
    pant->avg_speed.add_item(dist / dt);
    pant->last_frame_ticks = frame_ticks;
    pant->score += pn->score;
    if (pant->score > max_score)
        pant->score = max_score;
    DPRINTF("Find ant id %d at %d %d now %d %d score %d speed %6.2lf frame %d\n",
           pant->id, pant->last.x, pant->last.y, 
           pn->xc, pn->yc,
           pant->score, pant->avg_speed.average(), frame_index);
    pant->last.x = pn->xc;
    pant->last.y = pn->yc;
    int ldx = pant->last.x - phw->cur_loc.px;
    int ldy = pant->last.y - phw->cur_loc.py;
    pant->laser_dist = sqrt((double)(ldx * ldx + ldy * ldy));
    pant->last_frame = frame_index;
    if (take_snapshots)
        psnap->snap_ant(pant->last);
}

void ants::add_ant(struct rec_list *pn)
{
    struct ant_list *pnew = new ant_list;
    static int next_id = 1;
    pnew->id = next_id++;
    pnew->score = pn->score;
    pnew->last.x = pn->xc;
    pnew->last.y = pn->yc;
    pnew->uv = direction_average(5);
    pnew->avg_speed = running_average(10);
    pnew->last_frame = frame_index;
    pnew->last_frame_ticks = frame_ticks;
    pnew->total_distance = 0.0;
    pnew->next = pants;
    pnew->pred = Point(0, 0);
    pants = pnew;
    if (take_snapshots)
        psnap->snap_ant(pnew->last);
    DPRINTF("New ant id %d at %d %d score %d frame %d\n",
            pnew->id, pnew->last.x, pnew->last.y, pn->score, frame_index);
}

struct ant_list *ants::delete_dead_ants(struct ant_list *pant)
{
    if (!pant)
        return NULL;
    struct ant_list *nxt = pant->next;
    pant->score -= 1;
    if (pant->score <= 0) {
        DPRINTF("Dead ant id %d at %d %d frame %d\n",
               pant->id, pant->last.x, pant->last.y, frame_index);
        delete pant;
        return delete_dead_ants(nxt);
    }
    pant->next = delete_dead_ants(nxt);
    return pant;
}

struct ant_list *ants::pick_best_ant(struct ant_list *pant)
{
    if (!pant)
        return NULL;
    struct ant_list *best_of_rest = pick_best_ant(pant->next);
    if (pant->score <= 25)
        return best_of_rest;
    if (frame_index - pant->last_frame > 3)
        return best_of_rest;
    if (!best_of_rest || pant->laser_dist < best_of_rest->laser_dist)
        return pant;
    return best_of_rest;
}

void draw_each_ant(Mat *phalf_fg, struct ant_list *pant)
{
    if (!pant)
        return;
    int scale = xpix / phalf_fg->cols;
    Point bottom, top;
    bottom.x = pant->last.x / scale;
    bottom.y = pant->last.y / scale;
    top.x = bottom.x;
    top.y = bottom.y - pant->score;
    line(*phalf_fg, bottom, top, Scalar(255, 255, 255));
    draw_each_ant(phalf_fg, pant->next);
}

void ants::draw_ants()
{
    draw_each_ant(phalf_fg, pants);
}


void plot_each_prediction(Mat &half, struct ant_list *pant)
{
    if (!pant)
        return;
    if (pant->pred.x + pant->pred.y != 0) {
        int scale = xpix / half.cols;
        int radius = close_blob/scale;
        Scalar s(255, 255, 255);
        Point spred(pant->pred.x/scale, pant->pred.y/scale);
        circle(half, spred, radius, s, 1);
        Point sant(pant->last.x/scale, pant->last.y/scale);
        line(half, sant, spred, s);
    }
    plot_each_prediction(half, pant->next);
}

void ants::plot_predictions(Mat &half)
{
    plot_each_prediction(half, pants);
}

struct ant_list* ants::select_ant()
{
    // Find blobs in fg
    struct rec_list *precs = find_bbb(*pfg, Rect(0, 0, pfg->cols, pfg->rows), phw, ant_thresh);
    // See if they look like ants
    score_ants(precs);
    // Match up the ones that look like ants
    match_blobs_to_ants(precs, pants);
    // Process ants and make new ones.
    while(precs) {
        struct rec_list *pn = precs;
        if (pn->score > 0) {
            if (pn->claimed)
                process_ant(pn);
            else
                add_ant(pn);
        }

        precs = precs->pnext;
        delete pn;
    }

    // Clean up dead ants
    pants = delete_dead_ants(pants);
    
    // Pick an ant from ant_list
    struct ant_list *best_ant = pick_best_ant(pants);

    // And return it if really good
    if (best_ant && best_ant->score > 25) {
        DPRINTF("Best ant id %d at %d %d score %d frame %d\n", 
               best_ant->id, best_ant->last.x, best_ant->last.y,
               best_ant->score, frame_index);
        return best_ant;
    }
    return NULL;
}
