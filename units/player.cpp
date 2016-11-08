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
#include "util.h"
#include "neuro.h"
#include "player.h"
#include "ants.h"

// Frame number for debug
extern int frame_index;

player::player(const char *filename)
{
    FILE *fd = fopen(filename, "r");
    if (!fd) {
        printf("Cant open %s\n", filename);
        exit(1);
    }
    bool more_to_read = true;
    int n;
    while (more_to_read) {
        int ch = fgetc(fd);
        if (ch == EOF)
            more_to_read = false;
        if (ch == '\n')
            n++;
    }
    pos = new struct rpos[n];
    rewind(fd);
    for (int i = 0; i < n; i++) {
        struct rpos *pr = &pos[i];
        fscanf(fd, "%d %d %d %d\n", 
        &pr->x, &pr->y, &pr->npix, &pr->frame);
    }
    cpos = 0;
    npos = n;
    done = false;
}

// Tries to predict the result of the pyrDown.
struct pix_tbl {
    int target;
    int len;
    int width;
};

struct pix_tbl pix_tbl_1[] = {
    { 1, 1, 1 },
    { 2, 1, 2 },
    { 3, 1, 3 },
    { 4, 2, 2 },
    { 6, 2, 3 },
    { 8, 2, 4 },
    { 9, 3, 3 },
    { 10, 2, 5 },
    { 12, 3, 4 },
    { 14, 2, 7 },
    { 15, 3, 5 },
    { 16, 4, 4 },
    { 18, 3, 6 },
    { 20, 4, 5 },
    { 21, 3, 7 },
    { 24, 4, 6 },
    {  0, 0, 0}
};

void interp(Mat &frame, struct rpos *pc, struct rpos *pn)
{
    double r = (double)(frame_index - pc->frame) /
               (double)(pn->frame - pc->frame);
    int px = round(r * (double)(pn->x - pc->x) + pc->x);
    int py = round(r * (double)(pn->y - pc->y) + pc->y);
    // Figure out ideal size - should match code in ant_score
    int ideal_size = get_ant_size(px, py);
    struct pix_tbl *pix_tbl;
    int scale = xpix / frame.cols;
    int width, len;
    if (scale == 1) {
        pix_tbl = pix_tbl_1;
    } else {
        DPRINTF("player.cpp: interp: No support for scale = %d!\n", scale);
        return;
    }
    for (int i = 0; pix_tbl[i+1].target > 0; i++) 
        if (ideal_size >= pix_tbl[i].target &&
            ideal_size < pix_tbl[i+1].target) {
            len = pix_tbl[i].len;
            width = pix_tbl[i].width;
            break;
        }
    px /= scale;
    py /= scale;
    DPRINTF("Play %d %d ideal_size: %d npix: %d frame: %d\n", px, py, ideal_size, width*len, frame_index);
    for (int i = 0; i < len; i++)
        for (int j = 0; j < width; j++)
            frame.at<uchar>(py + i, px + j) = 0;
}

void player::add_ant(Mat &frame)
{
    if (done)
        return;
    if (cpos == 0 && frame_index < pos[0].frame)
        return;
    struct rpos *pc = &pos[cpos];
    struct rpos *pn = &pos[cpos + 1];
    if (frame_index >= pc->frame && frame_index <= pn->frame)
        interp(frame, pc, pn);
    if (frame_index == pn->frame) {
        cpos++;
        if (cpos >= npos - 1)
            done = true;
    }
}
