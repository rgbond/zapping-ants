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

/*
 * Reads stdin for lines of the form
 * /media/rgb/USB20FD/ants.avi 0 870 680
 * <filename> <frame> <x> <y>
 * and writes out the ant snapshots
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
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

// constants
const uint32_t snap_size = 20;
const uint32_t search_size = snap_size + snap_size / 2;
const uint32_t ant_pixel = 100;

// options
bool verbose = false;
bool no_ants = false;

struct option {
    const char *opt;
    bool *vbl;
    const char *msg;
} opts[] = {
    { "-v", &verbose, "Verbose logging" },
    { "-n", &no_ants, "No ants" },
    { NULL, NULL, NULL }
};

#define DPRINTF if (verbose) printf

// Need something better than these globals
char *file_name;
uint32_t nframes;

void find_cg(Mat &src, Rect roi, int &xc, int &yc, int thresh)
{
    int xtot = 0;
    int ytot = 0;
    int npix = 0;
    for (int y = roi.y; y < roi.y + roi.height; y++) {
        for (int x = roi.x; x < roi.x + roi.width; x++) {
            if (src.at<uchar>(y, x) < thresh) {
                xtot += x;
                ytot += y;
                npix++;
            }
        }
    }
    if (npix > 0) {
        xc = xtot / npix;
        yc = ytot / npix;
    } else {
        xc = roi.x + roi.width / 2;
        yc = roi.y + roi.height / 2;
    }
}

int main(int argc, char* argv[])
{
	Mat frame;

    printf("snap\n");

    ++argv;
    while (--argc) {
        for (struct option *p = opts; p->opt; p++) {
            if (strcmp(*argv, p->opt) == 0) {
                *p->vbl = true;
                printf("%s\n", p->msg);
            }
        }
        argv++;
    }
    fflush(stdout);

    time_t secs = time(NULL);
    struct tm *ptm = localtime(&secs);

    VideoCapture cap;

    namedWindow("snap", CV_WINDOW_AUTOSIZE);

    char file_name[500];
    char cur_file_name[500];
    cur_file_name[0] = 0;
    uint32_t frame_index;
    uint32_t cur_frame_index;
    uint32_t nframes;
    uint32_t px;
    uint32_t py;
    uint32_t snap_half = snap_size / 2;
    uint32_t search_half = search_size / 2;
    uint32_t snap_num = 0;
    const char* tag;
    Mat tot = Mat(snap_size, snap_size, CV_32FC1, 0.0); 
    string dir;

    if (no_ants) {
        dir = "images/no_ants/";
        tag = "not_ant";
    } else {
        dir = "images/ants/";
        tag = "ant";
    }

    while (scanf("%s %u %u %u\n", file_name, &frame_index, &px, &py) == 4) {

        DPRINTF("%s %u %u %u\n", file_name, frame_index, px, py);

        if (no_ants)
            frame_index += 50;
        if (strcmp(file_name, cur_file_name) != 0) {
            cap.open(file_name);

            if (!cap.isOpened()) {
                printf("Can't open %s\n", file_name);
                exit(1);
            }
            nframes = (uint32_t)cap.get(CV_CAP_PROP_FRAME_COUNT);
            strcpy(cur_file_name, file_name);
            cur_frame_index = ~0;
        }
        if (frame_index > nframes) {
            printf("Invalid frame %u %u, skipping!\n", frame_index, nframes);
            continue;
        }
        if (frame_index != cur_frame_index) {
            cap.set(CV_CAP_PROP_POS_FRAMES, (double) frame_index);
            cap.read(frame);
            if (frame.empty()) {
                printf("Empty frame: %s %u %u %u\n", 
                       file_name, frame_index, px, py);
                exit(1);
            }
            cur_frame_index = frame_index;
        }
        if (px > frame.cols || py > frame.rows) {
            printf("Bad xy coord: %s %u %u %u\n", 
                   file_name, frame_index, px, py);
            exit(1);
        }

        int xc, yc;
        if (no_ants) {
            xc = px;
            yc = py;
        } else {
            // Try to center the ant
            if (search_half > px || search_half > py ||
                px + search_half > frame.cols || py + search_half > frame.rows) {
                    printf("Skipping %s %u %u %u\n", 
                           file_name, frame_index, px, py);
                    continue;
            }
            Rect search_roi(px - search_half, py - search_half, search_size, search_size);
            find_cg(frame, search_roi, xc, yc, ant_pixel);
        }

        if (snap_half > xc || snap_half > yc ||
            xc + snap_half > frame.cols || yc + snap_half > frame.rows) {
                printf("Skipping %s %u %u %u\n", 
                       file_name, frame_index, xc, yc);
            continue;
        }

        Rect roi(xc - snap_half, yc - snap_half, snap_size, snap_size);
        Mat snap(frame, roi);
        if (verbose) {
            imshow("snap", snap);
            waitKey(3000);
        }
        char image_name[500];
        sprintf(image_name, "%s_%04d%02d%02d%02d%02d_%04u.png", tag,
                             ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday,
                             ptm->tm_hour, ptm->tm_min, snap_num++);
        imwrite(dir + string(image_name), snap);
        printf("%s %u %u %u %s\n",
                file_name, frame_index, px, py,
                image_name);
        Mat tmp;
        snap.convertTo(tmp, CV_32FC1);
        tot += tmp;
    }

    cout << tot;
    tot /= (float)snap_num * 255.0;
    cout << tot;
    imshow("snap", tot);
    waitKey(0);

    exit(0);
}
