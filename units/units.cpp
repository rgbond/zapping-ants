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
#include "ants.h"
#include "blobs.h"
#include "player.h"

// options
bool accurate = false;
bool alternate_frame = false;
bool dont_correct = false;
bool draw_laser = false;
bool fake_laser = false;
bool overlay_laser = false;
bool movie = false;
bool no_ants = false;
bool neural_class = false;
bool play_ants = false;
bool plot_predictions = false;
bool random_moves = false;
bool show_mog = false;
bool take_snapshots = false;
bool sql_backlash = false;
bool verbose = false;

struct option {
    const char *opt;
    bool *vbl;
    const char *msg;
} opts[] = {
    { "-a", &alternate_frame, "Alternate frame display enabled" },
    { "-c", &accurate, "Repeat corrections until loop closed" },
    { "-d", &dont_correct, "Don't do closed loop corrections" },
    { "-f", &fake_laser, "Fake the laser coms" },
    { "-l", &draw_laser, "Draw the laser on the screen" },
    { "-O", &overlay_laser, "Overlay the laser on a movie" },
    { "-o", &show_mog, "Show mog window enabled" },
    { "-m", &movie, "Use /media/rgb/6633-6433/ants.avi as source" },
    { "-n", &no_ants, "No ants" },
    { "-N", &neural_class, "Use neural network to classify images" },
    { "-p", &play_ants, "Replay ants from recorded positions" },
    { "-P", &plot_predictions, "Plot predictions for ant movement" },
    { "-r", &random_moves, "Do random moves" },
    { "-s", &sql_backlash, "Save sql formatted backlash data" },
    { "-S", &take_snapshots, "Take snapshots of the ants and laser" },
    { "-v", &verbose, "Verbose logging" },
    { NULL, NULL, NULL }
};

// Need something better than these globals
int frame_index = 0;
uint64_t frame_ticks = 0;
double total_frame_time = 0;
double average_frame_time = 0;
double tps;
backlash *pbl;
hw *phw;
laser *plas;
ants *pan;
player *play;
snapshots *psnap;
image_classifier *pclass;
bool mouse_click;
running_average laser_frame_lag(10);

static void onMouse(int event, int px, int py, int flags, void* userdata)
{
    if (event != EVENT_LBUTTONDOWN)
        return;
    // Assume we clicked on a 1/2 size window
    px = px * 2;
    py = py * 2;
    if (!phw->keepout(px, py, 1)) {
        phw->do_move(px, py, frame_index, "onMouse");
        mouse_click = true;
    } else {
        DPRINTF("In keepout: %d %d\n", px, py);
    }
}

inline double normal(double mean, double sigma)
{
    double u1 = (double)rand()/(double)RAND_MAX;
    double u2 = (double)rand()/(double)RAND_MAX;
    return sqrt(-2.0 * log(u1)*sigma*sigma)*cos(2.0 * M_PI * u2) + mean;
}

void move_randomly(bool *pdone)
{
    static int count = 400;

    if (count == 0) {
        count = -1;
        phw->do_move(xpix/2, ypix/2, frame_index, "Home");
        return;
    } else if (count == -1) {
        *pdone = true;
        return;
    }

    --count;
    int px = round(normal(640, 100));
    int py = round(normal(480, 75));
    px = px > 100 ? px : 100;
    px = px < 1180 ? px : 1180;
    py = py > 100 ? py : 100;
    py = py < 860 ? py : 860;
    phw->do_move(px, py, frame_index, "random move");
}


bool ant_looker(bool do_move)
{
    struct rec_list *precs;
    int px, py;
    bool retval = false;
    struct ant_list *best_ant;

    if (no_ants)
        return false;

    best_ant = pan->select_ant();

    if (do_move && best_ant != NULL) {
        pan->predict_next_pos(best_ant, &px, &py);
        DPRINTF("ant_looker: %4d %4d frame: %d\n", px, py, frame_index);
        phw->do_move(px, py, frame_index, "  ant");
        retval = true;
    }

    return retval;
}

// Laser blobs are full of bright pixels.
#define LT 250
inline bool laser_blob(const struct rec_list *pn, Mat &frame, Mat &fg)
{
    if (neural_class) {
        std::vector<float> image_type;
        image_type = pclass->get_image_type(&frame, Point(pn->xc, pn->yc));
        return image_type[laser_index] > 0.9f;
    } else {
        int lcount = 0;
        // cout << "laser_blob looking at " << pn->rect << endl;
        for (int y = pn->rect.y; y < pn->rect.y + pn->rect.height; y++) {
            for (int x = pn->rect.x; x < pn->rect.x + pn->rect.width; x++) {
                if (fg.at<uchar>(y, x) == LT && frame.at<uchar>(y, x) > LT)
                    lcount++;
                DPRINTF("%3d ", frame.at<uchar>(y, x));
            }
            DPRINTF("\n");
        }
        DPRINTF("laser_blob counted %d\n", lcount);
        return lcount > 60;
    }
}

bool find_laser(Mat &frame, Mat& fg, int xc, int yc, int size, Point &center, Rect &r)
{
    int half_size = size/2;
    int xs = std::max(xc - half_size, 0);
    int ys = std::max(yc - half_size, 0);
    int xe = std::min(xc + half_size, fg.cols);
    int ye = std::min(yc + half_size, fg.rows);

    Rect roi(Point(xs, ys), Point(xe, ye));

    struct rec_list *blobs = find_bbb(fg, roi, phw, 250); 

    // Check to see which blobs might be the laser
    bool got_laser = false;
    while(blobs) {
        struct rec_list *pn = blobs;

        if (got_laser == false && pn->npix > 80) {
                if (laser_blob(pn, frame, fg)) {
                    center.x = pn->xc;
                    center.y = pn->yc;
                    r = pn->rect;
                    got_laser = true;
                    if (take_snapshots)
                        psnap->snap_laser(center);
                    DPRINTF("Found laser at %d, %d, npix = %d\n", 
                            center.x, center.y, pn->npix);
            }
        }

        blobs = blobs->pnext;
        delete pn;
    }

    return got_laser;
}

// Returns true if moved laser
bool correct(Mat& frame, Point &center, Rect &box)
{
    bool moved_laser = false;

    if (dont_correct)
        return false;

    phw->pxy_to_loc(center.x, center.y, &phw->cur_loc);
    int tx = phw->target.px;
    int ty = phw->target.py;
    int dx = tx - center.x;
    int dy = ty - center.y;
    double dist = sqrt((double)(dx*dx) + (double)(dy*dy));
    if (dist > 3.0 && !box.contains(Point(tx, ty))) {
        DPRINTF("Correct: %d, %d target: %d %d frame: %d\n",
               center.x, center.y, tx, ty, frame_index);
        phw->do_correction(tx, ty, frame_index, "  correct");
        moved_laser = true;
    } else {
        pbl->stop(phw);
        pbl->dumpit();
    }

    return moved_laser;
}

inline int border(int i)
{
    double x = i * 2;
    return (int)round(0.000362 * x * x  - 0.511 * x + 220.732)/2;
}

VIBE_GPU vibe;

// Main pixel porcessing
void process_frame(VideoCapture &ccap, VideoCapture &mcap,
                   Mat &frame, Mat &fg, Mat &half, Mat &half_fg)
{


    if (movie)
        mcap.read(frame);
    else
        ccap.read(frame);

    if (frame.empty()) {
        printf("Can't read a movie frame!\n");
        exit(1);
    }

    if (draw_laser)
        plas->draw_laser(frame);

    if (play_ants)
        play->add_ant(frame);

    // Load up the frames
    GpuMat d_frame(frame);
    GpuMat d_tmp;
    if (movie && overlay_laser) {
        Mat overlay;
        ccap.read(overlay);
        if (overlay.empty()) {
            printf("Can't read an overlay frame!\n");
            exit(1);
        }
        GpuMat d_overlay(overlay);
        gpu::threshold(d_overlay, d_overlay, 250.0, 255.0, THRESH_BINARY); 
        gpu::bitwise_or(d_overlay, d_frame, d_frame);
        d_frame.download(frame);
    }

    // Process fg/bg
    GpuMat d_fg;
    vibe.operator()(d_frame, d_fg);
    if (countNonZero(d_fg, d_tmp) > 1000) {
        // The bg processing blew up...
        printf("Background frame reset!\n");
        vibe.initialize(d_frame);
        d_fg = Scalar(0); // No pixels this frame
    }

    d_fg.download(fg);

    // Make a displayable frame
    if (verbose) {
        GpuMat d_half;
        gpu::pyrDown(d_frame, d_half);
        d_half.download(half);
    }

    // Setup mog display
    if (verbose && show_mog) {
        GpuMat d_half_fg;
        gpu::pyrDown(d_fg, d_half_fg);
        d_half_fg.download(half_fg);
    }

}
   
int main(int argc, char* argv[])
{
	int64 st;

	Mat frame;
    Mat fg;
    Mat half;
    Mat half_fg;
    uint32_t frame_count = 1000000; 

    printf("units\n");

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

    pbl = new backlash();
    phw = new hw(pbl);
    plas = new laser(phw, false);
    pclass = new image_classifier();
    if (take_snapshots)
        psnap = new snapshots(&frame);
    pan = new ants(phw, &frame, &fg, &half_fg, psnap, pclass);
    if (play_ants)
        play = new player("ants.pos"); 

    // The camera video file for reading
    VideoCapture ccap;
    // The movie video file
    VideoCapture mcap;

    if (verbose) {
        namedWindow("Units", CV_WINDOW_AUTOSIZE);
        setMouseCallback("Units", onMouse, NULL);
    }

    if (alternate_frame)
        namedWindow("laser", CV_WINDOW_AUTOSIZE);

    if (show_mog)
        namedWindow("mog", CV_WINDOW_AUTOSIZE);

    if (movie) {
        // mcap.open("/media/rgb/6633-6433/ants.avi");
        mcap.open("/home/rgb/ants2.avi");
        frame_count = (uint32_t)mcap.get(CV_CAP_PROP_FRAME_COUNT);
        // account for a few skipped frames;
        frame_count -= frame_count / 20;
        printf("%u frames in the movie\n", frame_count);
        phw->pxy_to_loc(xpix/2, ypix/2, &phw->cur_loc);
        phw->do_move(xpix/2, ypix/2, frame_index, "Start");
        if (!mcap.isOpened()) {
             cout << "Cannot open the video file" << endl;
             return -1;
        }
    }
    if (!movie || overlay_laser) {
        ccap.open(0);
        if (!ccap.isOpened()) {
             cout << "Cannot open the video file" << endl;
             return -1;
        }
    }

    // Get things running
    //    Warm up the fg/bg window
    //    Then fire up the laser
    //    Give it a couple of frames for the laser to start
    for (int i = 0; i < 5 || !phw->hw_idle(); i++) {
        process_frame(ccap, mcap, frame, fg, half, half_fg);

        if (i == 3)
            plas->laser_on();

        if (verbose) {
            imshow("Units", half);
            if (show_mog)
                imshow("mog", half_fg);
            waitKey(1);
        }
    }
    
    // Try to start off at the center

    Point lcenter;
    Rect lbox;
    bool found_laser = false;
    int ntries = 0;
    while (!found_laser && ntries++ < 20) {
        process_frame(ccap, mcap, frame, fg, half, half_fg);
        if (verbose) {
            imshow("Units", half);
            if (show_mog)
                imshow("mog", half_fg);
            if (waitKey(1) > 0) {
                printf("Bailing in startup\n");
                while (!phw->hw_idle())
                    ;
                phw->shutdown();
                exit(1);
            }
        }

        if (find_laser(frame, fg, xpix/2, ypix/2, xpix, lcenter, lbox))
            found_laser = true;

        sleep(1);
    }

    if (found_laser) {
        DPRINTF("Found laser on startup at %d %d\n", lcenter.x, lcenter.y);
        phw->pxy_to_loc(lcenter.x, lcenter.y, &phw->cur_loc);
        phw->do_move(xpix/2, ypix/2, frame_index, "Start");
        phw->set_home();
    } else {
        printf("No laser on startup\n");
        while (!phw->hw_idle())
            ;
        phw->shutdown();
        exit(1);
    }

    plas->laser_off();
    while (!phw->hw_idle())
        /* spin */ ;

    pbl->start(phw, 0.0, 0.0);

    bool done = false;
    enum state {
        idle_laser_off,
        idle_1,
        idle_2,
        delay_1,
        delay_2,
        wait_laser,
    };

    const char *state_labels[] = {
        [idle_laser_off] = "idle_laser_off",
        [idle_1] = "idle_1",
        [idle_2] = "idle_2",
        [delay_1] = "delay_1",
        [delay_2] = "delay_2",
        [wait_laser] = "wait_laser",
    };

    enum state cur_state = idle_1;
    tps = getTickFrequency();
    int laser_on_frame = 0;
    laser_frame_lag.add_item(3);

    while(!done) {
		double dt;
        Point left, right, top, bottom;
        bool laser_vis = false;
        double tstart, tpix, twork, tend;
        frame_ticks = getTickCount();       // exported to ants.cpp
        tstart = frame_ticks/tps;
        int laser_frame_delay;

        process_frame(ccap, mcap, frame, fg, half, half_fg);

        tpix = getTickCount()/tps;
        frame_index++;

        if (!--frame_count)
            done = true;

        if (mouse_click) {
            mouse_click = false;
            cur_state = idle_1;
        }

        enum state next_state = cur_state;
        DPRINTF("cur_state: %s\n", state_labels[cur_state]);

        Point lcenter;
        Rect lbox;
        laser_vis = find_laser(frame, fg, phw->target.px,
                               phw->target.py, 100, lcenter, lbox);

        if (laser_vis) {
            if (laser_on_frame != 0)
            {
                laser_frame_lag.add_item((double)(frame_index - laser_on_frame));
                laser_on_frame = 0;
            }
        }

        DPRINTF("laser: %s\n", laser_vis ? "on" : "off");
            
        switch (cur_state) {
        case idle_laser_off:
            if(random_moves) {  
                move_randomly(&done);
                next_state = idle_1;
            } else if (!laser_vis && ant_looker(true)) {
                next_state = idle_1;
            }
            break;
        case idle_1:
            if (phw->hw_idle()) {
                plas->laser_on();
                next_state = delay_1;
                laser_on_frame = frame_index;
            } else if (!laser_vis) {
                ant_looker(false);
            }
            break;
        case delay_1:
            if (phw->hw_idle()) {
                plas->laser_off();
                laser_frame_delay = (int)round(laser_frame_lag.average()) + 1;
                if (laser_vis) {
                    if (correct(frame, lcenter, lbox))
                        next_state = idle_2;
                    else
                        next_state = delay_2;
                } else {
                    ant_looker(false);
                    next_state = wait_laser;
                }
            } else {
                ant_looker(false);
            }
            break;
        case wait_laser:
            if (laser_vis) {
                if (correct(frame, lcenter, lbox))
                    next_state = idle_2;
                else
                    next_state = delay_2;
            } else {
                ant_looker(false);
                --laser_frame_delay;
                // Give up if we don't see it
                if (laser_frame_delay == 0)
                    next_state = delay_2;
            }
            break;
        case delay_2:
            if (!laser_vis) {
                next_state = idle_laser_off;
                ant_looker(false);
            }
            break;
        case idle_2:
            if (phw->hw_idle()) {
                if (accurate) {
                    plas->laser_on();
                    next_state = delay_1;
                } else {
                    next_state = delay_2;
                }
            } else {
                if (!laser_vis)
                    ant_looker(false);
            }
            break;
        };

        // DPRINTF("next_state: %s\n", state_labels[next_state]);
        cur_state = next_state;

        left.x = 0;
        left.y = half.rows / 2;
        right.x = half.cols;
        right.y = left.y;
        top.x = half.cols / 2;
        top.y = 0;
        bottom.x = top.x;
        bottom.y = half.rows;

        line(half, left, right, Scalar(255, 255, 255));
        line(half, top, bottom, Scalar(255, 255, 255));

#if 0
        for (int i = 0; i <= half.cols - 16; i += 16) {
            left.x = i;
            left.y = border(i);
            right.x = i + 16;
            right.y = border(i + 16);
            line(half, left, right, Scalar(255, 255, 255));
        }
#endif
        twork = getTickCount()/tps;
        
        if (verbose) {
            if (show_mog) {
                pan->draw_ants();
                imshow("mog", half_fg);
            }
            if (alternate_frame && laser_vis) {
                imshow("laser", half);
            } else {
                if (plot_predictions)
                    pan->plot_predictions(half);
                imshow("Units", half);
            }
        }

        // Wait 1ms or for any key
        if((waitKey(1) & 0xff) == 27)
            done = true; 

        tend = getTickCount()/tps;

        double loop_total = tend - tstart;
        total_frame_time += loop_total;
        average_frame_time = total_frame_time / (double) frame_index;

        DPRINTF("Loop time: %d Pix: %d Work: %d Overhead: %d Average: %d frame: %d\n", 
                (int)round((loop_total)*1000.0),
                (int)round((tpix-tstart)*1000.0),
                (int)round((twork-tpix)*1000.0),
                (int)round((tend-twork)*1000.0),
                (int)round(average_frame_time*1000.0),
                frame_index);
    }

    while (!phw->hw_idle())
        ;
    phw->shutdown();
    if (verbose)
        destroyWindow("Units");
    if (show_mog)
        destroyWindow("mog");
    if (alternate_frame)
        destroyWindow("laser");
    vibe.release();
    
    exit(0);
}
