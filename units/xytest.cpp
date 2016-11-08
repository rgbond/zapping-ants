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
#include "hw.h"

using namespace std;

// options
bool alternate_frame = false;
bool fake_laser = false;
bool sql_backlash = false;
bool random_moves = false;
bool verbose = true;
bool draw_laser = false;

struct option {
    const char *opt;
    bool *vbl;
    const char *msg;
} opts[] = {
    { "-a", &alternate_frame, "Alternate frame mode enabled" },
    { "-f", &fake_laser, "Fake the laser coms" },
    { "-s", &sql_backlash, "Save sql formatted backlash data" },
    { "-r", &random_moves, "Do random moves" },
    { NULL, NULL, NULL }
};

// Need something better than these globals
backlash *pbl;
hw *phw;

inline double normal(double mean, double sigma)
{
    double u1 = (double)rand()/(double)RAND_MAX;
    double u2 = (double)rand()/(double)RAND_MAX;
    return sqrt(-2.0 * log(u1)*sigma*sigma)*cos(2.0 * M_PI * u2) + mean;
}

void move_randomly()
{
    double x = normal(0, 2.0);
    double y = normal(0, 1.5);
    x = x >  1.0 ? x :  1.0;
    x = x < -9.0 ? x : -9.0;
    y = y > -6.0 ? y : -6.0;
    y = y <  6.0 ? y :  6.0;
    phw->do_xy_move(x, y, "Random");
}

int main(int argc, char* argv[])
{
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

    pbl = new backlash();
    phw = new hw(pbl);

    // Assume we start centered under the camera
    phw->pxy_to_loc(xpix/2, ypix/2, &phw->target);
    phw->pxy_to_loc(xpix/2, ypix/2, &phw->cur_loc);
    pbl->start(phw, 0.0, 0.0);

    int random_count = 150;
    bool done = false;
    while(!done) {
        char buff[50];
        printf("cmd > "); fflush(stdout);
        fgets(buff, 50, stdin);
        char *p = buff + 1;
        switch(buff[0]) {
        case 'm':
            double x, y;
            x = strtod(p, &p);  
            if (*p == ',')
                p++;
            y = strtod(p, &p);  
            phw->do_xy_move(x, y, "m cmd");
            break;
        case 'q':
        case EOF:
            done = true;
            break;
        default:
            printf("eh?\n");
            break;
        }
    }
    if (random_moves && phw->hw_idle()) {
        if(random_count > 0) {
            random_count--;
            move_randomly();
        } else if (random_count == 0) {
            random_count--;
            phw->do_move(xpix/2, ypix/2, 0, "Home");
        }
    }


    phw->shutdown();
    exit(0);
}
