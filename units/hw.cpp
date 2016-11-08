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

struct coms {
    uint32_t magic;
    uint16_t ms;
    int16_t  m1_steps;
    int16_t  m2_steps;
    uint16_t flags;
    uint16_t ok;
};

/* 
 * The camera coordinate system is centered under the camera, 
 * Inches, +x to the right, +y up, as the camera sees it.
 * Mirror coordinate system is centred on mirror 1,
 * Inches +x toward motor 1, +y toward mirror 2
 * Mirror 1 angles are measured m1 straight up toward laser,
 * Mirror 2 straight down
 */

// flags:
#define LASER_ON        0x01
#define MOTORS_ON       0x02
#define SHUTDOWN        0x04
#define M1_NEG          0x08
#define M2_NEG          0x10

// Constants

// Derived at home
// Measured distance to focal plane to be 320.5
// 308mm to flange of the camera + csmount spec of 12.5mm to the sensor
// For 16mm squares, measured 94 pixels
// Using 4.65um for pixel size
// f/94 == 320.5/16 so f == 94 * 320.5 / 16 == 1883.94 "pixels"
// 1883.94 * 0.00465 == 8.76mm
// To get new K1 - P3, run get_calib, feed the coordinates to 
// Mathematica, least_sqares.nb
#define lens_focal_len (8.76 / 25.4)
#define K1 0.0010958
#define K2 0.00021057 
#define K3 (-5.575E-6)
#define P1 (-0.00299204)
#define P2 0.000119739 
#define P3 (-0.0227986)

#define in_per_pix (0.00465 / 25.4)
#define camera_height (320.5 / 25.4)

#define m1x 0.0
#define m1y 0.0
#define m1z 1.625
// y axis wants m2z smaller, x axis bigger
#define m2z 10.125
#define m2za (m2z-0.625)
#define m2zb (m2z+0.625)

#define steps_per_rev 200.0
#define microsteps_per_step 16
#define gear_ratio 5.2
#define camera_to_mirrors_x (49.0/25.4)
// from offset.py
// #define camera_to_mirrors_y 10.303022
#define camera_to_mirrors_y 10.1

// Step offsets from pixel pos 640, 480
#define m1_max 345
#define m1_min -380
#define m2_max 980
#define m2_min -860

// Move constants. Must agree with defaults in eibot.py
#define accel 2800.0
#define max_v 800.0
#define ramp_time (max_v/accel)
#define ramp_dist (accel * ramp_time * ramp_time / 2.0)

double steps_to_theta(int steps);

// options
extern bool fake_laser;
extern bool sql_backlash;
extern bool draw_laser;

// HW class
hw::hw(backlash *pbl)
{
    struct coms icoms;
    int fd;

    this->pbl = pbl;
    m1_limit = 0;
    m2_limit = 0;

    // Setup shared mem
    icoms.ms = 0;
    icoms.m1_steps = 0;
    icoms.m2_steps = 0;
    icoms.flags = 0;
    icoms.ok = 0;
    icoms.magic = 0x12344321;

    fd = open("/home/rgb/shmem", O_RDWR | O_CREAT);
    if (fd < 0) {
        printf("can't open /tmp/shmem\n");
        exit(1);
    }
    fchmod(fd, S_IRUSR|S_IWUSR);
    write(fd, &icoms, sizeof(icoms));
    pc = (struct coms *) mmap(0, sizeof(coms), PROT_WRITE | PROT_WRITE,
                              MAP_SHARED, fd, 0);
    if (pc == MAP_FAILED) {
        printf("mmap failed\n");
        exit(1);
    }

    // fire up the unit
    pc->flags |= MOTORS_ON;
    if (!fake_laser)
        pc->ok = 1;

    // And wait here until clicker.py is up and running
    while (pc->ok == 1)
        ;
}

void hw::set_home()
{
    m1_limit = 0;
    m2_limit = 0;
}

double hw::px_to_xd(double px)
{
    return (px - xpix/2) * in_per_pix * camera_height / lens_focal_len;
}

double hw::py_to_yd(double py)
{
    return -(py - ypix/2) * in_per_pix * camera_height / lens_focal_len;
}

double hw::xdyd_to_x(double xd, double yd)
{
    double r2 = xd*xd + yd*yd;
    double x_radial_corr = xd*(K1*r2 + K2*r2*r2 + K3*r2*r2*r2);
    double x_tan_corr = (P1*(r2 + 2.0*xd*xd) + 2.0*P2*xd*yd)*(1 + P3*r2);
    return (xd + x_radial_corr + x_tan_corr);
}

double hw::xdyd_to_y(double xd, double yd)
{
    double r2 = xd*xd + yd*yd;
    double y_radial_corr = yd*(K1*r2 + K2*r2*r2 + K3*r2*r2*r2);
    double y_tan_corr = (2.0*P1*xd*yd + P2*(r2 + 2.0*yd*yd))*(1 + P3*r2);
    return (yd + y_radial_corr + y_tan_corr);
}

double hw::calc_m2_theta(double x)
{
    return -atan2(m1x - x, m2zb)/2.0;
}

double hw::calc_m1_theta(double y, double m2Theta)
{
    return -atan2(m1y - y, m2za - m1z + m2z  / cos(2.0 * m2Theta)) / 2.0;
}

double hw::theta_to_steps(double theta)
{
    return (theta * steps_per_rev * microsteps_per_step * gear_ratio /
                  (2 * M_PI));
}

double hw::steps_to_theta(int steps)
{
    return (steps * 2 * M_PI /
            (steps_per_rev * microsteps_per_step * gear_ratio));
}

static uint64_t move_timer;
bool hw::start_move(double m1_steps, double m2_steps, bool laser_on)
{
    if (fake_laser) 
        return true;
    int m1 = (int)(round(m1_steps));
    int m2 = (int)(round(m2_steps));
    if (m1_limit + m1 < m1_min ||
        m1_limit + m1 > m1_max ||
        m2_limit + m2 < m2_min ||
        m2_limit + m2 > m2_max) {

        DPRINTF("Bad move!\n");
        DPRINTF("at %d %d moving %d %d\n",
                m1_limit, m2_limit, m1, m2);
        return false;
    }
    m1_limit += m1;
    m2_limit += m2;
    if (pc->ok == 0) {
        move_timer = getTickCount();
        if (m1 != 0)
            last_m1 = m1;
        if (m2 != 0)
            last_m2 = m2;
        pc->ms = 0;
        // Oh boy this sucks
        if (m1 < 0) {
            pc->m1_steps = -m1;
            pc->flags |= M1_NEG;
        } else {
            pc->m1_steps = m1;
            pc->flags &= ~M1_NEG;
        }
        if (m2_steps < 0) {
            pc->m2_steps = -m2;
            pc->flags |= M2_NEG;
        } else {
            pc->m2_steps = m2;
            pc->flags &= ~M2_NEG;
        }
        if (laser_on)
            pc->flags |= LASER_ON;
        else
            pc->flags &= ~LASER_ON;
        pc->ok = 1;
    } else {
        DPRINTF("start_move command not done\n");
        return false;
    }
    return true;
}

void hw::switch_laser(bool laser_on)
{
    if (fake_laser) 
        return;
    if (pc->ok == 0) {
        pc->m1_steps = 0;
        pc->m2_steps = 0;
        pc->flags &= ~M1_NEG;
        pc->flags &= ~M2_NEG;
        if (laser_on)
            pc->flags |= LASER_ON;
        else
            pc->flags &= ~LASER_ON;
        pc->ok = 1;
    } else {
        DPRINTF("laser command not done!\n");
    }
}

void hw::xy_to_loc(double x, double y, struct loc *ploc)
{
    ploc->x = x;
    ploc->y = y;
    ploc->xm = camera_to_mirrors_x - ploc->x;
    ploc->ym = camera_to_mirrors_y - ploc->y;
    ploc->m2_theta = calc_m2_theta(ploc->xm);
    ploc->m1_theta = calc_m1_theta(ploc->ym, ploc->m2_theta);
    ploc->m1_steps = -theta_to_steps(ploc->m1_theta);
    ploc->m2_steps = -theta_to_steps(ploc->m2_theta);
// #if 0
    DPRINTF("  px: %d, py: %d, xd: %6.3lf, yd:%6.3lf\n",
           ploc->px, ploc->py, ploc->xd, ploc->yd);
    DPRINTF("  x: %6.3lf, y: %6.3lf, xm: %6.3lf, ym: %6.3lf\n",
           ploc->x, ploc->y, ploc->xm, ploc->ym);
    DPRINTF("  m1_theta: %6.4lf, m2_theta: %6.4lf, m1_steps: %6.1lf, m2_steps: %6.1lf\n",
           ploc->m1_theta, ploc->m2_theta, ploc->m1_steps, ploc->m2_steps);
// #endif
}

void hw::pxy_to_loc(int px, int py, struct loc *ploc)
{
    double x, y;
    ploc->px = px;
    ploc->py = py;
    ploc->xd = px_to_xd(px); 
    ploc->yd = py_to_yd(py); 
    x = xdyd_to_x(ploc->xd, ploc->yd);
    y = xdyd_to_y(ploc->xd, ploc->yd);
    xy_to_loc(x, y, ploc);
}

void hw::pxy_to_xy(int px, int py, double &x, double &y)
{
    double xd, yd;
    xd = px_to_xd(px);
    yd = py_to_yd(py);
    x = xdyd_to_x(xd, yd);
    y = xdyd_to_y(xd, yd);
}

double hw::mm_per_pixel(int px, int py)
{
    double x1, y1, x2, y2;
    int px1;
    if (px >= xpix - 10)
        px1 = px - 10;
    else
        px1 = px + 10;
    pxy_to_xy(px, py, x1, y1);
    pxy_to_xy(px1, py, x2, y2);
    double dx = x1 - x2;
    double dy = y1 - y2;
    double dist_inches = sqrt(dx * dx + dy * dy) / 10.0;
    return dist_inches * 25.4;
}

double hw::move_time(int px, int py)
{
    struct loc tloc;
    pxy_to_loc(px, py, &tloc);
    double m1_delta = tloc.m1_steps - cur_loc.m1_steps;
    double m2_delta = tloc.m2_steps - cur_loc.m2_steps;
    double dist = sqrt(m1_delta * m1_delta + m2_delta * m2_delta);
    double t;
    if (draw_laser) {
        t = 0.0;
    } else if (dist > ramp_dist * 2.0) {
        t = ramp_time * 2.0 + (dist - ramp_dist * 2.0)/max_v;
    } else {
        // Integral relating distance to accel is d = acc * t^2 / 2
        // So t = sqrt(2d/accel)
        // dist == 2d, because want ramp up then down. 
        t = 2.0 * sqrt(dist / accel);
    }
    DPRINTF("move_time to px: %d py: %d dist: %6.1lf t: %6.3lf\n",
           px, py, dist, t);
    return t;
}

void hw::do_move(int px, int py, int frame_index, const char *msg)
{
    DPRINTF("%s, frame_index: %d\n", msg, frame_index);
    if (px != target.px || py != target.py)
        pxy_to_loc(px, py, &target);
    double m1_delta = target.m1_steps - cur_loc.m1_steps;
    double m2_delta = target.m2_steps - cur_loc.m2_steps;
    pbl->start(this, m1_delta, m2_delta);
    DPRINTF("  Moving %6.1lf %6.1lf\n", m1_delta, m2_delta);
    while (pc->ok)
        ;
    if (start_move(m1_delta, m2_delta, false))
        cur_loc = target;
}

void hw::do_correction(int px, int py, int frame_index, const char *msg)
{
    // DPRINTF("%s, frame_index: %d\n", msg, frame_index);
    if (px != target.px || py != target.py)
        pxy_to_loc(px, py, &target);
    double m1_delta = (target.m1_steps - cur_loc.m1_steps) * 1.0;
    double m2_delta = (target.m2_steps - cur_loc.m2_steps) * 1.0;
    pbl->add_corr(this, m1_delta, m2_delta);
    DPRINTF("  Moving %6.1lf %6.1lf\n", m1_delta, m2_delta);
    while (pc->ok)
        ;
    if(start_move(m1_delta, m2_delta, false))
        cur_loc = target;
}

void hw::do_xy_move(double x, double y, const char *msg)
{
    DPRINTF("%s\n", msg);
    target.px = 0;
    target.py = 0;
    target.xd = 0;
    target.yd = 0;
    xy_to_loc(x, y, &target);
    double m1_delta = target.m1_steps - cur_loc.m1_steps;
    double m2_delta = target.m2_steps - cur_loc.m2_steps;
    pbl->start(this, m1_delta, m2_delta);
    DPRINTF("  Moving %6.1lf %6.1lf\n", m1_delta, m2_delta);
    while (pc->ok)
        ;
    if(start_move(m1_delta, m2_delta, false))
        cur_loc = target;
}

void hw::shutdown(void)
{
    while (pc->ok)
        ;
    pc->m1_steps = 0;
    pc->m2_steps = 0;
    pc->flags = SHUTDOWN;
    if (!fake_laser)
        pc->ok = 1;
}

bool hw::hw_idle()
{
    if (pc-> ok == 0 && move_timer != 0) {
        uint64_t cur_time = getTickCount();
        double tps = getTickFrequency();
        DPRINTF("Move Timer: %d\n", (int)((cur_time-move_timer)*1000.0/tps));
        move_timer = 0;
    }
    return pc->ok == 0;
}

/*
 * much easier with the camera at 8.8mm
 */

bool hw::keepout(int px, int py, int scale)
{
    if (scale == 2) {
        px += px;
        py += py;
    } else if (scale != 1) {
        px *= scale;
        py *= scale;
    }
    
    if (py < 0 || py > 959)
        return true;

    if (px < 0 || px > 1279)
        return true;

    return false;
}

struct step_list {
    struct loc *ploc;
    int last_m1;
    int last_m2;
    double m1s;
    double m2s;
    struct step_list *next;
};

// Backlash class

backlash::backlash()
{
    last_m1 = 0;
    last_m2 = 0;
    pstart = NULL;
    ptarget = NULL;
    pls = NULL;
    ple = NULL;
    mvidx = 0;

    if (!sql_backlash)
        return;

    sql_out = fopen("backlash.sql", "w");
    if (!sql_out) {
        DPRINTF("can't open backlash.sql\n");
        exit(1);
    }
}


void backlash::correct(double *pm1s, double *pm2s)
{
// wrong
return;
}

void backlash::cleanup()
{
    if (pstart)
        delete pstart;
    if (ptarget)
        delete ptarget;
    while (pls) {
        struct step_list *tmp = pls;
        if (pls->ploc)
            delete pls->ploc;
        pls = pls->next;
        delete tmp;
    }
    last_m1 = 0;
    last_m2 = 0;
    pstart = NULL;
    ptarget = NULL;
    pls = NULL;
    ple = NULL;
}

void backlash::start(hw *phw, double m1s, double m2s)
{
    cleanup();
    last_m1 = phw->last_m1;
    last_m2 = phw->last_m2;
    pstart = new struct loc;
    *pstart = phw->cur_loc;
    ptarget = new struct loc;
    *ptarget = phw->target;
    pls = new struct step_list;
    pls->m1s = m1s;
    pls->m2s = m2s;
    pls->ploc = NULL;
    pls->next = NULL;
    ple = pls;
    mvidx++;
}

void backlash::add_corr(hw *phw, double m1s, double m2s)
{
    if (!sql_backlash)
        return;

    struct step_list *pn = new struct step_list;
    pn->last_m1 = phw->last_m1;
    pn->last_m2 = phw->last_m2;
    pn->m1s = m1s;
    pn->m2s = m2s;
    pn->ploc = new struct loc;
    *pn->ploc = phw->cur_loc;
    pn->next = NULL;
    if (ple)
        ple->next = pn;
    ple = pn;
}

void backlash::stop(hw *phw)
{
    struct step_list *pn = new struct step_list;
    pn->last_m1 = phw->last_m1;
    pn->last_m2 = phw->last_m2;
    pn->m1s = 0;
    pn->m2s = 0;
    pn->ploc = new struct loc;
    *pn->ploc = phw->cur_loc;
    pn->next = NULL;
    if (ple)
        ple->next = pn;
    ple = pn;
}

void backlash::actuals(struct step_list *pn, int *pm1, int *pm2)
{
    int m1 = 0;
    int m2 = 0;
    while (pn) {
        m1 += round(pn->m1s);
        m2 += round(pn->m2s);
        pn = pn->next;
    }
    *pm1 = m1;
    *pm2 = m2;
}

void backlash::dead_zone(struct step_list *pn, int *pm1dz, int *pm2dz)
{
    int m1dz = 0;
    int m2dz = 0;
    struct step_list *plast = NULL;

    while (pn) {
        if (pn->ploc && plast && plast->ploc) {
            if (plast->ploc->px == pn->ploc->px)
                m2dz += round(plast->m2s);
            if (plast->ploc->py == pn->ploc->py)
                m1dz += round(plast->m1s);
        }
        plast = pn;
        pn = pn->next;
    }
    *pm1dz = m1dz;
    *pm2dz = m2dz;
}

void backlash::dumpit()
{
    struct step_list *pn = pls;
    int px, py;
    int m1_delta, m2_delta;
    int m1_actual, m2_actual;
    int tmp_last_m1, tmp_last_m2;
    int m1_dz, m2_dz;

    if (!sql_backlash)
        return;

    // ploc is NULL for starts.
    while (pn) {
        if (pn->ploc) {
            px = pn->ploc->px;
            py = pn->ploc->py;
            m1_delta = round(ptarget->m1_steps - pn->ploc->m1_steps);
            m2_delta = round(ptarget->m2_steps - pn->ploc->m2_steps);
            tmp_last_m1 = pn->last_m1;
            tmp_last_m2 = pn->last_m2;
            fprintf(sql_out, "INSERT INTO corr VALUES( %4d,", mvidx);
        } else {
            tmp_last_m1 = last_m1;
            tmp_last_m2 = last_m2;
            fprintf(sql_out, "INSERT INTO move VALUES( %4d,", mvidx);
        }
        fprintf(sql_out, "%4d, %4d, ", pstart->px, pstart->py);
        fprintf(sql_out, "%4d, %4d, ", ptarget->px, ptarget->py);
        fprintf(sql_out, "%4d, %4d, ", tmp_last_m1, tmp_last_m2);
        if (pn->ploc) {
            fprintf(sql_out, "%4d, %4d, ", px, py);
            fprintf(sql_out, "%4d, %4d, ", m1_delta, m2_delta);
        }
        fprintf(sql_out, "%4d, %4d, ", (int)round(pn->m1s), (int)round(pn->m2s));
        actuals(pn, &m1_actual, &m2_actual);
        fprintf(sql_out, "%4d, %4d, ", m1_actual, m2_actual);
        dead_zone(pn, &m1_dz, &m2_dz);
        fprintf(sql_out, "%4d, %4d ", m1_dz, m2_dz);

        fprintf(sql_out, ");");
        fprintf(sql_out, "\n");
        pn = pn->next;
    }
    cleanup();
}
