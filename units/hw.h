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

#define xpix 1280
#define ypix 960

struct loc {
    int px;             // Pixel coords
    int py;
    double xd;          // Pinhole (distored) camera coords
    double yd;
    double x;           // Undistored camera cooords
    double y;
    double xm;          // Mirror coord system coords
    double ym;
    double m1_theta;    // Angles for mirrors 
    double m2_theta;
    double m1_steps;    // Steps for mirrors
    double m2_steps;
};

class backlash;

class hw {
    public: 
        struct loc cur_loc;
        struct loc target;
        int last_m1;
        int last_m2;

        hw(backlash *pbl);
        void set_home(void);
        void do_move(int px, int py, int frame_index, const char *msg);
        double move_time(int px, int py);
        void do_xy_move(double x, double y, const char *msg);
        void do_correction(int px, int py, int frame_index, const char *msg);
        void switch_laser(bool laser_on);
        void pxy_to_loc(int px, int py, struct loc *ploc);
        void xy_to_loc(double x, double y, struct loc *ploc);
        double mm_per_pixel(int px, int py);
        void shutdown(void);
        bool hw_idle(void);
        bool keepout(int px, int py, int scale);
    private:
        volatile struct coms *pc;
        backlash *pbl;
        int m1_limit;
        int m2_limit;

        double px_to_xd(double px);
        double py_to_yd(double py);
        double xdyd_to_x(double xd, double yd);
        double xdyd_to_y(double xd, double yd);
        void pxy_to_xy(int px, int py, double &x, double &y);
        double calc_m2_theta(double x);
        double calc_m1_theta(double y, double m2Theta);
        double steps_to_theta(int steps);
        double theta_to_steps(double theta);
        bool start_move(double m1_steps, double m2_steps, bool laser_on);
};

class backlash {
    public:
        backlash();
        void correct(double *pm1s, double *pm2s);
        void start(hw* phw, double m1s, double m2s);
        void add_corr(hw* phw, double m1s, double m2s);
        void stop(hw* phw);
        void dumpit();
    private:
        int last_m1;
        int last_m2;
        int m1_limit;
        int m2_limit;
        struct loc *pstart;
        struct loc *ptarget;
        struct step_list *pls;
        struct step_list *ple;
        int mvidx;
        void cleanup();
        void actuals(struct step_list *pn, int *pm1, int *pm2);
        void dead_zone(struct step_list *pn, int *pm1dz, int *pm2dz);
        FILE *sql_out;
};

extern bool verbose;
#define DPRINTF if (verbose) printf
