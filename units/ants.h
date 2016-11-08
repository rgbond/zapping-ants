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

struct ant_list {
    int id;                           // My name
    int score;
    Point last;                       // Last found at
    Point pred;                       // Prediction for this one
    direction_average uv;            // Unit vector direcion ant is traveling
    running_average avg_speed;        // pixels per second
    uint32_t last_frame; 
    uint64_t last_frame_ticks;
    double total_distance;
    uint32_t this_frame;
    uint32_t blobs_this_frame;
    double laser_dist;
    struct ant_list *next;
};

// Tuning
const int close_blob = 40;            // Max for how close a blob must be to an ant
const int ant_ppf = 35;               // Pixels per frame, average
const int ant_frames = 10;            // Max frames to track
const double ant_len = 2.5;           // Actual length of an ideal ant in mm
const double ant_width = ant_len/2.0; // Actual width of an ideal ant in mm
const int ant_color = 80;             // Ideal color
const int max_score = 50;

class ants {
    public:
        ants(hw *phw, Mat *pframe, Mat *pfg, Mat *phalf_fg,
             snapshots *psnap, image_classifier *pclass);
        struct ant_list *select_ant();
        void predict_next_pos(struct ant_list *pant, int *px, int *py);
        void draw_ants();
        void plot_predictions(Mat &half);
        struct ant_list *all_ants(void);
    private:
        hw *phw;
        Mat *pframe;
        Mat *pfg;
        Mat *phalf_fg;
        snapshots *psnap;
        image_classifier *pclass;
        struct ant_list *pants;
        void ant_score(struct rec_list *pn);
        void score_ants(struct rec_list *precs);
        struct ant_list *pick_best_ant(struct ant_list *pant);
        struct ant_list *delete_dead_ants(struct ant_list *pant);
        struct ant_list *best_ant(struct ant_list *pant);
        void match_blobs_to_ant(struct rec_list *precs, struct ant_list *pant);
        void match_blobs_to_ants(struct rec_list *precs, struct ant_list *pants);
        void process_ant(struct rec_list *pn);
        void add_ant(struct rec_list *pn);
};

// Sizes of ants in sq pixels
#define PTG 20
#define PIX_TBL_WIDTH (xpix / PTG)
#define PIX_TBL_HEIGHT (ypix / PTG)
extern uint16_t ant_pix[PIX_TBL_WIDTH][PIX_TBL_HEIGHT];
inline uint8_t get_ant_size(int x, int y)
{
    return ant_pix[x/PTG][y/PTG];
}
