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

#include <queue>

struct bg_pt {
    Point p;
    int frame;
};


class snapshots {
    public:
        snapshots(Mat *pframe);
        void snap_laser(Point p);
        void snap_ant(Point p);
        void snap_bg(Point p);
    private:
        Mat *pframe;
        int size;
        int half_size;
        uint32_t snap_num;
        char image_name[50];
        string dir;
        std::queue<bg_pt> bg_pts;

        void make_snapshot(Point p, string tag);
};

enum image_type {
    bg_index = 0,
    ant_index,
    laser_index,
};

class Classifier;

class image_classifier {
    public:
        image_classifier(void);
        std::vector<float> get_image_type(Mat *pframe, Point p);
    private:
        Classifier *pclass;
};
