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

class running_average {
    public:
        running_average();
        running_average(int nitems);
        void add_item(double d);
        double average();
    private:
        double *items;
        int nitems;
        int n;
        int in;
        double total;
};

class direction_average {
    public:
        direction_average();
        direction_average(int nitems);
        void add_item(Point2d p);
        Point2d average();
    private:
        Point2d *items;
        int nitems;
        int n;
        int in;
        Point2d total;
};

class laser {
    public:
        laser(hw *phw, bool start);
        void laser_on();
        void laser_off();
        void draw_laser(Mat &src);
    private:
        hw *phw;
        bool is_on;
};
