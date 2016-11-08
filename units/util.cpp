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
#include <assert.h>

#include "opencv2/core/core.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

#include "hw.h"
#include "util.h"

running_average::running_average()
{
    this->nitems = 0;
    items = NULL;
    total = 0.0;
    n = 0;
    in = 0;
}

running_average::running_average(int nitems)
{
    this->nitems = nitems;
    items = new double[nitems];
    total = 0.0;
    n = 0;
    in = 0;
}

void running_average::add_item(double d)
{
    assert(items != NULL);
    if (n == nitems)
        total -= items[in];
    else
        n++;
    total += d;
    items[in] = d;
    in++;
    if (in == nitems)
        in = 0;
}

double running_average::average()
{
    if (n == 0)
        return 0;
    return total/n;
}

direction_average::direction_average()
{
    this->nitems = 0;
    items = NULL;
    total = Point2d(0.0, 0.0);
    n = 0;
    in = 0;
}

direction_average::direction_average(int nitems)
{
    this->nitems = nitems;
    items = new Point2d[nitems];
    total = Point2d(0.0, 0.0);
    n = 0;
    n = 0;
    in = 0;
}

void direction_average::add_item(Point2d p)
{
    assert(items != NULL);
    if (n == nitems)
        total -= items[in];
    else
        n++;
    total += p;
    items[in] = p;
    in++;
    if (in == nitems)
        in = 0;
}

Point2d direction_average::average()
{
    if (n == 0)
        return Point2d(0.0, 0.0);
    double mag = sqrt(total.x * total.x + total.y * total.y);
    if (mag == 0.0)
        return Point2d(0.0, 0.0);
    return Point2d(total.x/mag, total.y/mag);
}

laser::laser(hw *phw, bool start)
{
    is_on = start;
    this->phw = phw;
}

void laser::laser_on(void)
{
    is_on = true;
    this->phw->switch_laser(true);
}

void laser::laser_off(void)
{
    is_on = false;
    this->phw->switch_laser(false);
}

void laser::draw_laser(Mat &src)
{
    if (is_on) {
        Point center(this->phw->target.px,
                     this->phw->target.py);
        int rad = 8;
        circle(src, center, rad, Scalar(255, 255, 255), -1);
    }
}
