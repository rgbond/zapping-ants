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

#include "blobs.h"

extern int verbose;

class bit_vec {
    public:
        bit_vec(uint32_t size, bool value);
        void set(uint32_t i);
        void clear(uint32_t i);
        bool value(uint32_t i);
    private:
        uint32_t len;
        uint32_t nel;
        unsigned char *p;
        static const int as = 8;
};

bit_vec::bit_vec(uint32_t len, bool value)
{
    this->len = len;
    this->nel = (len + as)/as;
    this->p = new unsigned char[nel];
    if (value) {
        for (int i = 0; i < nel; i++)
            p[i/as] = ~0;
    } else {
        for (int i = 0; i < nel; i++)
            p[i/as] = 0;
    }
}

void bit_vec::set(uint32_t i)
{ 
    uint32_t el = i / as;
    uint32_t bit = i - el*as;
    p[el] |= (1<<bit);
}

void bit_vec::clear(uint32_t i)
{
    uint32_t el = i / as;
    uint32_t bit = i - el*as;
    p[el] &= ~(1<<bit);
}

bool bit_vec::value(uint32_t i)
{
    uint32_t el = i / as;
    uint32_t bit = i - el*as;
    if (p[el] & (1<<bit))
        return true;
    else
        return false;
}

struct pt_stack {
    int x;
    int y;
    struct pt_stack *next;
};

struct pt_stack *ps_top;

inline void push_pt(int x, int y)
{
    pt_stack *ps = new pt_stack;
    ps->x = x;
    ps->y = y;
    ps->next = ps_top;
    ps_top = ps;
}

inline bool pop_pt(int *px, int *py)
{
    struct pt_stack *tmp;

    if (ps_top == NULL)
        return false;
    *px = ps_top->x;
    *py = ps_top->y;
    tmp = ps_top;
    ps_top = ps_top->next;
    delete tmp;
    return true;
}

inline struct rec_list *
add_blob(Mat &src, int x0, int y0, Rect r, struct rec_list *precs,
         bool *error, int thresh, bit_vec &checked)
{
    int blob_left = r.x + r.width;
    int blob_top = r.y + r.height;
    int blob_right = r.x;
    int blob_bottom = r.y;
    uint64_t xtot = 0;
    uint64_t ytot = 0;
    uint32_t npix = 0;
    int x0r = x0 - r.x;
    int y0r = y0 - r.y;
    int x;
    int y;
    struct rec_list *pnr;

    ps_top = NULL;
    if (src.at<uchar>(y0, x0) >= thresh ||
        checked.value(y0r*r.width + x0r))
        return precs;
    push_pt(x0, y0);
    while (pop_pt(&x, &y)) {
        int xr = x - r.x;
        int yr = y - r.y;
        if (src.at<uchar>(y, x) < thresh && 
            !checked.value(yr*r.width + xr)) {
            checked.set(yr*r.width + xr);
            blob_left = std::min(x, blob_left);
            blob_top = std::min(y, blob_top);
            blob_right = std::max(x, blob_right);
            blob_bottom = std::max(y, blob_bottom);
            xtot += x;
            ytot += y;
            npix++;
            if (npix > 2000) {
                // Background subtract sucks 
                int jx, jy;
                while (pop_pt(&jx, &jy))
                    ;
                *error = true;
                return precs;
            }
            if (y > r.y &&
                src.at<uchar>(y-1, x) < thresh)
                push_pt(x, y-1);
            if (x > r.x &&
                src.at<uchar>(y, x-1) < thresh)
                push_pt(x-1, y);
            if (y < r.y + r.height -1 &&
                src.at<uchar>(y+1, x) < thresh)
                push_pt(x, y+1);
            if (x < r.x + r.width -1 &&
                src.at<uchar>(y, x+1) < thresh)
                push_pt(x+1, y);
        }
    }

    if (npix < 3) {
        *error = false;
        return precs;
    }

    // convert to full image size
    pnr = new rec_list;
    pnr->rect.x = blob_left;
    pnr->rect.y = blob_top;
    pnr->rect.width = (blob_right - blob_left + 1);
    pnr->rect.height = (blob_bottom - blob_top + 1);
    pnr->xc = xtot/npix;
    pnr->yc = ytot/npix;
    pnr->npix = npix;
    pnr->pnext = precs;
    precs = pnr;

    if (verbose) {
        cout << "Blob: " << pnr->rect.x << " " << pnr->rect.y;
        cout <<  " " << pnr->rect.width << "x" << pnr->rect.height;
        cout << " npix: " << npix;
        cout << "\n";
    }

    *error = false;
    return precs;
}

// Finds a list of blobs that  are < thresh in color
struct rec_list *find_bbb(Mat& fg, Rect r, int thresh)
{
    struct rec_list *precs = NULL;
    const int inc64 = sizeof(uint64_t);
    int cols = fg.cols & ~(sizeof(uint64_t) - 1);
    int num_blob = 0;

    int xs = r.x;
    int xe = r.x + r.width;
    int ys = r.y;
    int ye = r.y + r.height;

    bit_vec checked(r.width * r.height, false);

    for (int y = ys; y < ye; y++) {
        uint8_t *pfg = fg.data + fg.step * y + xs;
        int x;

        for (int x = xs; x < xe; x += inc64, pfg += inc64) {
            if (*(uint64_t *)pfg != 0) {
                for (int x1 = x; x1 < x + inc64 && x1 < fg.cols; x1++) {
                    if (fg.at<uchar>(y, x1) < thresh) {
                        bool error;
                        if (num_blob++ > 1000) {
                            printf("More than 1000 Blob candidates!\n");
                            goto bail;
                        }
                        precs = add_blob(fg, x1, y, r, precs, &error, thresh, checked);
                        if (error) {
                            while (precs) {
                                struct rec_list *pn = precs;
                                precs = precs->pnext;
                                delete pn;
                            }
                            printf("Blob overflow!\n");
                            goto bail;
                        }
                    }
                }
            }
        }
    }
bail:
    return precs;
}
