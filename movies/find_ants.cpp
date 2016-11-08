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
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

// options
bool verbose = false;

struct option {
    const char *opt;
    bool *vbl;
    const char *msg;
} opts[] = {
    { "-v", &verbose, "Verbose logging" },
    { NULL, NULL, NULL }
};

// Need something better than these globals
char *file_name;
uint32_t nframes;
bool mouse_click;
int mouse_event;
int mouse_px;
int mouse_py;

static void onMouse(int event, int px, int py, int flags, void* userdata)
{
    switch(event) {
        case (EVENT_LBUTTONDOWN):
        case (EVENT_RBUTTONDOWN):
            mouse_px = px;
            mouse_py = py;
            mouse_event = event;
            mouse_click = true;
            break;
        default:
            break;
    }
}

void
process_keys(bool &done, uint32_t &frame_index)
{
    uint32_t cur_frame = frame_index;
    int key;
    int64_t count = 0;
    while (cur_frame == frame_index && !done) {
        if((key = waitKey(1)) > 0) {
            key &= 0xffff;
            switch(key) {
                case(0xff53): // right arrow
                case('n'):    
                case(' '):
                    if (count == 0)
                        count = 1;
                    if (count + (int64_t)frame_index < (int64_t)nframes)
                        frame_index += count;
                    else
                        frame_index = nframes - 1;
                    break;
                case(0xff51): // left arrow
                case('p'):
                    if (count == 0)
                        count = 1;
                    if ((int64_t) frame_index - count > 0)
                        frame_index -= count;
                    else
                        frame_index = 0;
                    break;
                case('0'):
                case('1'):
                case('2'):
                case('3'):
                case('4'):
                case('5'):
                case('6'):
                case('7'):
                case('8'):
                case('9'):
                    count = count * 10 + key - '0';
                    break;
                case(27): // escape
                case('q'):
                    done = true;
                    break;
                default:
                    printf("Key 0x%x %c\n", key, key);
                    break;
            }
        }
        if (mouse_click) {
            printf("%s %u %d %d\n",
                   file_name, frame_index, mouse_px, mouse_py);
            if (mouse_event == EVENT_RBUTTONDOWN)
                frame_index++;
            mouse_click = false;
        }
    }
}

int main(int argc, char* argv[])
{
	Mat frame;

    printf("find_ants\n");

    if (argc < 2) {
        printf("Need at least a file name\n");
        exit(1);
    }

    ++argv;
    --argc;
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

    file_name = *argv;

    // open the video file for reading
    VideoCapture cap;

    namedWindow("find_ants", CV_WINDOW_AUTOSIZE);
    setMouseCallback("find_ants", onMouse, NULL);

    cap.open(file_name);

    if (!cap.isOpened()) {
        printf("Can't open %s\n", file_name);
        exit(1);
    }

    bool done = false;
    uint32_t frame_index = 0;
    nframes = (uint32_t)cap.get(CV_CAP_PROP_FRAME_COUNT);
    printf("nframes = %d\n", nframes);
    while(!done) {
        cap.set(CV_CAP_PROP_POS_FRAMES, (double) frame_index);
        cap.read(frame); // read a new frame from video
		if (frame.empty()) {
            printf("last frame!\n");
			done = true;
        } else {
            char txt[50];
            sprintf(txt, "%u", frame_index);
            putText(frame, (string) txt, Point(10,50), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 255, 255)); 
            imshow("find_ants", frame);
            process_keys(done, frame_index);
        }
    }

    exit(0);
}
