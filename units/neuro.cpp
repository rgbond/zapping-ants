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

#include <algorithm>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <caffe/caffe.hpp>


using namespace std;
using namespace cv;
using namespace caffe;  // NOLINT(build/namespaces)
using std::string;

#include "neuro.h"

extern int frame_index;
extern bool verbose;

#define IMG_SIZE 28

snapshots::snapshots(Mat *pframe)
{
    this->pframe = pframe;
    size = IMG_SIZE;
    half_size = size / 2;
    snap_num = 0;
    time_t secs = time(NULL);
    struct tm *ptm = localtime(&secs);
    sprintf(image_name, "%04d%02d%02d%02d%02d",
            ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday,
            ptm->tm_hour, ptm->tm_min);
    dir = "images/";
}

void snapshots::snap_laser(Point p)
{
    make_snapshot(p, "laser");
    snap_bg(p);
}

void snapshots::snap_ant(Point p)
{
    make_snapshot(p, "ant");
    snap_bg(p);
}

void snapshots::snap_bg(Point p)
{
    bg_pt new_pt = { p, frame_index + 50 };
    bg_pts.push(new_pt);
}

void snapshots::make_snapshot(Point p, string tag)
{
    // Start with a big box to allow rotations
    if (half_size > p.x ||
        half_size > p.y ||
        p.x + half_size > pframe->cols ||
        p.y + half_size > pframe->rows) {
        
        printf("skipping %d %d\n", p.x, p.y);
        return;
    }
    Rect src_roi(p.x - half_size, p.y - half_size, size, size);
    Mat snap(*pframe, src_roi);
    char file_name[100];
    sprintf(file_name, "%s/%s_%s_%04u.png",
            tag.c_str(), tag.c_str(), image_name, snap_num++);
    imwrite(dir + string(file_name), snap);
    if(!bg_pts.empty() && bg_pts.front().frame < frame_index) {
        bg_pt cur = bg_pts.front();
        bg_pts.pop();
        make_snapshot(cur.p, "bg");
    }   
}

// See Caffe, examples/cpp_classification/classification.cpp

class Classifier {
    public:
        Classifier(const string& model_file, const string& trained_file);
        std::vector<float> Classify(const cv::Mat& img);

    private:
        shared_ptr<Net<float> > net_;
        cv::Size input_geometry_;
        int num_channels_;
};

Classifier::Classifier(const string& model_file,
                       const string& trained_file)
{
    Caffe::set_mode(Caffe::GPU);

    /* Load the network. */
    net_.reset(new Net<float>(model_file, TEST));
    net_->CopyTrainedLayersFrom(trained_file);

    CHECK_EQ(net_->num_inputs(), 1) << "Network should have exactly one input.";
    CHECK_EQ(net_->num_outputs(), 1) << "Network should have exactly one output.";

    Blob<float>* input_layer = net_->input_blobs()[0];
    num_channels_ = input_layer->channels();
    CHECK(num_channels_ == 1) << "Input layer should have 1 channel";
    input_geometry_ = cv::Size(input_layer->width(), input_layer->height());
    CHECK(input_geometry_ == cv::Size(IMG_SIZE, IMG_SIZE)) << "Input layer height, width wrong";
}

std::vector<float> Classifier::Classify(const cv::Mat& img)
{
    Blob<float>* input_layer = net_->input_blobs()[0];
    input_layer->Reshape(1, num_channels_,
                         input_geometry_.height, input_geometry_.width);
    /* Forward dimension change to all layers. */
    net_->Reshape();

    /*
     * Map dest Matrix data directly to input layer
     * so that converTo fills it.
     */
    float* input_data = input_layer->mutable_cpu_data();
    cv::Mat dest(IMG_SIZE, IMG_SIZE, CV_32FC1, input_data);
    img.convertTo(dest, CV_32FC1, 1, 0);

    net_->ForwardPrefilled();

    /* Copy the output layer to a std::vector */
    Blob<float>* output_layer = net_->output_blobs()[0];
    const float* begin = output_layer->cpu_data();
    const float* end = begin + output_layer->channels();
    return std::vector<float>(begin, end);
}

image_classifier::image_classifier()
{
    ::google::InitGoogleLogging("units");

    string model_file   = "/home/rgb/caffe/examples/ants/lenet_deploy.prototxt";
    string trained_file = "/home/rgb/caffe/examples/ants/lenet_iter_20000.caffemodel";
    pclass = new Classifier(model_file, trained_file);
}

std::vector<float> image_classifier::get_image_type(Mat *pframe, Point p)
{


    if (IMG_SIZE/2 > p.x ||
        IMG_SIZE/2 > p.y ||
        p.x + IMG_SIZE/2 > pframe->cols ||
        p.y + IMG_SIZE/2 > pframe->rows) {

        printf("image_classifier: bad point %d %d\n", p.x, p.y);
        return std::vector<float> (3, 0.0f);
    }

    Rect src_roi(p.x - IMG_SIZE/2, p.y - IMG_SIZE/2, IMG_SIZE, IMG_SIZE);
    Mat img(*pframe, src_roi);

    std::vector<float> retv = pclass->Classify(img);

    if (verbose)
        printf("image_classifier: ant %5.3f, laser %5.3f, bg %5.3f\n",
               retv[ant_index], retv[laser_index], retv[bg_index]);

    return retv;
}
