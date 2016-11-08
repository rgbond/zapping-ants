# Copyright 2016 Robert Bond
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
# http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

libs  = -L/usr/local/lib
libs += -L/usr/local/cuda-6.5/targets/armv7-linux-gnueabihf/lib
libs += -L/usr/local/caffe/lib
libs += -lcaffe
libs += -lglog
libs += -lprotobuf
libs += -lopencv_calib3d
libs += -lopencv_contrib
libs += -lopencv_core
libs += -lopencv_features2d
libs += -lopencv_flann
libs += -lopencv_gpu
libs += -lopencv_highgui
libs += -lopencv_imgproc
libs += -lopencv_legacy
libs += -lopencv_ml
libs += -lopencv_nonfree
libs += -lopencv_objdetect
libs += -lopencv_ocl
libs += -lopencv_photo
libs += -lopencv_stitching
libs += -lopencv_superres
libs += -lopencv_ts
libs += -lopencv_video
libs += -lopencv_videostab
libs += -lcufft
libs += -lnpps
libs += -lnppi
libs += -lnppc
libs += -lcudart
libs += -lrt
libs += -lpthread
libs += -lm
libs += -ldl  

inc  = -I/usr/local/include/opencv
inc += -I/usr/local/include
inc += -I/usr/local/caffe/include
inc += -I/usr/local/cuda-6.5/targets/armv7-linux-gnueabihf/include

all: units xytest
clean:
	rm units xytest
units.o: units.cpp hw.h ants.h player.h util.h neuro.h 
	g++ -ggdb $(inc) -c units.cpp 
hw.o: hw.cpp hw.h
	g++ -ggdb $(inc) -c hw.cpp 
ants.o: ants.cpp hw.h ants.h blobs.h util.h neuro.h
	g++ -ggdb $(inc) -c ants.cpp 
blobs.o: blobs.cpp hw.h blobs.h util.h
	g++ -ggdb $(inc) -c blobs.cpp 
player.o: player.cpp player.h hw.h ants.h util.h neuro.h
	g++ -ggdb $(inc) -c player.cpp 
util.o: util.cpp util.h hw.h
	g++ -ggdb $(inc) -c util.cpp 
neuro.o: neuro.cpp neuro.h
	g++ -ggdb $(inc) -c neuro.cpp 
units: units.o hw.o ants.o blobs.o player.o util.o neuro.o
	g++ -ggdb -o units units.o hw.o ants.o blobs.o player.o util.o neuro.o $(libs)
xytest.o: xytest.cpp hw.h
	g++ -ggdb $(inc) -c xytest.cpp 
xytest: xytest.o hw.o
	g++ -ggdb -o xytest xytest.o hw.o $(libs)
