#!/bin/bash
# makes train.txt and test.txt to feed to caffe training
cd=~/ants/vid
ant_dir=$cd/images/ant/
bg_dir=$cd/images/bg/
laser_dir=$cd/images/laser/
ls $bg_dir | sed -e s!^!${bg_dir}! -e "s/$/ 0/" > list
ls $ant_dir | sed -e s!^!${ant_dir}! -e "s/$/ 1/" >> list
ls $laser_dir | sed -e s!^!${laser_dir}! -e "s/$/ 2/" >> list
sort --random-sort list > list.sort
lc=`wc -l list | awk '{ print $1 }'`
test_len=`expr $lc / 4`
train_len=`expr $lc '*' 3 / 4`
head -n $train_len list.sort > train.txt
tail -n $test_len list.sort > test.txt
