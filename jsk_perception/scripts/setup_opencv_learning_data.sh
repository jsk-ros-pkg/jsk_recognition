#!/bin/sh

if [ "$1" != "" ]; then
    TARGET_DIR=$1
else
    TARGET_DIR=training_dir
fi
git clone https://github.com/garaemon/TrainingAssistant.git $TARGET_DIR -b patched-head
cd $TARGET_DIR
git submodule init
git submodule update
(cd static/Jcrop; git checkout master)
# generating negative data
svn checkout http://tutorial-haartraining.googlecode.com/svn/trunk/data/negatives negative_images
find negative_images -name '*jpg' > bg.txt
