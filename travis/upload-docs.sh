#!/bin/bash

set -x
export GIT_COMMITTER_NAME=$GIT_NAME
export GIT_COMMITTER_EMAIL=$GIT_EMAIL
export GIT_AUTHOR_NAME=$GIT_NAME
export GIT_AUTHOR_EMAIL=$GIT_EMAIL
export REPOSITORY_NAME=${PWD##*/}
git clone https://github.com/jsk-ros-pkg/euslisp-docs.git ~/euslisp-docs
cd ~/euslisp-docs/docs
for pkg in ~/ros/ws_$REPOSITORY_NAME/build/*; do
    if ls $pkg/*.md > /dev/null 2>&1; then
        name=`basename $pkg`;
        echo $name;
        ls -al $pkg/*.md
        mkdir -p $name
        cp $pkg/*.md $name/
        git add $name/*.md
        git commit -m "update documentation for $name" -a
    fi
done
git push --quiet https://$GH_TOKEN@github.com/jsk-ros-pkg/euslisp-docs.git master
