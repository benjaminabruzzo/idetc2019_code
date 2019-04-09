#!/bin/bash
  rm *.m~

  REMOVE=($(git status | grep deleted | awk '{ print $2}'))
    for i in "${REMOVE[@]}"
    do :
      git rm $i
    done

  git rm *.m~
  git add *.m *.md *.sh
  git add ./ugvStereoFunctions/*.m
  git add ./archive/*.m

  git commit -m "$2"
  git push $1 master
