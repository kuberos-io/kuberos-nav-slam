#!/bin/bash

while [ true ]; do
    topic_list=$(ign topic -l)
    ret=$?
    if [[ "$topic_list" == *"clock"* ]]; then
      echo "Simulation clock topic found."
      exit 0
    else
      echo "Waiting for Simulation clock topic..."
    fi
    sleep 1.0
done

exit 0