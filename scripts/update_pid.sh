#!/bin/bash

# Publish a PID update message

if [[ $# != 3 ]]; then
    :
elif ! [[ $1 =~ [0-9]+.?[0-9]? ]]; then
    echo "$1 is non-numeric"
    exit 1
elif ! [[ $2 =~ [0-9]+.?[0-9]? ]]; then
    echo "$2 is non-numeric"
    exit 1
elif ! [[ $3 =~ [0-9]+.?[0-9]? ]]; then
    echo "$3 is non-numeric"
    exit 1
elif [[ $(bc <<<"(($1 > 20) + ($2 > 20) + ($3 > 20) + ($1 < 0) + ($2 < 0) + ($3 < 0))>0") == 1 ]]; then 
    echo "Value out of bounds (valid range 0-20)"
    exit 1
fi

echo "Updating PID values to P: $1, I: $2, D: $3"

# valid range 0 - 20, increments of 0.1

rostopic pub -1 /tycho/pid std_msgs/Float32MultiArray "layout:
  dim: []
  data_offset: 0
data: [$1, $2, $3]"
