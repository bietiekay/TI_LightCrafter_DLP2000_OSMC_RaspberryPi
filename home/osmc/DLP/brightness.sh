#! /bin/bash

cd /home/osmc/DLP/

#echo blah >>running.txt

echo sets brightness - 0 - 1000

sudo python LEDSet.py $1 $1 $1
