#!/bin/bash

screen -dmS roscore ~/scripts/start-roscore.sh
sleep 10
screen -dmS robotcore ~/scripts/start-robotcore.sh
