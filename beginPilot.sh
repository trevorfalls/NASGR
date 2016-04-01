#!/bin/bash

screen -dmS connection ~/scripts/begin-connection.sh
sleep 1
screen -dmS init ~/git/NASGR/Guides/XBOX/init_xbox.sh
sleep 1
screen -dmS xbox ~/scripts/begin-xbox.sh
sleep 1
screen -dmS pilot ~/scripts/begin-pilot.sh
rm ~/git/NASGR/qtworkspace/qdude/src/CMakeLists.txt.user
