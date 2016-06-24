#!/bin/bash

sudo wakeonlan -i 10.0.0.255 -p 7 c0:3f:d5:6b:d6:85
ping 10.0.0.2
