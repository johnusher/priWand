#!/bin/sh

echo running Wand1 on pi4
sleep 1s
./wand1 -rasp-id=67 --web-addr :8082 -no-duino -log-level debug
