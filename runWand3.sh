#!/bin/sh

echo running Wand3 on pi3 id 65
sleep 1s
./wand1 -rasp-id=65 --web-addr :8082 -no-duino -log-level debug
