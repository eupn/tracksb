#!/bin/bash
tail -f motion_log.txt | { count=0; old=$(date +%s); while read line; do ((count++)); s=$(date +%s); if [ "$s" -ne "$old" ]; then echo "$count lines per second"; count=0; old=$s; fi; done; }
