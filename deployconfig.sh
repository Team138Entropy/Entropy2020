#!/bin/sh
scp -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null src/main/deploy/config.properties admin@10.1.38.2:/home/lvuser/deploy/config.properties