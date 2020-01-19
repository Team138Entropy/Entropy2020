#!/usr/bin/env python3
import os
import sys

dl_path = f"{os.getcwd()}/.checkstyle"

if not os.path.exists(dl_path):
    os.mkdir(dl_path)

if not os.path.isdir(dl_path):
    sys.stderr.write(f"Error: {dl_path} exists but is not a directory.")
    sys.stderr.flush()
    exit(1)

