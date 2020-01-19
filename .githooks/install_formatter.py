import os
import sys
import requests

dl_path = f"{os.getcwd()}/.format"
url = "https://github.com/google/google-java-format/releases/download/google-java-format-1.7/google-java-format-1.7-all-deps.jar"
file_name = url.split("/")[-1]

if not os.path.exists(dl_path):
    os.mkdir(dl_path)

if not os.path.isdir(dl_path):
    sys.stderr.write(f"Error: {dl_path} exists but is not a directory.")
    sys.stderr.flush()
    exit(1)

if not os.path.exists(f"{dl_path}/{file_name}"):
    with open(f"{dl_path}/{file_name}", "wb") as f:
        print(f"Downloading {file_name}...")
        res = requests.get(url, stream=True)
        length = res.headers.get("content-length")

        print(res)

        if length is None:
            f.write(res.content)
        else:
            dl = 0
            for data in res.iter_content(chunk_size=4096):
                dl += len(data)
                f.write(data)
                exit(0) # Success!
elif not os.path.isfile(f"{dl_path}/{file_name}"):
    sys.stderr.write(f"Error: {file_name} already exists but is not a regular file.")
    sys.stderr.flush()
    exit(1)
else:
    print(f"Found existing {file_name}, skipping download...")
    exit(0)