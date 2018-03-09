# Activity Daily Living (ADL) Error Detection

## Requirements
Install the following python modules:
```
sudo pip install pika
sudo pip install configparser
```

This repository has some encrypted files. You will need `git-crypt` installed and the key to decrypt the files.
```
sudo apt install git-crypt
```

### Error Detector
Run the error detection node.
```
rosrun adl_error_detection error_detector.py
```
