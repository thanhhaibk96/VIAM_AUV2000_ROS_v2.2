#!/bin/bash

#ffmpeg -loglevel debug -rtsp_transport tcp -i "rtsp://169.254.219.239:554/user=admin_password=tlJwpbo6_channel=1_stream=0.sdp?real_stream" \-c copy -map 0 /home/hai_bker96/testfffstream.mp4

username=$(whoami)
datetime=$(date "+%d_%m_%Y__%H_%M_%S")
echo "$username record video from $datetime"
ffmpeg -loglevel debug -rtsp_transport tcp -i "${1}" \-c copy -map 0 /home/$username/${2}_$datetime.mp4
