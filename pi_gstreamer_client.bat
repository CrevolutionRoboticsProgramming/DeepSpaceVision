C:\gstreamer\1.0\x86_64\bin\gst-launch-1.0.exe -e -v udpsrc port=9001 ! ^
application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! ^
rtph264depay ! h264parse ! avdec_h264 ! ^
videoconvert ! autovideosink sync=false async=false
