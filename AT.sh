!#bash/bin

#OPEN DRIVER FOR CAMERA
sudo modprobe bcm2835-v4l2

#CUSTOM SETTINGS FOR PI CAM
v4l2-ctl -L
v4l2-ctl --set-ctrl=horizontal_flip=0
v4l2-ctl --set-ctrl=vertical_flip=1
v4l2-ctl --set-ctrl=color_effects=0

#RUN ARPIL TAGS
./build/bin/apriltagscsv -S .415 -F 13.6