#!/bin/zsh

for i in camera1/*.png
do
    echo "Processing frame $(basename ${i})"...
    convert \
        \( ${i} -crop '800x900+500+30' \) \
        \( ${i/camera1/camera2} -crop '800x900+620+100' \) \
        +append \
        -draw 'line 800,0 800,900' \
        ${i/camera1/split}
done
