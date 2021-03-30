#!/usr/bin/env bash

cd /opt
sudo mkdir vision
cd vision
sudo mkdir weights
cd weights

sudo mkdir deeplab
cd deeplab
sudo wget --no-check-certificate "https://onedrive.live.com/download?cid=B7DCBFD795E0B204&resid=B7DCBFD795E0B204%212745&authkey=APzYHuqFiJMXYkg"
sudo mv "/opt/vision/weights/deeplab/download?cid=B7DCBFD795E0B204&resid=B7DCBFD795E0B204!2745&authkey=APzYHuqFiJMXYkg" "model_best.pth.tar"

cd ..
sudo mkdir yolov4
cd yolov4
sudo wget --no-check-certificate "https://onedrive.live.com/download?cid=B7DCBFD795E0B204&resid=B7DCBFD795E0B204%212746&authkey=AOan3WK5lY1cZro"
sudo mv "/opt/vision/weights/yolov4/download?cid=B7DCBFD795E0B204&resid=B7DCBFD795E0B204!2746&authkey=AOan3WK5lY1cZro" "pumps-v2.data"
sudo wget --no-check-certificate "https://onedrive.live.com/download?cid=B7DCBFD795E0B204&resid=B7DCBFD795E0B204%212747&authkey=AGwJMFh6BCASi7E"
sudo mv "/opt/vision/weights/yolov4/download?cid=B7DCBFD795E0B204&resid=B7DCBFD795E0B204!2747&authkey=AGwJMFh6BCASi7E" "yolov4-pumps-v2.cfg"
sudo wget --no-check-certificate "https://onedrive.live.com/download?cid=B7DCBFD795E0B204&resid=B7DCBFD795E0B204%212748&authkey=AGM6m4QtG2CDItI"
sudo mv "/opt/vision/weights/yolov4/download?cid=B7DCBFD795E0B204&resid=B7DCBFD795E0B204!2748&authkey=AGM6m4QtG2CDItI" "yolov4-pumps-v2_final.weights"

cd

mkdir .environmnetReconstruction