CUDA_VISIBLE_DEVICES=0 python3 train.py --backbone resnet --lr 0.01 --workers 4 --epochs 28 --batch-size 16 --gpu-ids 0 --checkname deeplab-resnet --eval-interval 1 --dataset coco
