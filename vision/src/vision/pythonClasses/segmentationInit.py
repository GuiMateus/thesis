import torch
import cv2
import sys
from .imageManipulation import imageManipulation
from .acquireData import acquireImage
from .pointCloudProcessing import pointCloudProcessing
from .deeplab.modeling.deeplab import *
from PIL import Image
from torchvision import transforms
from .deeplab.dataloaders.utils import  *
from torchvision.utils import make_grid, save_image
from .deeplab.dataloaders import custom_transforms as tr
from . import img
from tensorboardX import SummaryWriter



class segmentationInit():
    """Initialize the segmentation node
    """
    def __init__(self):
        """segmentationInit class constructor
        """
        self.crops = []
        self.deepLabDimensions = (256,256)
        self.newBbox = []
        self.mean=(0.485, 0.456, 0.406)
        self.std=(0.229, 0.224, 0.225)



    def deeplabInit(self):
        """Initialize DeepLabV3

        Returns:
            DeepLab, Compose: Network model, Normalization method
        """
        # Load the DeepLabV3 model with a Resnet101 backbone
        model = DeepLab(num_classes=9,
            backbone='resnet',
            output_stride=8,
            sync_bn=None,
            freeze_bn=True)
        
        print(model)

        # Load the weights for DeepLabV3 trained on different types of screws


        

        weights = torch.load('/home/gui/Documents/dynamicEnvironment/model_best.pth.tar', map_location='cpu')['state_dict']

        from collections import OrderedDict
        new_state_dict = OrderedDict()
        for k, v in weights.items():
            name = k.replace("module.", "") # remove 'module.' of dataparallel
            new_state_dict[name]=v

        state_dict = new_state_dict

        model.load_state_dict(state_dict)
        
        model.eval()

        ##### PC CANT HANDLE WHILE RUNNING YOLO ALSO, BIG SAD :(((((( #####
        # if torch.cuda.is_available():
        #     model = model.to('cuda')
        # print("4")
        
        # Create a compose on how the input images should be normalized
        
        return model


    def inference(self, model, tensorArray):
        """Inference for semantic segmentation

        Args:
            model (DeepLab): Network model with weights
            tensorArray (torch.tensor): Cropped image in the form of tensors
            i ([type]): [description]
        """
        ##### PC CANT HANDLE WHILE RUNNING YOLO ALSO, BIG SAD :(((((( #####
        # if torch.cuda.is_available():
        #     self.tensor_in = self.tensor_in.cuda()

    
        masks = []
        # Run inference on all images
        for tensor in tensorArray:
            with torch.no_grad():
                normalizedMask = []
                output = model(tensor[0])

                experiments = make_grid(decode_seg_map_sequence(torch.max(output[:3], 1)[1].detach().cpu().numpy(), dataset="pascal"), 3, normalize=False, range=(0, 255))


                numpyMask = experiments.numpy()
                print("inference")
                
                finalMask = numpyMask.transpose([1, 2, 0])
                finalMask = finalMask[np.newaxis, :]
                masks.append(finalMask)

        return masks


    def toImgCoord(self, masks, depth, feedback):
        """Transforms masks to original image frame. This function is very messy and out of place.

        Args:
            masks (mask[]): Array of masks
            feedback (np.array): Image with bounding boxes

        Returns:
            np.array: Image with bounding boxes and masks.
            np.array: Array containing individual masks for further processing.
        """

        maskArray = []
        im = imageManipulation()
        pc = pointCloudProcessing()
        ai = acquireImage()
        i = 0
        # Check if masks exist
        if len(masks) is not 0:
            # Check every mask/detection
            for detection in self.crops:

                # Extract bounding box
                xmin, ymin, xmax, ymax = im.extractBbox(detection[3])
                
                # Turn current mask into a np.array of uint8 and trasnform it into grey
                currentMask = masks[i][0].astype('uint8') * 255
                
                currentMask = cv2.resize(currentMask, (int(xmax-xmin), int(ymax-ymin)))

                maskGrey = cv2.cvtColor(currentMask,cv2.COLOR_BGR2GRAY)
                print("to image!!")

                # Check if mask is not empty
                if cv2.countNonZero(maskGrey) is not 0:

                    # Create a binary mask
                    _, mask = cv2.threshold(maskGrey, 10, 255, cv2.THRESH_BINARY)
                    maskInv = cv2.bitwise_not(mask)

                    # Find the ROI where the screw mask belongs
                    roi = feedback[ymin:ymax, xmin:xmax]

                    # Create ROI and extract mask
                    if roi.shape[0] == maskInv.shape[0] and roi.shape[1] == maskInv.shape[1]:
                        if roi is not None and maskInv is not None:
                            imgROI = cv2.bitwise_and(roi,roi, mask=maskInv)
                            extractMask = cv2.bitwise_and(currentMask,currentMask, mask = mask)
                            # croppedDepth = im.cropImage(depth, xmin, ymin, w, h)
                            maskArray.append(extractMask)
                            dst = cv2.add(imgROI, extractMask)
                            feedback[ymin:ymax, xmin:xmax] = dst
                i += 1
        return feedback, maskArray

    def handleObjectCropping(self, inputImage, detections):
        # depthImageCV = inputDepthImage.astype(np.float32)
        tempVec = []
        if detections is not None:
            for label, confidence, bbox in detections:
                croppedImage = self.cropObjects(inputImage, bbox)

                if croppedImage is not None and croppedImage.shape[0] > 10 and croppedImage.shape[1] > 10:
                    croppedImage = cv2.resize(croppedImage, self.deepLabDimensions)
                    self.crops.append([croppedImage, label, confidence, self.newBbox, inputImage.shape[0], inputImage.shape[1]])
                    label = str(label)
                    tempVec.append(croppedImage)
            return tempVec


    def cropObjects(self, image, bbox):
        """Crop objects from the input image using the detections from the object detector

        Args:
            image (np.array): Input image
            bbox (bbox): Object bounding box

        Returns:
            np.array: Cropped image
        """
        im = imageManipulation()
        x, y, w, h = im.extractBbox(bbox)

        x = int(round(x-(w/2)))
        y = int(round(y-(h/2)))

        cropped, self.newBbox = im.cropImage(image, x, y, w, h)

        return cropped


    def imageToTensor(self):
        """Create a tensors from the cropped images

        Args:
            normalize (Compose): Normalization of the tensors

        Returns:
            torch.tensor[]: Returns an array of tensors from the cropped regions 
        """

        tensorArray = []
        for croppedIndex in self.crops:
            # Convert arrays into PIL.image
            # Create tensor
            tensor = self.normalizeImages(croppedIndex[0])
            tensor = self.toTensor(tensor).unsqueeze(0)
            tensorArray.append([tensor, croppedIndex[1], croppedIndex[2], croppedIndex[3], croppedIndex[4], croppedIndex[5]])
        return tensorArray

    def toTensor(self, img):
        # swap color axis because
        # numpy image: H x W x C
        # torch image: C X H X W
        img = img.astype(np.float32).transpose((2, 0, 1))

        img = torch.from_numpy(img).float()
        
        return img

    def normalizeImages (self, img):
        img = img.astype(np.float32)
        img /= 255.0
        img -= self.mean
        img /= self.std
        return img

        
    
    def getCrops(self):
        return self.crops