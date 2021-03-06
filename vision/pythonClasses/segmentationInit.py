import torch
import cv2
from .imageManipulation import imageManipulation
from .deeplab.modeling.deeplab import *
from PIL import Image
from torchvision import transforms
from .deeplab.dataloaders.utils import  *
from torchvision.utils import make_grid, save_image
from .deeplab.dataloaders import custom_transforms as tr



class segmentationInit():
    """Initialize the segmentation node
    """
    def __init__(self):
        """segmentationInit class constructor
        """
        self.crops = []


    def deeplabInit(self):
        """Initialize DeepLabV3

        Returns:
            DeepLab, Compose: Network model, Normalization method
        """
        # Load the DeepLabV3 model with a Resnet101 backbone
        model = DeepLab(num_classes=3,
            backbone='resnet',
            output_stride=8,
            sync_bn=False,
            freeze_bn=False)

        # Load the weights for DeepLabV3 trained on different types of screws
        weights = torch.load('/home/gui/Documents/data/model_best.pth.tar', map_location='cpu')
        model.load_state_dict(weights['state_dict'])
        model.eval()

        ##### PC CANT HANDLE WHILE RUNNING YOLO ALSO, BIG SAD :(((((( #####
        # if torch.cuda.is_available():
        #     model = model.to('cuda')
        # print("4")
        
        # Create a compose on how the input images should be normalized
        normalizeImage = transforms.Compose([
            tr.Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225)),
            tr.ToTensor()])
        return model, normalizeImage


    def inference(self, model, tensorArray, i):
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
                # print(type(tensor[0]))
                output = model(tensor[0])
                outputMax = decode_seg_map_sequence(torch.max(output[:3], 1)[1].detach().cpu().numpy())
                masks.append(outputMax)
        return masks


    def toImgCoord(self, masks, feedback):
        """Transforms masks to original image frame. This function is very messy and out of place.

        Args:
            masks (mask[]): Array of masks
            feedback (np.array): Image with bounding boxes

        Returns:
            np.array: Image with bounding boxes and masks.
        """
        im = imageManipulation()
        i = 0
        # Check if masks exist
        if len(masks) is not 0:
            # Check every mask/detection
            for detection in self.crops:
                # Extract bounding box
                x, y, w, h = im.extractBbox(detection[3])
                
                # Turn current mask into a np.array of uint8 and trasnform it into grey
                currentMask = masks[i][0].astype('uint8') * 255
                maskGrey = cv2.cvtColor(currentMask,cv2.COLOR_BGR2GRAY)

                # Check if mask is not empty
                if cv2.countNonZero(maskGrey) is not 0:
                    
                    # Create a binary mask
                    _, mask = cv2.threshold(maskGrey, 10, 255, cv2.THRESH_BINARY)
                    maskInv = cv2.bitwise_not(mask)

                    # Find the ROI where the screw mask belongs
                    roi = feedback[int(y-h/2):int(y-h/2)+maskInv.shape[0], int(x-w/2):int(x-w/2)+maskInv.shape[1]]

                    # Create ROI and extract mask
                    imgROI = cv2.bitwise_and(roi,roi, mask=maskInv)
                    extractMask = cv2.bitwise_and(currentMask,currentMask, mask = mask)

                    dst = cv2.add(imgROI, extractMask)
                    feedback[int(y-h/2):int(y-h/2)+maskInv.shape[0], int(x-w/2):int(x-w/2)+maskInv.shape[1]] = dst
                i += 1
        return feedback


    def handleObjectCropping(self, inputImage, detections):
        if detections is not None:
            for label, confidence, bbox in detections:
                if label == "Screw":
                    croppedImage = self.cropObjects(inputImage, bbox)
                    if croppedImage is not None and croppedImage.shape[0] > 10 and croppedImage.shape[1] > 10:
                        self.crops.append([croppedImage, label, confidence, bbox, inputImage.shape[0], inputImage.shape[1]])
                        label = str(label)


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

        cropped = im.cropImage(image, x, y, w, h)
        return cropped


    def imageToTensor(self, normalize):
        """Create a tensors from the cropped images

        Args:
            normalize (Compose): Normalization of the tensors

        Returns:
            torch.tensor[]: Returns an array of tensors from the cropped regions 
        """
        tensorArray = []
        for croppedIndex in self.crops:
            # Convert arrays into PIL.image
            image = Image.fromarray(croppedIndex[0]).convert('RGB')
            target = Image.fromarray(croppedIndex[0]).convert('L')

            # Create tensor
            sample = {'image': image, 'label': target}
            tensor = normalize(sample)['image'].unsqueeze(0)
            tensorArray.append([tensor, croppedIndex[1], croppedIndex[2], croppedIndex[3], croppedIndex[4], croppedIndex[5]])
        return tensorArray