import cv2
import numpy as np
from ultralytics import YOLO
import os
from shapely.geometry import Polygon, box
import time
import torch

class Detector:
    """
    A base class for implementing object segmentation and detection for Yolov8-seg

    Attributes:
        model_path: (required) path of the yolo8-seg pt model file
        isMax: (optional) only retrieve the detected objects with biggest area
        log: (optional) log object for verbose

    Methods:
        detect_n_seg: perform detection and segmentation which result in list of DetectedObject object 
        remove_bg: remove background of chosen detected object
        crop: crop bounding box of chosen detected object
        get_label_idx: get index number of certain classses
        conv_seg: convert the result of raw inference to be formatted data
    
    Examples:
        VehicleDetector = Detector("path/to/yolo8-seg_model.pt")
        image = cv2.imread("path/to/image.jpg")
        result = VehicleDetector.detect_n_seg(image, labels=["car", "bike"], score_threshold=0.5)
    """
    def __init__(self, model_path:str):
        """
        initialize a new instance of Detector class.
        This constructor sets up the model based on the provided model path or name.

        Raises:
            FileNotFoundError: If the specified model file does not exist or is inaccessible.
            ValueError: If the model file or configuration is invalid or unsupported.
            ImportError: If required dependencies for specific model types (like HUB SDK) are not installed.

        Examples:
            >>> CarDetector = Detector("yolo8-seg.pt")
            >>> VehicleDetector = Detector("yolo8-seg.pt")
        """
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file '{model_path}' not found.")
        self.extension = model_path.split('.')[-1].lower()
        if self.extension == 'pt':
            self.model = YOLO(model_path)
        basePath = os.path.dirname(model_path)
        nameModel,_ = os.path.splitext(os.path.basename(model_path))
        labelsPath = os.path.join(basePath, f"{nameModel}.txt")
        with open(labelsPath) as f:
            self.classes = f.read().split("\n")
    
    def get_label_idx(self, labels:list):
        if len(labels) > 0:
            clss = []
            for label in labels:
                if label in self.classes:
                    clss.append(self.classes.index(label))
                else:
                    raise ValueError(f'{label} Tidak Ditemukan')
        else:
            clss = None
        return clss

    def crop(self, image, box):
        xmin, ymin, xmax, ymax = box[:4]
        crop_face = image[ymin:ymax, xmin:xmax]
        return crop_face

    def detect_n_seg(self, image:np.ndarray, labels:list=[], score_threshold:float=0.5, stream=True):
        if len(labels) == 0: labels = self.classes
        clss = self.get_label_idx(labels)
        results = self.model.predict(image, conf=score_threshold, stream=stream, classes=clss, verbose=False, save=False)
        results = self.convert_inference(results)
        return results
    
    def find_max_area(self, object_list, label):
        max_obj = max((obj for obj in object_list if obj.object_class == label), key=lambda obj: obj.area, default=None)
        object_list = [max_obj] if max_obj else []
        return object_list

    def remove_bg(self, image, results, labels, main_label):
        mask = np.zeros_like(image.copy())
        for label in labels:
            if label in results:
                points = np.array(results[label]['maskxy'], np.int32)
                points = points.reshape((-1, 1, 2))
                cv2.fillPoly(mask, [points], (255, 255, 255))

        image_rm = cv2.bitwise_and(image, mask)
        return self.crop(image_rm, results[main_label]['box'])
    
    def convert_inference(self, detections):
        object_list = []
        for result in detections:
            if result.boxes == None:
                continue
            xyxy = result.boxes.xyxy
            _classes = result.boxes.cls
            _confs = result.boxes.conf

            for idx, data in enumerate(_classes):
                _cls = self.classes[int(data)]
                x_min, y_min, x_max, y_max = xyxy[idx]
                x_min, y_min, x_max, y_max = int(x_min), int(y_min), int(x_max), int(y_max)
                detected_object = DetectedObject()
                detected_object.id = idx
                detected_object.object_class = _cls
                detected_object.conf = float(_confs[idx])
                detected_object.box = [x_min, y_min, x_max, y_max]
                detected_object.area = (y_max-y_min)*(x_max-x_min)
                if not result.masks == None:
                    if isinstance(result.masks[idx].xy[0], list):
                        detected_object.maskxy = result.masks[idx].xy[0]
                    else:
                        detected_object.maskxy = result.masks[idx].xy[0].tolist()
                object_list.append(detected_object)
        return object_list
    
    def mask2polygon(self, maskxy):
        points = np.array(maskxy, np.int32)
        polygon = Polygon(points)
        return polygon
    
    def box2polygon(self, bbox):
        return box(bbox[0], bbox[1], bbox[2], bbox[3])
        
    def isIntersect(self, main:Polygon, child:Polygon):
        return main.intersects(child)
    
    def create_context(self):
        self.model.create_context()
        
class DetectedObject:

    def __init__(self, id=None, object_class=None, conf=None, box=None, maskxy=None, area=None):
        self.id = None
        self.object_class = object_class
        self.conf = conf
        self.box = box
        self.maskxy = maskxy
        self.area = area

    def __str__(self):
        return f"id : {self.id}, labels: {self.object_class}, conf: {self.conf}, box: {self.box}"