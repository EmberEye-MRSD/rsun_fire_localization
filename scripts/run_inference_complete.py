# modules for the model
import torch
import torch.nn as nn
import torch.utils.data
from torchvision import transforms
from torch.utils.data import DataLoader
import cv2
import numpy as np
import argparse
import os
from tqdm import tqdm
import matplotlib.pyplot as plt
from datasets.MS2_dataset import DataLoader_MS2
from datasets.fireview_dataset import Dataloader_Fireview
from datasets.ms2_utils import (
    visualize_disp_as_numpy,
    visualize_depth_as_numpy,
    process_image,
    invNormalize,
    get_normalize,
    load_as_float_depth,
    load_fireview_img,
    align_contrast,
)
from models import __models__

import time

# ROS and additional modules
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError

# import modules for rectification
from rectify_image import RectifyImage
import message_filters



# TODO: Load from dataset
FOCAL_LENGTH = 406.33233091474426
BASELINE = 0.24584925266278748

parser = argparse.ArgumentParser(
    description="Inference script for Stereo Thermal depth estimation",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
)
parser.add_argument("--model", default="Fast_ACVNet_plus", choices=__models__.keys(), help="select a model structure")
parser.add_argument("--maxdisp", type=int, default=192, help="maximum disparity")
parser.add_argument("--dataset_dir", type=str, default="/media/devansh/T7 Shield/ms2")
parser.add_argument("--seq_name", type=str, default=None, help="sequence name")
parser.add_argument("--output_dir", default="./inference_results", type=str, help="Output directory")
parser.add_argument("--process", default="hist_99", help="data preprocess mode (raw, minmax, hist_99)")
parser.add_argument("--loadckpt", required=True, help="load the weights from a specific checkpoint")
parser.add_argument("--data_type", default="forest_st", choices=["ms2", "forest_st"], help="Which dataset to use")
parser.add_argument("--mode", default="disparity", choices=["disparity", "depth"], help="Visualize disparity or depth")
parser.add_argument("--visualize", action="store_true", help="Visualize the results")
parser.add_argument("--save", action="store_true", help="Save the results")

# Single image inference
"""
Expected root directory structure:
<dir>
    ├── img_left
    └── img_right
    └── depth_filtered (optional)

The dir must contain the left and right images with the same name <image.png>.
"""

parser.add_argument("--dir", type=str, default=None, help="Path to the left image")
parser.add_argument("--images", type=str, nargs="+", default=None, help="Name of the image")


class DepthEstimationModel:
    def __init__(self, args):
        self.args = args
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = self.load_model()
        self.inv_normalize = invNormalize()
        
        image_files = []
        image_directory = os.path.join(self.args.dir, "img_left")
        image_extensions = ['.png']

        for file in os.listdir(image_directory):
            if os.path.isfile(os.path.join(image_directory, file)):
                if any(file.lower().endswith(ext) for ext in image_extensions):
                    image_files.append(file)

        self.args.images = sorted(image_files)

        self.thermal_left_sub = message_filters.Subscriber('/thermal_left/image', Image)
        self.thermal_right_sub = message_filters.Subscriber('/thermal_right/image', Image)
        self.depth_pub = rospy.Publisher("/thermal_depth/image_raw", Float32MultiArray, queue_size=10)

        self.tolerance = 100e-03
        self.time_sync = message_filters.ApproximateTimeSynchronizer([self.thermal_left_sub, self.thermal_right_sub], queue_size=10, slop=self.tolerance)
        self.time_sync.registerCallback(self.rectify_callback)

        self.left_img = None
        self.right_img = None

        left_calibration_file = "/wildfire/development/embereye_ws/src/config/fake_thermal_left.yaml"
        right_calibration_file = "/wildfire/development/embereye_ws/src/config/fake_thermal_right.yaml"
        self.rectify = RectifyImage(left_calibration_file, right_calibration_file)
    
    def rectify_callback(self, left_image_msg, right_image_msg):
        try:
            left_image = CvBridge().imgmsg_to_cv2(left_image_msg, "16UC1")
            right_image = CvBridge().imgmsg_to_cv2(right_image_msg, "16UC1")
        except CvBridgeError as e:
            print(e)

        self.left_img, self.right_img, *_ = self.rectify(left_image, right_image)
        # print(self.left_img.shape, self.right_img.shape)
        # cv2.imshow("Left Image", self.left_img)
        # cv2.waitKey(1)

        # cv2.imshow("Right Image", self.right_img)
        # cv2.waitKey(1)

        # plt.imshow(self.left_img, cmap='gray')
        # plt.show()


    def load_model(self):
        model = __models__[self.args.model](self.args.maxdisp, False)
        model = nn.DataParallel(model).to(self.device)

        print(f"loading model {self.args.loadckpt}")
        state_dict = torch.load(self.args.loadckpt, map_location=self.device)
        model_dict = model.state_dict()
        pre_dict = {k: v for k, v in state_dict["model"].items() if k in model_dict}
        model_dict.update(pre_dict)
        model.load_state_dict(model_dict)
        model.eval()
        return model

    def run_inference(self):
        print("\n\nInferencing single image")
        if self.left_img is None or self.right_img is None:
            print("No images received yet")
            return

        imgL, imgR = self.load_and_process_images(self.left_img, self.right_img)
        imgL, imgR = imgL.unsqueeze(0), imgR.unsqueeze(0)
    
        with torch.no_grad():
            disp_ests = self.model(imgL.to(self.device), imgR.to(self.device))[0]
        # self.disp_pub.publish(CvBridge().cv2_to_imgmsg(visualize_disp_as_numpy(disp_ests[0]), "passthrough"))

        # publish depth image
        depth = self.disp2depth(disp_ests, focal=torch.as_tensor(FOCAL_LENGTH), baseline=torch.as_tensor(BASELINE))
        depth = depth.permute(1, 2, 0).cpu().numpy()

        # print(f"Depth: {depth}")

        # cv2.imshow("Depth Image", depth)
        # cv2.waitKey(1)
        print(depth.dtype)
        img = Float32MultiArray()
        img.data = [depth]
        # img = CvBridge().cv2_to_imgmsg(depth, "32FC1")
        # print(f"\nimg: {img}")
        self.depth_pub.publish(img)

        return

    def depth2disp(self, depth, focal, baseline):
        min_depth = 1e-3
        mask = depth < min_depth
        disp = baseline * focal / (depth + 1e-10)
        disp[mask] = 0.0
        return disp

    def disp2depth(self, disp, focal, baseline):
        mask = disp < 1e-3
        depth = baseline * focal / (disp + 1e-10)
        depth[mask] = 0.0
        return depth

    def load_depth_as_disp(self, depth_path):
        if not os.path.exists(depth_path):
            return None

        depth = load_as_float_depth(depth_path) / 256.0
        disp = self.depth2disp(
            depth,
            focal=torch.as_tensor(FOCAL_LENGTH),
            baseline=torch.as_tensor(BASELINE),
        )
        return torch.as_tensor(disp).unsqueeze(0)

    def load_and_process_images(self, left_img, right_img):
        if len(left_img.shape) == 2 and len(right_img.shape) == 2:  # for non-agc images
            left_img = np.expand_dims(left_img, axis=2)
            right_img = np.expand_dims(right_img, axis=2)
            agc = False
        else:
            agc = True

        if not agc:
            left_img, right_img = process_image(left_img, right_img, self.args.process)
            left_img = cv2.cvtColor(left_img, cv2.COLOR_GRAY2RGB)
            right_img = cv2.cvtColor(right_img, cv2.COLOR_GRAY2RGB)

        to_tensor = transforms.ToTensor()
        left_img = to_tensor(left_img.astype(np.uint8))
        right_img = to_tensor(right_img.astype(np.uint8))

        left_img, right_img = align_contrast(left_img, right_img)

        normalize = get_normalize()
        left_img = normalize(left_img)
        right_img = normalize(right_img)  # [3,h,w]

        return left_img, right_img




if __name__ == "__main__":
    rospy.init_node("depth_estimation_node")

    rate = rospy.Rate(100)

    args = parser.parse_args()
    depth_estimator = DepthEstimationModel(args)
    # rospy.sleep(1)

    while not rospy.is_shutdown():
        depth_estimator.run_inference()
        rate.sleep()

    rospy.spin()