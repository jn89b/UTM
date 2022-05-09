#!/usr/bin/env python


#import setup_path
import airsim
import os
import re
import time 
#from airsim.client import VehicleClient


"""
lqr into wind: -9,-9
px4 into wind: 9, 9

lqr against wind: 9, 9
px4 against wind: -9,-9 
"""

if __name__=='__main__':
	wsl_ip = os.getenv('WSL_HOST_IP')
	client = airsim.VehicleClient(ip=str(wsl_ip))

	# Just change the below to test different cameras easily!
	CAM_NAME = "front_camera"
	IS_EXTERNAL_CAM = False
 
	distort_vals = [-0.002049, 0.002078, 0.000474, -0.000015, 0.000000]
 
	new_params_dict = {"K1": distort_vals[0], "K2": distort_vals[1], "K3": distort_vals[2],
                    "P1": distort_vals[3], "P2": distort_vals[4]}
 
	print(f"Setting distortion params as {new_params_dict}")
	client.simSetDistortionParams(CAM_NAME, new_params_dict)
 
	# Test Distortion params APIs
	dist_params = client.simGetDistortionParams(CAM_NAME, external=IS_EXTERNAL_CAM)
	print(f"Distortion Params: {dist_params}")