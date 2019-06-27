#!/usr/bin/env python

from __future__ import print_function

import os
import subprocess
import yaml


def getCatkinConfig(profile="default"):
    print("TODO: this may fail if you are not inside a catkin package!")
    ws_path = subprocess.check_output(["catkin",
                                       "locate"]).split()[0].decode('ascii')
    CATKIN_TOOLS_SUBF = ".catkin_tools"
    config_yaml_path = os.path.join(ws_path, CATKIN_TOOLS_SUBF, profile,
                                    "config.yaml")
    if not os.path.exists(config_yaml_path):
        config_yaml_path = os.path.join(ws_path, CATKIN_TOOLS_SUBF, "profiles",
                                        profile, "config.yaml")
    if not os.path.exists(config_yaml_path):
        raise ValueError(
            "config_yaml_path does not exist: '{}'".format(config_yaml_path))
    in_file = open(config_yaml_path)
    config = yaml.safe_load(in_file)
    in_file.close()
    return config


def catkinFind(package_name):
    return subprocess.check_output(["catkin_find",
                                    package_name]).decode('ascii').split()


def catkinFindSubfolder(package_name, req_sub_folder):
    folder_list = catkinFind(package_name)
    for folder in folder_list:
        if req_sub_folder in folder:
            return folder.strip()
    raise ValueError("Could not find " + req_sub_folder +
                     " folder for the package " + package_name)


def catkinFindSrc(package_name):
    return catkinFindSubfolder(package_name, "src/")


def catkinFindLib(package_name):
    return catkinFindSubfolder(package_name, "devel/lib")


def catkinFindCamCalib(ze_cam_id):
    ze_calib_folders = catkinFind("ze_calibration")
    for folder in ze_calib_folders:
        cam_calib_folder = os.path.join(folder, str(ze_cam_id))
        if os.path.exists(cam_calib_folder) and os.path.isdir(
                cam_calib_folder):
            return cam_calib_folder
    raise ValueError(
        "Could not find calibration folder for ZE camera id " + str(ze_cam_id))


def catkinFindTestData(req_sub_folder):
    ze_test_data_folders = catkinFind('ze_test_data')
    for folder in ze_test_data_folders:
        data_folder = os.path.join(folder, 'data', req_sub_folder)
        if os.path.exists(data_folder) and os.path.isdir(data_folder):
            return data_folder
    raise ValueError("Could not find ZE data folder for " + req_sub_folder)


def catkinFindExperimentsFolder():
    ze_test_data_folders = catkinFind('ze_experiments')
    for folder in ze_test_data_folders:
        experiments_folder = os.path.join(folder, 'experiments')
        if os.path.exists(experiments_folder) and os.path.isdir(
                experiments_folder):
            return folder
    return ''


def getRevString(cwd_folder):
    rev_cmd = ["git", "rev-parse", "HEAD"]
    out_lines = subprocess.check_output(
        rev_cmd, cwd=cwd_folder).decode('ascii').split()
    if len(out_lines) != 1:
        raise ValueError("Subprocess call returned wrong number of lines")
    return out_lines[0]


def getSrcRevision(package_name):
    src_folder = catkinFindSrc(package_name)
    return getRevString(src_folder)


def getCalibRevision(ze_cam_id):
    cam_calib_folder = catkinFindCamCalib(ze_cam_id)
    return getRevString(cam_calib_folder)


def getTestDataRevision(req_sub_folder):
    data_folder = catkinFindTestData(req_sub_folder)
    return getRevString(data_folder)
