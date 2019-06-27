#!/usr/bin/env python

from __future__ import print_function

import logging
import os
import sys


def assertParam(options, parameter_name):
    if parameter_name not in options:
        raise ValueError(parameter_name + " not set")


def checkParam(options, parameter_name, default_value):
    if parameter_name not in options:
        options[parameter_name] = default_value
        logger = logging.getLogger(__name__)
        logger.info("Setting default value for parameter '" + parameter_name +
                    "' = " + str(default_value))


def userYesNoQuery(question):
    while True:
        val = raw_input(question + " - Yes o No?\n").lower()
        if val == 'y' or val == 'yes':
            return True
        elif val == 'n' or val == 'no':
            return False
        else:
            sys.stdout.write('Please respond with \'y\' or \'n\'.\n')


def findFileOrDir(root_folder, base_folder, file_name):
    """Try to find a given file or directory.

    This will look for file_name in the current directory, in
    <root_folder>/<file_name> and <root_folder>/<base_folder>/<file_name> and
    <root_folder>/../<base_folder>/<file_name>.

    Returns the path of the file on the disk if it was found, otherwise an
    exception is raised.
    """
    if os.path.isfile(file_name) or os.path.isdir(file_name):
        return file_name

    file_name_to_try_1 = os.path.join(root_folder, file_name)
    if os.path.isfile(file_name_to_try_1) or os.path.isdir(file_name_to_try_1):
        return file_name_to_try_1

    file_name_to_try_2 = os.path.join(root_folder, base_folder, file_name)
    if os.path.isfile(file_name_to_try_2) or os.path.isdir(file_name_to_try_2):
        return file_name_to_try_2

    file_name_to_try_3 = os.path.join(
        os.path.dirname(root_folder), base_folder, file_name)
    if os.path.isfile(file_name_to_try_3) or os.path.isdir(file_name_to_try_3):
        return file_name_to_try_3

    raise Exception(
        'Unable to find the file "' + file_name + '". Checked in:\n- ' +
        file_name + '\n- ' + file_name_to_try_1 + '\n -' + file_name_to_try_2 +
        '\n- ' + file_name_to_try_3)
