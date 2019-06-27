#!/usr/bin/env python

from __future__ import print_function

import os

import nose.tools

from evaluation_tools.catkin_utils import catkinFindSrc
from evaluation_tools.job import Job
from evaluation_tools.run_experiment import Experiment

RESULTS_FOLDER = './results'
AUTOMATIC_DATASET_DOWNLOAD = True


def _create_jobs():
    experiment_path = os.path.join(
        catkinFindSrc('evaluation_tools'), 'experiments',
        'sample_experiment.yaml')
    assert os.path.isfile(
        experiment_path), 'File ' + experiment_path + ' not found.'
    experiment = Experiment(experiment_path, RESULTS_FOLDER,
                            AUTOMATIC_DATASET_DOWNLOAD)
    return experiment.job_list


def test_job_creation():
    NUMBER_OF_DATASETS = 1
    NUMBER_OF_NORMAL_PARAMETER_FILES = 2
    NUMBER_OF_PARAMETERS_IN_PARAMETER_SWEEP_FILES = [3]
    NUMBER_OF_EXPECTED_JOBS = NUMBER_OF_DATASETS * (
        NUMBER_OF_NORMAL_PARAMETER_FILES +
        sum(NUMBER_OF_PARAMETERS_IN_PARAMETER_SWEEP_FILES))

    jobs = _create_jobs()
    nose.tools.eq_(len(jobs), NUMBER_OF_EXPECTED_JOBS)


def test_load_job_from_folder():
    jobs = _create_jobs()
    for job in jobs:
        print('Checking job:', job.job_path)
        job_from_file = Job()
        job_from_file.loadConfigFromFolder(job.job_path)
        nose.tools.eq_(job, job_from_file)
