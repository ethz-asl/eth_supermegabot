#!/usr/bin/env python

import argparse
import logging
import os
import time
import yaml

from evaluation_tools.command_runner import CommandRunnerException
import evaluation_tools.dataset_tools as dataset_tools
from evaluation_tools.evaluation import Evaluation
from evaluation_tools.job import Job
from evaluation_tools.simple_summarization import SimpleSummarization
import evaluation_tools.utils as eval_utils


class Experiment(object):
    """Main class for running an evaluation experiment."""

    def __init__(self,
                 experiment_file,
                 results_folder,
                 automatic_dataset_download,
                 enable_progress_bars=True):
        """Initializes the experiment.

        Loads and parses the yaml and creates the corresponding job objects to
        be run at a later stage.

        Input:
        - experiment_file: yaml with the experiment info.
        - results_folder: folder where the results of the evaluation are stored.
        - automatic_dataset_download: if True, datasets that cannot be found on
              disk will be automatically retrieved from a remote location as
              specified in the datasets yaml.
        """
        logging.basicConfig(level=logging.DEBUG)
        self.logger = logging.getLogger(__name__)

        self.logger.info("Checking parameters")
        if not os.path.isfile(experiment_file):
            raise Exception(
                'The experiment file "' + experiment_file + "' doesn't exist.")

        experiment_file = os.path.realpath(experiment_file)
        self.root_folder = os.path.dirname(experiment_file)
        self.experiment_file = experiment_file
        self.results_folder = results_folder
        self.evaluation_results = {}

        if not os.path.exists(self.results_folder):
            os.makedirs(self.results_folder)

        # Check Parameters
        if not experiment_file.endswith('.yaml'):
            experiment_file += '.yaml'

        # Read Evaluation File
        self.experiment_filename = os.path.basename(experiment_file).replace(
            '.yaml', '')
        self.eval_dict = yaml.safe_load(open(self.experiment_file))
        self.experiment_file = experiment_file

        # Check necessary parameters in evaluation file:
        eval_utils.assertParam(self.eval_dict, "app_package_name")
        eval_utils.assertParam(self.eval_dict, "app_executable")
        eval_utils.assertParam(self.eval_dict, "datasets")
        eval_utils.assertParam(self.eval_dict, "parameter_files")
        eval_utils.checkParam(self.eval_dict, "cam_calib_source",
                              "dataset_folder")

        # Information stored in the job but not used to run the algorithm.
        self.eval_dict['experiment_generated_time'] = time.strftime(
            "%Y%m%d_%H%M%S", time.localtime())
        self.eval_dict['experiment_filename'] = self.experiment_filename

        # Set experiment basename
        if "experiment_name" not in self.eval_dict:
            experiment_basename = (
                self.eval_dict['experiment_generated_time'] + '_' +
                self.eval_dict['experiment_filename'])
        else:
            experiment_basename = (self.eval_dict['experiment_generated_time']
                                   + '_' + self.eval_dict["experiment_name"])

        # Find sensors file:
        sensors_file = ''
        if 'sensors_file' in self.eval_dict.keys():
            sensors_file = eval_utils.findFileOrDir(
                self.root_folder, "calibrations",
                self.eval_dict['sensors_file'])
        self.eval_dict['sensors_file'] = sensors_file
        self.logger.info("Using sensors file: %s", sensors_file)

        # Find the map if there is any.
        if ('localization_map' in self.eval_dict.keys()
                and self.eval_dict['localization_map'] is not None
                and self.eval_dict['localization_map'] != ''):
            localization_map = eval_utils.findFileOrDir(
                self.root_folder, "maps", self.eval_dict["localization_map"])

            self.eval_dict['localization_map'] = localization_map
            self.logger.info("Localization map: %s", localization_map)
        else:
            self.eval_dict['localization_map'] = ''

        # Check if summarization is enabled
        self.summarize_statistics = False
        if 'summarize_statistics' in self.eval_dict:
            if 'enabled' in self.eval_dict['summarize_statistics']:
                if self.eval_dict['summarize_statistics']['enabled']:
                    self.summarize_statistics = True
                    # Summarization requires that each job runs the
                    # prepare_statistics.py script after execution.
                    if ('evaluation_scripts' not in self.eval_dict
                            or self.eval_dict['evaluation_scripts'] is None):
                        self.eval_dict['evaluation_scripts'] = []
                    self.eval_dict['evaluation_scripts'].append({
                        'name':
                        'prepare_statistics.py'
                    })

        # Create set of datasets and download them if needed.
        dataset_tools.root_folder = self.root_folder
        self.enable_progress_bars = enable_progress_bars
        dataset_tools.enable_download_progress_bar = self.enable_progress_bars
        available_datasets = dataset_tools.getDatasetList()
        downloaded_datasets, _ = dataset_tools.getDownloadedDatasets()
        for dataset in self.eval_dict['datasets']:
            # Check if dataset is available:
            dataset_path, dataset_name = os.path.split(dataset['name'])
            if dataset_path == '' or not os.path.isfile(dataset['name']):
                if dataset['name'] not in downloaded_datasets:
                    self.logger.info("Dataset '%s' is not available.",
                                     dataset['name'])
                    if dataset['name'] not in available_datasets:
                        self.logger.info(
                            "Dataset is not available on the server.")

                    # Download dataset:
                    if automatic_dataset_download:
                        download = True
                    else:
                        download = eval_utils.userYesNoQuery(
                            "Download datasets from server?")
                    if download:
                        print 'dataset[name]', dataset['name']
                        dataset['name'] = dataset_tools.downloadDataset(
                            dataset['name'])
                        print 'dataset[name]', dataset['name']
                        available_datasets = dataset_tools.getDatasetList()
                        downloaded_datasets, _ = \
                            dataset_tools.getDownloadedDatasets()
                else:
                    # Get path for dataset.
                    dataset['name'] = dataset_tools.getPathForDataset(
                        dataset_name)

                dataset_path = dataset['name']
                print 'dataset path', dataset_path
                if not os.path.isfile(dataset_path):
                    raise Exception("Unable to obtain the dataset " +
                                    dataset['name'] + ".")
                dataset['name'] = dataset_path

        # Create set of parameter files
        self.parameter_files = set()
        for filename in self.eval_dict["parameter_files"]:
            self.parameter_files.add(
                eval_utils.findFileOrDir(self.root_folder, "parameter_files",
                                         filename))

        # Create jobs for all dataset-parameter file combination.
        self.job_list = []
        if ('create_job_for_each_dataset' in self.eval_dict
                and not self.eval_dict['create_job_for_each_dataset']):
            # Create only one job for all datasets.
            self._createJobsForDatasets(experiment_basename,
                                        self.eval_dict['datasets'])
        else:
            # Default value is true.
            # Create a job for every dataset.
            for dataset in self.eval_dict['datasets']:
                self._createJobsForDatasets(experiment_basename, [dataset])

    def _createJobsForDatasets(self, experiment_basename, datasets):
        assert datasets
        job_name_from_dataset = os.path.basename(datasets[0]['name']).replace(
            '.bag', '')
        if len(datasets) > 1:
            job_name_from_dataset += '_and_others'

        for parameter_file in self.parameter_files:
            params = yaml.safe_load(open(parameter_file))

            if 'parameter_sweep' in params:
                p_name = params['parameter_sweep']["name"]
                p_min = params['parameter_sweep']["min"]
                p_max = params['parameter_sweep']["max"]
                p_step_size = params['parameter_sweep']["step_size"]

                step = 0
                max_steps = 100
                p_current = p_min
                while p_current <= p_max and step < max_steps:
                    params[p_name] = p_current
                    self.eval_dict['experiment_name'] = str(
                        experiment_basename + '/' + job_name_from_dataset +
                        '__' + os.path.basename(parameter_file)
                        .replace('.yaml', '') + '__SWEEP_' + str(step))

                    parameter_tag = str(parameter_file) + "_SWEEP_" + str(
                        p_current)
                    job = Job()
                    job.createJob(
                        datasets_dict=datasets,
                        experiment_root_folder=self.root_folder,
                        results_folder=self.results_folder,
                        experiment_dict=self.eval_dict,
                        parameter_name=parameter_tag,
                        parameter_dict=params)
                    self.job_list.append(job)
                    p_current += p_step_size
                    step += 1
            else:
                self.eval_dict['experiment_name'] = str(
                    experiment_basename + '/' + job_name_from_dataset + '__' +
                    os.path.basename(parameter_file).replace('.yaml', ''))

                job = Job()
                job.createJob(
                    datasets_dict=datasets,
                    experiment_root_folder=self.root_folder,
                    results_folder=self.results_folder,
                    experiment_dict=self.eval_dict,
                    parameter_name=str(parameter_file),
                    parameter_dict=params)
                self.job_list.append(job)

    def runAndEvaluate(self):
        """Run estimator and console commands and all evaluation scripts."""
        RESULTS_JOB_LABEL = 'job_estimator_and_console'
        for job in self.job_list:
            self.logger.info("Run job: %s/job.yaml", job.job_path)
            try:
                job.execute(
                    enable_console_progress_bars=self.enable_progress_bars)
                self.evaluation_results[job.job_name] = {RESULTS_JOB_LABEL: 0}
            except CommandRunnerException as ex:
                self.logger.error(
                    'Running the job %s failed: the estimator or console '
                    'command returned a non-zero exit code: %i.', job.job_name,
                    ex.return_value)
                self.evaluation_results[job.job_name] = {
                    RESULTS_JOB_LABEL: ex.return_value
                }
                continue

            job.writeSummary("job_summary.yaml")

            self.logger.info("Run evaluation: %s", job.job_path)
            evaluation = Evaluation(job)
            self.evaluation_results[job.job_name].update(
                evaluation.runEvaluations())

    def runSummarization(self):
        if self.summarize_statistics:
            whitelist = []
            blacklist = []
            if 'whitelisted_metrics' in self.eval_dict['summarize_statistics']:
                whitelist = self.eval_dict['summarize_statistics'][
                    'whitelisted_metrics']
            if 'blacklisted_metrics' in self.eval_dict['summarize_statistics']:
                blacklist = self.eval_dict['summarize_statistics'][
                    'blacklisted_metrics']

            files_to_summarize = []
            for job in self.job_list:
                files_to_summarize.append(
                    job.job_path + "/formatted_stats.yaml")

            s = SimpleSummarization(files_to_summarize, whitelist, blacklist)
            s.runSummarization()


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger(__name__)
    logger.info('Experiment started')

    local_data_folder_default = dataset_tools.getLocalDatasetsFolder()
    output_folder_default = './results'

    parser = argparse.ArgumentParser(description='''Experiment''')
    parser.add_argument(
        'experiment_yaml_file', help='Experiment YAML file in input_folder')
    parser.add_argument(
        '--results_output_folder',
        help='The folder where to store results',
        default=output_folder_default)
    parser.add_argument(
        '--data_folder',
        help='the path to the input data',
        default=local_data_folder_default)
    parser.add_argument(
        '--automatic_download',
        action='store_true',
        help='download dataset if it is not available locally')
    args = parser.parse_args()

    eval_file = args.experiment_yaml_file

    # Create experiment folders.
    e = Experiment(eval_file, args.results_output_folder,
                   args.automatic_download)

    # Run each job and the evaluation of each job.
    e.runAndEvaluate()

    # Run summarizations
    e.runSummarization()
