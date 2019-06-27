#!/usr/bin/env python

import argparse
import copy
import logging
import os
import re
import yaml

import evaluation_tools.catkin_utils as catkin_utils
from evaluation_tools.command_runner import runCommand


class Job(object):
    """Contains the information to run the experiment (estimator and console).
    """

    def __init__(self):
        logging.basicConfig(level=logging.DEBUG)
        self.logger = logging.getLogger(__name__)
        self.params_dict = []
        self.additional_placeholders = []
        self._dataset_log_dir_prefix = 'estimator_output_'

        self.dataset_additional_parameters = None
        self.dataset_log_dirs = None
        self.dataset_names = None
        self.dataset_paths = None
        self.exec_app = None
        self.exec_folder = None
        self.exec_name = None
        self.exec_path = None
        self.experiment_root_folder = None
        self.info = None
        self.job_name = None
        self.job_path = None
        self.localization_map = None
        self.output_map_folders = None
        self.sensors_file = None

    def __eq__(self, other):
        if isinstance(self, other.__class__):
            return vars(self) == vars(other)
        return False

    def createJob(self,
                  datasets_dict,
                  experiment_root_folder,
                  results_folder,
                  experiment_dict,
                  parameter_name,
                  parameter_dict,
                  summarize_statistics=False):
        """Initializes the job.

        Input:
        - datasets_dict: list of datasets dict containing the dataset name and
              additional parameters.
        - experiment_root_folder: root folder of the experiment. This is usally
              the folder where the experiment yaml is located in or one folder
              above.
        - results_folder: location to store the results in.
        - experiment_dict: dictionary containing the loaded information from the
              experiment yaml.
        - parameter_name: name of the parameter set of this job.
        - parameter_dict: dictionary containing all parameters for this job
              (loaded from the corresponding parameter yaml).
        - summarize_statistics: (only works with SWE).

        Return value: nothing.

        This function will do various things to initialize a job:
        1) Create a job folder in <results_folder>/<experiment_name>
           (experiment_name is read from the experiment_dict).
        2) Read parameters and replace all placedholders.
        3) Create control file for the maplab batch runner to later run console
           commands.
        4) Export the job control information to yaml for later reference.

        This function needs to be called once per job so that all necessary
        files are created on disks.
        """
        self.job_name = experiment_dict['experiment_name']
        self.experiment_root_folder = experiment_root_folder
        self.job_path = os.path.join(results_folder, self.job_name)
        self.logger.info("==> Creating job:in folder '%s'", self.job_path)
        if not os.path.exists(self.job_path):
            os.makedirs(self.job_path)
        else:
            self.logger.info("Job folder already exists '%s'", self.job_path)

        self._parseDatasetsDict(datasets_dict)

        self.sensors_file = experiment_dict['sensors_file']
        self.localization_map = experiment_dict['localization_map']

        self._obtainOutputMapKeyAndFolderForDataset()
        self._addAdditionalPlaceholders()

        for dataset_log_dir in self.dataset_log_dirs:
            if not os.path.exists(dataset_log_dir):
                os.makedirs(dataset_log_dir)

        self._createParamsDict(parameter_dict, summarize_statistics)

        # Write console batch runner file.
        if ('console_commands' in experiment_dict
                and experiment_dict['console_commands']):
            console_batch_runner_settings = {
                "vi_map_folder_paths": [self.output_map_folders[0]],
                "commands": []
            }
            for command in experiment_dict['console_commands']:
                if isinstance(command, str):
                    console_batch_runner_settings['commands'].append(
                        self.replacePlaceholdersInString(command))

            console_batch_runner_filename = os.path.join(
                self.job_path, "console_commands.yaml")
            self.logger.info("Write %s", console_batch_runner_filename)
            with open(console_batch_runner_filename, "w") as out_file_stream:
                yaml.safe_dump(
                    console_batch_runner_settings,
                    stream=out_file_stream,
                    default_flow_style=False,
                    width=10000)  # Prevent random line breaks in long strings.

        # Write options to file.
        self.info = copy.deepcopy(experiment_dict)
        self.info['experiment_root_folder'] = self.experiment_root_folder
        self.info['datasets'] = [{
            'name':
            name,
            'additional_parameters':
            additional_parameters,
            'parameters':
            parameters
        } for name, additional_parameters, parameters in zip(
            self.dataset_paths, self.dataset_additional_parameters,
            self.params_dict)]
        self.info['parameter_file'] = parameter_name
        del self.info['parameter_files']
        if 'console_commands' in self.info:
            del self.info['console_commands']

        job_filename = os.path.join(self.job_path, "job.yaml")
        self.logger.info("Write %s", job_filename)
        with open(job_filename, "w") as out_file_stream:
            yaml.safe_dump(
                self.info, stream=out_file_stream, default_flow_style=False)

        self.exec_app = self.info["app_package_name"]
        self.exec_name = self.info["app_executable"]
        self.exec_folder = catkin_utils.catkinFindLib(self.exec_app)
        self.exec_path = os.path.join(self.exec_folder, self.exec_name)

    def _parseDatasetsDict(self, datasets_dict):
        """Creates a list of dataset names and additional parameters from the
        dict.
        """
        self.dataset_paths = [
            dataset_dict['name'] for dataset_dict in datasets_dict
        ]
        self.dataset_additional_parameters = []
        for dataset_dict in datasets_dict:
            if 'additional_parameters' in dataset_dict:
                self.dataset_additional_parameters.append(
                    dataset_dict['additional_parameters'])
            else:
                self.dataset_additional_parameters.append({})

    def _obtainOutputMapKeyAndFolderForDataset(self):
        """Generates suggested map keys and folders from the dataset names."""
        self.dataset_names = [
            os.path.basename(dataset_name).replace('.bag', '')
            for dataset_name in self.dataset_paths
        ]
        self.dataset_log_dirs = [
            os.path.join(self.job_path,
                         self._dataset_log_dir_prefix + dataset_name)
            for dataset_name in self.dataset_names
        ]
        self.output_map_folders = [
            os.path.join(dataset_log_dir, dataset_name) for dataset_log_dir,
            dataset_name in zip(self.dataset_log_dirs, self.dataset_names)
        ]

    def _addAdditionalPlaceholders(self):
        """Creates placeholders for the additional parameters."""
        for idx, dataset_additional_parameters in \
            enumerate(self.dataset_additional_parameters):
            for key, value in dataset_additional_parameters.iteritems():
                self.additional_placeholders.append({})
                if isinstance(value, str):
                    dataset_additional_parameters[key] = (
                        self.replacePlaceholdersInString(
                            value, dataset_index=idx))
                self.additional_placeholders[idx]['<' + key + '>'] = str(
                    dataset_additional_parameters[key])

    def _createParamsDict(self, parameter_dict, summarize_statistics=False):
        """Create a placeholder-free params dict."""
        for idx in range(len(self.dataset_paths)):
            self.params_dict.append({})
            for key, value in parameter_dict.items():
                if isinstance(value, str):
                    self.params_dict[idx][
                        key] = self.replacePlaceholdersInString(
                            value, dataset_index=idx)
                else:
                    self.params_dict[idx][key] = value

            # We do not want to pass parameter_sweep as an argument to the
            # executable.
            if 'parameter_sweep' in self.params_dict[idx]:
                del self.params_dict[idx]['parameter_sweep']
            if summarize_statistics:
                # TODO(eggerk): generalize or remove.
                self.params_dict[idx]['swe_write_statistics_to_file'] = 1

    def replacePlaceholdersInString(self, string, dataset_index=0):
        """Replaces placeholders in a string with the actual value for the job.

    This is used to adapt the parameters and console commands to the current
    running job. Replaced placeholders include the current job directory, the
    bag file and others.

    Inputs:
    - string: string in which placeholders should be replaced.
    - dataset_index: (default: 0) index of the entry that should be used for
          placeholders that don't contain an index (e.g. <BAG_FILENAME> instead
          of <BAG_FILENAME_#>).

    Return value: input string with all placeholders replaced.

    Throws an exception if there are still "<" or ">" characters left in the
    output string.

    The following placeholders exist (# refers to the dataset index):
    - <BAG_FILENAME>: path to the dataset bag file.
    - <BAG_FOLDER>: folder of the bag file.
    - <SENSORS_YAML>: path to the sensor calibration file as specified in the
          experiment yaml.
    - <LOCALIZATION_MAP>: path to the localization map as specified in the
          experiment yaml.
    - <OUTPUT_MAP_FOLDER>, <OUTPUT_MAP_FOLDER_#>: suggested folder to save the
          map in if it's needed for the console execution or the evaluation
          scripts.
    - <DATASET_NAME>, <DATASET_NAME_#>, <OUTPUT_MAP_KEY>, <OUTPUT_MAP_KEY_#>:
          dataset name, i.e. the basename of the bag path without the '.bag'
          extension. This is the suggested map key to use when the map is loaded
          into the console.
    - <JOB_DIR>: path of the job.
    - <DATASET_LOG_DIR>: suggested output folder for dataset-specific data,
          i.e. primarily output data from the estimator.  This will be equal to
          <JOB_DIR>/estimator_output_<DATASET_NAME>
    """
        string = string.replace('<BAG_FILENAME>',
                                self.dataset_paths[dataset_index])
        string = string.replace('<BAG_FOLDER>',
                                os.path.dirname(
                                    self.dataset_paths[dataset_index]))
        for i in range(0, len(self.dataset_names)):
            string = string.replace('<BAG_FILENAME_' + str(i) + '>',
                                    self.dataset_names[i])
            string = string.replace('<BAG_FOLDER_' + str(i) + '>',
                                    os.path.dirname(self.dataset_names[i]))

        string = string.replace('<SENSORS_YAML>', self.sensors_file)
        string = string.replace('<LOCALIZATION_MAP>', self.localization_map)

        string = string.replace('<OUTPUT_MAP_FOLDER>',
                                self.output_map_folders[dataset_index])
        for i in range(0, len(self.output_map_folders)):
            string = string.replace('<OUTPUT_MAP_FOLDER_' + str(i) + '>',
                                    self.output_map_folders[i])

        string = string.replace('<DATASET_NAME>',
                                self.dataset_names[dataset_index])
        for i in range(0, len(self.dataset_names)):
            string = string.replace('<DATASET_NAME_' + str(i) + '>',
                                    self.dataset_names[i])
        string = string.replace('<OUTPUT_MAP_KEY>',
                                self.dataset_names[dataset_index])
        for i in range(0, len(self.dataset_names)):
            string = string.replace('<OUTPUT_MAP_KEY_' + str(i) + '>',
                                    self.dataset_names[i])

        string = string.replace('<JOB_DIR>', self.job_path)
        string = string.replace('<DATASET_LOG_DIR>',
                                self.dataset_log_dirs[dataset_index])

        if len(self.additional_placeholders) > dataset_index:
            for original, replacement in self.additional_placeholders[
                    dataset_index].iteritems():
                # No index is supported for additional placeholders. These come
                # from the additional dataset parameters.
                string = string.replace(original, replacement)

        # Check that no substrings in the form of <...> are left.
        regex_result = re.search('<.*>', string)
        if regex_result:
            raise Exception(
                'Replacing of placeholders did not complete: invalid '
                'placeholder "' + regex_result.group() +
                '" found. Resulting string: ' + string)
        return string

    def loadConfigFromFolder(self, job_path):
        """Loads the job configuration from disk.

    Reads the file named job.yaml inside job_path to initialize the job.
    """
        self.job_path = job_path
        self.logger = logging.getLogger(__name__)

        job_filename = os.path.join(job_path, 'job.yaml')
        self.logger.info("Loading job config from file: %s", job_filename)
        if not os.path.isfile(job_filename):
            raise ValueError("Job info file does not exist: " + job_filename)
        self.info = yaml.safe_load(open(job_filename))
        self.job_name = self.info['experiment_name']
        self._parseDatasetsDict(self.info['datasets'])
        self.sensors_file = self.info['sensors_file']
        self.localization_map = self.info['localization_map']
        self._obtainOutputMapKeyAndFolderForDataset()
        self._addAdditionalPlaceholders()
        self.exec_app = self.info["app_package_name"]
        self.exec_name = self.info["app_executable"]
        self.exec_folder = catkin_utils.catkinFindLib(self.exec_app)
        self.exec_path = os.path.join(self.exec_folder, self.exec_name)
        self.experiment_root_folder = self.info['experiment_root_folder']
        self.params_dict = [
            dataset_dict['parameters']
            for dataset_dict in self.info['datasets']
        ]

    def execute(self,
                skip_estimator=False,
                skip_console=False,
                enable_console_progress_bars=True):
        """Runs the estimator and maplab console as defined in this job.

    Input:
    - skip_estimator: skips the estimator step of the job. Useful for debugging.
    - skip_console: skips the console step of the job. Useful for debugging.
    - enable_console_progress_bars: if True, progress bars in the maplab
          console will be disabled. This is useful when the output is forwarded
          into a log file (e.g. on a Jenkins job).
    """
        if not skip_estimator:
            # Run estimator.
            for params in self.params_dict:
                runCommand(self.exec_path, params_dict=params)
        else:
            self.logger.info("Step estimator of job was skipped.")

        if not skip_console:
            # Run console commands.
            batch_runner_settings_file = os.path.join(self.job_path,
                                                      "console_commands.yaml")
            if os.path.isfile(batch_runner_settings_file):
                console_executable_path = catkin_utils.catkinFindLib(
                    "maplab_console")
                runCommand(
                    os.path.join(console_executable_path, "batch_runner"),
                    params_dict={
                        "log_dir": self.job_path,
                        "batch_control_file": batch_runner_settings_file,
                        "show_progress_bar": enable_console_progress_bars
                    })
            else:
                self.logger.info("No console commands to be run.")
        else:
            self.logger.info("Step console of job was skipped.")

    def writeSummary(self, filename):
        summary_dict = {}
        summary_dict["executable"] = {}
        summary_dict["executable"]["name"] = self.exec_name
        summary_dict["executable"]["path"] = self.exec_path
        summary_dict["executable"]["rev"] = \
            catkin_utils.getSrcRevision(self.exec_app)

        if "cam_id" in self.info:
            summary_dict["calib"] = {}
            summary_dict["calib"]["file"] = self.info["cam_calib_file"]
            summary_dict["calib"]["id"] = self.info["cam_id"]
            summary_dict["calib"]["rev"] = \
                catkin_utils.getCalibRevision(self.info["cam_id"])

        out_file_path = os.path.join(self.job_path, filename)
        out_file_stream = open(out_file_path, "w")
        yaml.safe_dump(
            summary_dict, stream=out_file_stream, default_flow_style=False)


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger(__name__)

    parser = argparse.ArgumentParser(description="""Process single job""")
    parser.add_argument('job_dir', help='directory of the job', default='')
    parser.add_argument(
        '--skip_estimator',
        help='Skip the estimator step when running the job.',
        required=False,
        default=False,
        action="store_true")
    parser.add_argument(
        '--skip_console',
        help='Skip the console step when running the job.',
        required=False,
        default=False,
        action="store_true")
    args = parser.parse_args()

    if args.job_dir:
        j = Job()
        j.loadConfigFromFolder(args.job_dir)
        j.execute(
            skip_estimator=args.skip_estimator, skip_console=args.skip_console)
