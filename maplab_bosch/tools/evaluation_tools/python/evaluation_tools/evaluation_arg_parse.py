import argparse
import yaml


class EvaluationArgParse(object):
    def __init__(self, description=''):
        self.parser = argparse.ArgumentParser(description)
        self.parser.add_argument('--job_dir')
        self.parser.add_argument('--localization_map', default='')
        self.parser.add_argument('--parameter_file', default='')
        self.parser.add_argument('--dataset_paths', default='', nargs='+')
        self.parser.add_argument('--dataset_log_dirs', default='', nargs='+')
        self.parser.add_argument(
            '--additional_dataset_parameters', type=yaml.load)
