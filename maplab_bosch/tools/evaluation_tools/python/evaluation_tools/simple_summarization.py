#!/usr/bin/env python

import argparse
from collections import defaultdict, namedtuple
import logging
from math import sqrt
import os
import re
import yaml

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np


def atoi(text):
    return int(text) if text.isdigit() else text


def alphanum_key(text):
    """Key used to sort strings in human order.

    Source: http://stackoverflow.com/questions/5967500/how-to-correctly-sort-a-string-with-a-number-inside  # pylint:disable=line-too-long
    """
    return [atoi(c) for c in re.split(r'(\d+)', text)]


class Metric(object):
    """Data Structure that stores some basic statistical properties."""

    def __init__(self):
        self.count = 0
        self.mean = 0
        self.stddev = 0
        self.var = 0
        self.min = float('nan')
        self.max = float('nan')

    def addSampleFromDict(self, sample):
        """Incorporate data from a sample that comes in dictionary form."""
        if not isinstance(sample, dict):
            raise TypeError("Sample is not a dictionary")

        if not {'samples', 'mean', 'stddev', 'min', 'max'}.issubset(
                sample.keys()):
            raise ValueError("Malformed sample")

        m = Metric()
        m.count = sample["samples"]
        m.mean = sample["mean"]
        m.stddev = sample["stddev"]
        m.var = m.stddev * m.stddev
        m.min = sample["min"]
        m.max = sample["max"]

        self.mergeMetric(m)

    def mergeMetric(self, other):
        """Incorporate data from another metric."""
        if self.count == 0:
            self.mean = other.mean
            self.var = other.var
            self.stddev = other.stddev
            self.min = other.min
            self.max = other.max

        elif other.count != 0:
            self.mean = (self.mean * self.count + other.mean * other.count) \
                / (self.count + other.count)

            self.var = (((self.count - 1) * self.var +
                         (other.count - 1) * other.var +
                         (self.count * other.count) /
                         (self.count + other.count) *
                         (self.mean * self.mean + other.mean * other.mean -
                          2 * self.mean * other.mean)) /
                        (self.count + other.count - 1))
            self.stddev = sqrt(self.var)

            self.min = min(self.min, other.min)
            self.max = max(self.max, other.max)

        self.count += other.count


class Plotter(object):
    """Plots the results from the summarization."""

    def __init__(self):
        self.colors = [
            '#FCA17D', '#DA627D', '#9A348E', '#FAF3DD', '#69c6bf', '#97d7ce',
            '#6DAEDB', '#2892D7', '#3E78B2', '#1D70A2', '#1B998B', '#4FB286',
            '#b9ee98', '#d5f396', '#f0f79a', '#ffd993', '#ffc966'
        ]

        self.Data = namedtuple(
            "Data", "label means stddevs mins maxes parameter_files")

        self.SweepData = namedtuple("SweepData",
                                    "label means stddevs mins maxes indices")

    def plot(self, metrics):
        """Main hook to plot metrics.

        Creates one Data namedtuple that contains the data from all runs,
        except those that come from parameter sweeps, and one SweepData
        namedtuple for each parameter sweep. Then calls the plotting functions
        for each Data and SweepData.
        """
        for metric, parameter_files in metrics.items():
            # Find parameter sweeps captured in the metric.
            parameter_sweep_files = set()
            for parameter_file in parameter_files:
                if any(strip == 'SWEEP'
                       for strip in parameter_file.split('_')):
                    param_filename = parameter_file.split('_SWEEP_')[0]
                    parameter_sweep_files.add(param_filename)

            # Create Data and SweepData instances.
            data_without_sweeps = self.Data(
                label=metric,
                means=[],
                stddevs=[],
                mins=[],
                maxes=[],
                parameter_files=[])
            sweep_data_dict = {}
            for parameter_sweep_file in parameter_sweep_files:
                sweep_data_dict[parameter_sweep_file] = self.SweepData(
                    label=metric,
                    indices=[],
                    means=[],
                    stddevs=[],
                    mins=[],
                    maxes=[])

            # Fill in Data and SweepData
            sorted_parameter_files = parameter_files.keys()
            sorted_parameter_files.sort(key=alphanum_key)
            for parameter_file in sorted_parameter_files:
                if any(strip == 'SWEEP'
                       for strip in parameter_file.split('_')):
                    parameter_filename = parameter_file.split('_SWEEP_')[0]
                    sweep_data_dict[parameter_filename].indices.append(
                        parameter_file.split('_SWEEP_')[-1])
                    sweep_data_dict[parameter_filename].means.append(
                        parameter_files[parameter_file]["mean"])
                    sweep_data_dict[parameter_filename].stddevs.append(
                        parameter_files[parameter_file]["stddev"])
                    sweep_data_dict[parameter_filename].mins.append(
                        parameter_files[parameter_file]["min"])
                    sweep_data_dict[parameter_filename].maxes.append(
                        parameter_files[parameter_file]["max"])

                else:
                    data_without_sweeps.means.append(
                        parameter_files[parameter_file]["mean"])
                    data_without_sweeps.stddevs.append(
                        parameter_files[parameter_file]["stddev"])
                    data_without_sweeps.mins.append(
                        parameter_files[parameter_file]["min"])
                    data_without_sweeps.maxes.append(
                        parameter_files[parameter_file]["max"])
                    data_without_sweeps.parameter_files.append(parameter_file)

            self.plotDataWithoutSweeps(data_without_sweeps)
            self.plotSweepsData(sweep_data_dict)

        plt.show()

    def plotDataWithoutSweeps(self, data):
        """Creates and shows one bar plot with the input data."""
        if len(data.means) is 0:
            return

        # Prepare the plot.
        patches = []
        plt.figure()
        plt.title('{}'.format(data.label), fontsize=14)
        plt.xlabel('Parameter file')
        plt.ylabel('Value')

        N = len(data.means)
        ind = np.arange(N)

        # Plot the data, bars and text.
        barlist = plt.bar(ind, data.means, 1)
        plt.errorbar(
            ind + 0.5, data.means, data.stddevs, fmt='o', ecolor='black', lw=3)
        plt.errorbar(
            ind + 0.5,
            data.means, [data.mins, data.maxes],
            fmt='.',
            ecolor='black',
            lw=1)
        for x, y in zip(ind, data.means):
            plt.text(x + 0.55, y * 1.05, "{0:.2f}".format(y))

        # Format the plot.
        for i in range(N):
            current_color = self.colors[i % len(self.colors)]
            barlist[i].set_color(current_color)
            patches.append(
                mpatches.Patch(
                    color=current_color, label=data.parameter_files[i]))

        max_y = plt.axis()[3]
        plt.xlim(-2, N + 2)
        plt.ylim(-0.5 * max_y, 1.5 * max_y)
        plt.legend(handles=patches)
        plt.grid()

    def plotSweepsData(self, data):
        """Creates one x-y plot for each SweepData inside data."""
        for param_file, sweep_data in data.items():
            # Prepare the plot.
            indices = np.array(sweep_data.indices).astype(np.float64)
            means = np.array(sweep_data.means).astype(np.float64)
            maxes = np.array(sweep_data.maxes).astype(np.float64)
            mins = np.array(sweep_data.mins).astype(np.float64)
            stddevs = np.array(sweep_data.stddevs).astype(np.float64)
            stddevs_sup = means + stddevs
            stddevs_inf = means - stddevs

            plt.figure()
            plt.title(
                '{} - param sweep from {}'.format(sweep_data.label,
                                                  param_file),
                fontsize=14)
            plt.xlabel('Value of parameter')
            plt.ylabel('Result')

            # Plot the data.
            ax = plt.gca()
            ax.plot(indices, means, linestyle='-', lw=3, color="black")
            ax.plot(indices, maxes, linestyle=':', lw=2, color="grey")
            ax.plot(indices, mins, linestyle=':', lw=2, color="grey")
            ax.plot(
                indices, stddevs_sup, linestyle='--', lw=2, color="#9a0bad")
            ax.plot(
                indices, stddevs_inf, linestyle='--', lw=2, color="#9a0bad")

            ax.fill_between(
                indices, stddevs_sup, stddevs_inf, color="#9a0bad", alpha=0.2)
            ax.fill_between(indices, mins, maxes, color="gray", alpha=0.1)

            # Format the plot.
            curr_axis = plt.axis()
            x_range = curr_axis[1] - curr_axis[0]
            y_range = curr_axis[3] - curr_axis[2]
            plt.xlim(curr_axis[0] - 0.1 * x_range,
                     curr_axis[1] + 0.1 * x_range)
            plt.ylim(curr_axis[2] - 0.1 * y_range,
                     curr_axis[3] + 0.1 * y_range)
            plt.grid()


class SimpleSummarization(object):
    """Performs and plots a summarization of the jobs run in the experiment.

    Reads information from the 'formated_stats.yaml' files available in each job
    result folder, and summarizes and plots the data.
    """

    def __init__(self, files_to_summarize, whitelist=None, blacklist=None):
        logging.basicConfig(level=logging.DEBUG)
        self.logger = logging.getLogger(__name__)
        self.files_to_summarize = files_to_summarize
        if whitelist is not None:
            self.whitelist = whitelist
        else:
            self.whitelist = []
        if blacklist is not None:
            self.blacklist = blacklist
        else:
            self.blacklist = []
        self.plotter = Plotter()

        # Load data
        self.datasets = set()
        self.parameter_files = set()
        self.metrics = defaultdict((dict))

        self.metrics = defaultdict(lambda: defaultdict(Metric))
        for file_to_summarize in files_to_summarize:
            if not os.path.isfile(file_to_summarize):
                raise ValueError(
                    "Output file does not exist: {}".format(file_to_summarize))
            statistics = yaml.safe_load(open(file_to_summarize))
            if not {'dataset', 'metrics', 'parameter_file'}.issubset(
                    statistics.keys()):
                raise ValueError(
                    'Malformed statistics file: {}'.format(file_to_summarize))

            dataset = statistics["dataset"]
            parameter_file = statistics["parameter_file"]
            metrics = statistics["metrics"]

            # Remove unwanted metrics
            if self.whitelist:
                metrics = {
                    key: metrics[key]
                    for key in metrics if key in self.whitelist
                }

            else:
                metrics = {
                    key: metrics[key]
                    for key in metrics if key not in self.blacklist
                }

            self.datasets.add(dataset)
            self.parameter_files.add(parameter_file)

            for metric, values in metrics.items():
                self.metrics[(
                    dataset, parameter_file)][metric].addSampleFromDict(values)

        self.createMetricsIndices()
        self.logger.info(
            "Extracted data from %i runs, across %i datasets and %i parameter "
            "sets.", len(self.files_to_summarize), len(self.datasets),
            len(self.parameter_files))

    def runSummarization(self):
        metrics = self.summarizeMetricsFromDatasets()
        self.plotter.plot(metrics)

    def createMetricsIndices(self):
        self.metrics_by_dataset = defaultdict(dict)
        for dataset in self.datasets:
            for parameter_file in self.parameter_files:
                self.metrics_by_dataset[dataset][
                    parameter_file] = self.metrics[(dataset, parameter_file)]

        self.metrics_by_parameter_file = defaultdict(dict)
        for parameter_file in self.parameter_files:
            for dataset in self.datasets:
                self.metrics_by_parameter_file[parameter_file][
                    dataset] = self.metrics[(dataset, parameter_file)]

    def summarizeMetricsFromDatasets(self, datasets='all'):
        if datasets == 'all':
            datasets = self.datasets

        merged_datasets = self.mergeDatasets(datasets)
        metrics_dict = defaultdict(lambda: defaultdict(dict))
        for parameter_file, keys in merged_datasets.items():
            for key, value in keys.items():
                metrics_dict[key][parameter_file]["mean"] = value.mean
                metrics_dict[key][parameter_file]["stddev"] = value.stddev
                metrics_dict[key][parameter_file]["max"] = value.max
                metrics_dict[key][parameter_file]["min"] = value.min
                metrics_dict[key][parameter_file]["count"] = value.count

        return metrics_dict

    def mergeDatasets(self, datasets='all'):
        if not set(datasets).issubset(self.datasets):
            raise Exception("One or more datasets not found.")

        result_dict = defaultdict(dict)
        for dataset in datasets:
            for parameter_file, metrics in self.metrics_by_dataset[
                    dataset].items():
                for key, metric in metrics.items():
                    if key not in result_dict[parameter_file].keys():
                        result_dict[parameter_file][key] = Metric()
                    result_dict[parameter_file][key].mergeMetric(metric)

        return result_dict


if __name__ == '__main__':

    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger(__name__)
    logger.info('Summarization started')

    parser = argparse.ArgumentParser(description="""Sumarization job""")
    parser.add_argument(
        '--results_folder',
        help='folder that contains all job folders',
        default="")
    args = parser.parse_args()

    if not os.path.isdir(args.results_folder):
        logger.error("Failed to open results folder")

    result_files = [
        args.results_folder + '/' + run_folder + '/formatted_stats.yaml'
        for run_folder in os.listdir(args.results_folder)
    ]

    # TODO Read whitelist / blacklist from job and pass it here.
    ev = SimpleSummarization(result_files)
    ev.runSummarization()
