#!/usr/bin/env python

from __future__ import print_function

import argparse
import hashlib
import logging
import os
import re
import shutil
import sys
import tarfile
import time
import urllib
import yaml

import evaluation_tools.catkin_utils as catkin_utils
from evaluation_tools.utils import findFileOrDir

enable_download_progress_bar = True
root_folder = ''


def getDatasetList():
    logger = logging.getLogger(__name__)
    datasets_folder = getLocalDatasetsFolder()
    datasets_yaml = os.path.join(datasets_folder, 'datasets.yaml')

    if os.path.exists(datasets_yaml):
        dataset_list = yaml.load(open(datasets_yaml, 'r'))

        # Create dictonary of datasets and check if they are well formatted.
        datasets = dict()
        for dataset in dataset_list:
            if "name" not in dataset:
                logger.warning("Malformed dataset entry: 'name' key not found")
                continue
            if ("dir" not in dataset and "url" not in dataset
                    and "webdir" not in dataset):
                logger.warning(
                    "Malformed dataset entry: one of the tags 'dir', 'url' "
                    "or 'webdir' need to be defined.")
                continue
            datasets[dataset["name"]] = dataset
        return datasets
    raise ValueError("Could not find 'datasets.yaml' in evaluation package.")


def getLocalDatasetsFolder():
    try:
        datasets_yaml = findFileOrDir(root_folder, 'datasets', 'datasets.yaml')
        datasets_folder = os.path.dirname(datasets_yaml)
    except:  # pylint: disable=bare-except
        datasets_folder = os.path.join(
            catkin_utils.catkinFindSrc('evaluation_tools'), 'datasets')
    return datasets_folder


def getDownloadedDatasets():
    local_datasets_dir = getLocalDatasetsFolder()
    downloaded_datasets = [
        name for name in os.listdir(local_datasets_dir)
        if re.search('.*.yaml', name) is None
    ]
    return downloaded_datasets, local_datasets_dir


def listDatasets():
    all_datasets = getDatasetList()
    downloaded_datasets, data_dir = getDownloadedDatasets()

    row_format = "{:<30} {:<15} {:<15}"
    print(row_format.format("Dataset Name", "Downloaded", "Version"))
    print(row_format.format("------------", "----------", "-------"))
    for dataset_name, dataset in sorted(all_datasets.items()):
        downloaded = False
        hash_status = " "
        if dataset_name in downloaded_datasets:
            downloaded = True

            # Check if hash is ok:
            hash_status = "No version file"
            if 'sha1' in dataset:
                version_filename = os.path.join(data_dir, dataset_name,
                                                "version.sha1")
                if os.path.exists(version_filename):
                    with open(version_filename, 'r') as afile:
                        version = afile.read().replace('\n', '')
                    hash_status = "OK" if dataset[
                        'sha1'] == version else "Old Version"
            if 'version' in dataset:
                version_filename = os.path.join(data_dir, dataset_name,
                                                "version.txt")
                if os.path.exists(version_filename):
                    with open(version_filename, 'r') as afile:
                        version = afile.read().replace('\n', '')
                    hash_status = "OK" if str(
                        dataset['version']) == str(version) else "Old Version"

        print(row_format.format(dataset_name, "Yes"
                                if downloaded else "No", hash_status))


def _download_reporthook(count, block_size, total_size):
    if count == 0:
        start_time = time.time()
        return
    duration = time.time() - start_time
    progress_size = int(count * block_size)
    speed = int(progress_size / (1024 * duration))
    percent = int(count * block_size * 100 / total_size)
    if enable_download_progress_bar:
        sys.stdout.write("\r...%d%%, %d MB, %d KB/s, %d seconds passed" %
                         (percent, progress_size / (1024 * 1024), speed,
                          duration))
    sys.stdout.flush()


def downloadFileFromServer(file_url, local_filename):
    logger = logging.getLogger(__name__)
    logger.info('Downloading file from server from %s', file_url)
    logger.info('to %s', local_filename)
    os.makedirs(os.path.dirname(local_filename))
    urllib.urlretrieve(file_url, local_filename, _download_reporthook)
    logger.info('\ndone.')
    assert os.path.exists(local_filename)  #TODO(cfo): checksum


def validFileOnServer(filename):
    logger = logging.getLogger(__name__)
    try:
        urllib.urlopen(filename)
    except ValueError:
        logger.warning('Unknown URL type: %s', filename)
        return False
    return True


def getFileHash(filename):
    logger = logging.getLogger(__name__)
    logger.debug("Creating hash of file: %s", filename)
    if not os.path.exists(filename):
        raise ValueError("File does not exist: " + filename)
    file_hash = hashlib.sha1(open(filename, 'rb').read()).hexdigest()
    return file_hash


def getPathForDataset(dataset_name):
    downloaded_datasets, data_dir = getDownloadedDatasets()
    assert dataset_name in downloaded_datasets
    datasets = getDatasetList()
    if dataset_name not in datasets:
        raise ValueError(
            'Dataset ' + dataset_name + ' is not listed in datasets.yaml')
    dataset = datasets[dataset_name]
    if 'file_name' in dataset:
        return os.path.join(data_dir, dataset_name, dataset['file_name'])
    else:
        return os.path.join(data_dir, dataset_name)


def downloadDataset(dataset_name):
    logger = logging.getLogger(__name__)

    # Check that dataset_name is valid:
    datasets = getDatasetList()
    if dataset_name not in datasets:
        logger.warning("Dataset: %s is not valid. Check datasets.yaml"
                       " for available datasets.", dataset_name)
        return
    dataset = datasets[dataset_name]

    # Create target directory:
    local_data_dir = getLocalDatasetsFolder()
    dataset_dir = os.path.join(local_data_dir, dataset['name'])
    #if os.path.exists(dataset_dir):
    #    logger.warning("Dataset folder already exists: "+dataset_dir)
    #    if not replace:
    #        return
    #    else:
    #        logger.info("Delete old dataset folder: "+dataset_dir)
    #        os.system("rm -rf " + dataset_dir)

    #logger.info("Creating dataset folder: "+dataset_dir)
    #os.makedirs(dataset_dir)

    # -------------------------------------------------------------------------
    # Download zip from url
    if 'url' in dataset:
        # Download dataset files from server as specified in info YAML.
        url = dataset['url']
        filename = url.split('/')[-1]
        local_filename = os.path.join(dataset_dir, filename)
        if validFileOnServer(url):
            downloadFileFromServer(url, local_filename)
        else:
            logger.warning('File %s does not exist on server.', url)

        # compute a hash of the file
        downloaded_file_hash = getFileHash(local_filename)
        version_file = open(os.path.join(dataset_dir, 'version.sha1'), 'w')
        version_file.write('{0}'.format(downloaded_file_hash))
        version_file.close()

        # check if hash is ok and write it
        if 'sha1' in dataset:
            if downloaded_file_hash == dataset['sha1']:
                logger.info(
                    "Successfully verified hash-key of downloaded file")
            else:
                raise ValueError(
                    "Hash of downloaded dataset is not valid: " + filename)

        # unpack tar.gz
        if local_filename.split(".")[-2] == 'tar' and local_filename.split(
                ".")[-1] == 'gz':
            logger.info("Unpacking .tar.gz file...")
            tfile = tarfile.open(local_filename, 'r:gz')
            tfile.extractall(dataset_dir)
            os.remove(local_filename)
            logger.info("...done.")

    elif 'dir' in dataset:
        dataset_path = os.path.join(dataset['dir'], dataset['name'])
        if (not os.path.isfile(dataset_path)
                and not os.path.isdir(dataset_path) and 'file_name' in dataset):
            dataset_path = dataset['dir']
        local_dataset_path = os.path.join(local_data_dir, dataset['name'])
        if 'copy_file' in dataset and not dataset['copy_file']:
            print('Creating a symlink to the dataset...')
            os.symlink(dataset_path, local_dataset_path)
        else:
            print('Starting copying of dataset to local folder...')
            if os.path.isfile(dataset_path):
                shutil.copyfile(dataset_path, local_dataset_path)
            elif os.path.isdir(dataset_path):
                shutil.copytree(dataset_path, local_dataset_path)
            else:
                raise Exception(
                    'Can\'t copy dataset under "%s" because no file or folder '
                    'exists in the given path.' % dataset_path)
        print('Done.')
    else:
        raise Exception("Unclear how to download the dataset.")

    return getPathForDataset(dataset_name)


if __name__ == '__main__':
    usage = """
        List and download available Zurich-Eye datasets.
    """
    parser = argparse.ArgumentParser(description=usage)
    parser.add_argument(
        '--list',
        dest='list_datasets',
        action='store_true',
        help='List all available datasets"')

    parser.add_argument(
        '--fetch', dest='fetch_dataset', default='', help='Download dataset')

    parser.add_argument(
        '--hash', dest='hash', default='', help='Create hash of file.')

    # Parse command-line arguments
    options = parser.parse_args()

    # Setup logging
    logging.basicConfig(level=logging.DEBUG)
    _ = logging.getLogger(__name__)

    if options.list_datasets:
        listDatasets()

    if options.fetch_dataset:
        downloadDataset(options.fetch_dataset)

    if options.hash:
        hash_of_file = getFileHash(options.hash)
        print(hash_of_file)
