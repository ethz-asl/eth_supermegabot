import os
import yaml
import argparse
import subprocess
from distutils.dir_util import copy_tree

# for debug
import ipdb

PRFIX = '[MAIN]: '

def makedirs(dirname):
    if not os.path.exists(dirname):
        os.makedirs(dirname)

# Bash cmds
def bash_command(cmd):
    try:
        process = subprocess.check_output(cmd, shell=True, executable='/bin/bash', stderr=subprocess.STDOUT)
        #process.wait()
        return process.rstrip().decode('utf-8')
    except subprocess.CalledProcessError:
        return False

# Argument parser
def parse_args():
    # Create parser instance
    parser = argparse.ArgumentParser(description="Update all required pkgs for summerschool private repo")
    # Define arguments
    parser.add_argument('--packages_list', type=str, default='packages_list')
    parser.add_argument('--source_ws', type=str, default=None, required=True)
    parser.add_argument('--target_dir', type=str, default=None, required=True)
    return parser.parse_args()

def main(args):
    # Parameters
    source_ws = args.source_ws
    assert os.path.isdir(source_ws), "The source_ws was not found: {}".format(source_ws)
    packages_list_file = args.packages_list
    assert os.path.isfile(packages_list_file), "The packages_list was not found."
    packages_list_file = os.path.realpath(packages_list_file)
    source_ws = os.path.realpath(source_ws)
    target_dir = args.target_dir
    makedirs(target_dir)
    target_dir = os.path.realpath(target_dir)
    # Try to load setup.bash from source_ws
    source_setup = os.path.join(source_ws, 'devel')
    source_setup = os.path.join(source_setup, 'setup.bash')
    source_cmd = "source " + source_setup + ";"
    #ipdb.set_trace()
    assert bash_command(source_cmd) is not False, "{} not found!".format(source_setup)

    print(PRFIX + 'Apps for updating all required pkgs in the summerschool repo')
    print(PRFIX + 'Package list file: ' + packages_list_file)
    print(PRFIX + 'Source workspace (copy from): ' + source_ws)
    print(PRFIX + 'Target directory (copy to): ' + target_dir)

    # Load packages list from file
    with open(packages_list_file, 'r') as stream:
        try:
            packages_list = yaml.safe_load(stream)         
            print(PRFIX + 'Successfully load yaml file.')
        except yaml.YAMLError as exc:
            print(exc)
    
    normal_packages = packages_list['normal_packages']
    meta_packages = packages_list['meta_packages']

    print('\n' + PRFIX + 'Numbers of normal packages: {}'.format(len(normal_packages)))
    print(PRFIX + 'Numbers of meta packages: {}'.format(len(meta_packages)))
    print('\n' + PRFIX + 'List of normal packages:')
    print(*normal_packages, sep='\n')
    print('\n' + PRFIX + 'List of meta packages:')
    for meta_name, sub_pkgs in meta_packages.items():
        print(PRFIX + 'Name of meta package: {}'.format(meta_name))
        print(*sub_pkgs, sep='\n')
        print()

    # User check
    answer = input("[WARNING]: This app will first delete all pkgs then fork again, continue?(Yes/No): ")
    if answer != 'Yes':
        print(PRFIX + 'Answer is {}, Exit!'.format(answer))
        return

    # Start remove/forked selected packages
    print(PRFIX + 'Start updating ...')
    failure_list = []
    rospack_cmd = source_cmd + ' rospack find '
    # For normal packages
    for pkg_name in normal_packages:
        src_dir = bash_command(rospack_cmd + pkg_name)
        #ipdb.set_trace()
        if src_dir is False:
            print(PRFIX + '[Fail] Can not find package: ', pkg_name)
            failure_list.append(pkg_name)
            continue
        dst_dir = os.path.join(target_dir, pkg_name)
        #print('src_dir: ', src_dir)
        #print('dst_dir: ', dst_dir)
        bash_command('rm -rf {}'.format(dst_dir))
        bash_command('cp -rf {} {}'.format(src_dir, dst_dir))
        bash_command('rm -rf {}/.git/'.format(dst_dir))
        print(PRFIX + pkg_name + ' has been updated!')

    # For meta packages
    for meta_name, sub_pkgs in meta_packages.items():
        meta_dir = os.path.join(target_dir, meta_name)
        makedirs(meta_dir)
        for pkg_name in sub_pkgs:
            src_dir = bash_command(rospack_cmd + pkg_name)
            #ipdb.set_trace()
            if src_dir is False:
                print(PRFIX + '[Fail] Can not find package: ', pkg_name)
                failure_list.append(pkg_name)
                continue
            dst_dir = os.path.join(meta_dir, pkg_name)
            #print('src_dir: ', src_dir)
            #print('dst_dir: ', dst_dir)
            bash_command('rm -rf {}'.format(dst_dir))
            bash_command('cp -rf {} {}'.format(src_dir, dst_dir))
            bash_command('rm -rf {}/.git/'.format(dst_dir))
            print(PRFIX + pkg_name + ' has been updated!')

    print('\n' + PRFIX + 'Failed updating case(s): {}'.format(len(failure_list)))
    print(PRFIX + 'List of copy failed packages:')
    print(*failure_list, sep='\n')

    print('\n' + PRFIX + 'Finished !')

if __name__ == "__main__":
    # Retrieve arguments
    ARGS = parse_args()
    # Run main function
    main(ARGS)
