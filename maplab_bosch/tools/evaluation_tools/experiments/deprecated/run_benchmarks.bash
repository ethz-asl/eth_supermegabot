#!/bin/bash
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 results_folder" >&2
    exit 1
fi

RESULTS_DIR=$1

mkdir -p ${RESULTS_DIR}

rosrun ze_evaluation_job experiment.py vo_benchmark_tum           --timestamp_result_folder=False --output_folder=${RESULTS_DIR} 
rosrun ze_evaluation_job experiment.py vo_benchmark_euroc_mono    --timestamp_result_folder=False --output_folder=${RESULTS_DIR}
rosrun ze_evaluation_job experiment.py vo_benchmark_euroc_stereo  --timestamp_result_folder=False --output_folder=${RESULTS_DIR}

rosrun ze_evaluation_job experiment.py vio_benchmark_euroc --timestamp_result_folder=False --output_folder=${RESULTS_DIR}
rosrun ze_evaluation_job experiment.py vio_benchmark_euroc_svo --timestamp_result_folder=False --output_folder=${RESULTS_DIR}
rosrun ze_evaluation_job experiment.py vio_benchmark_maltid --timestamp_result_folder=False --output_folder=${RESULTS_DIR}
rosrun ze_evaluation_job experiment.py vio_benchmark_maltid_svo --timestamp_result_folder=False --output_folder=${RESULTS_DIR}
rosrun ze_evaluation_job experiment.py vio_benchmark_prius_apr28 --timestamp_result_folder=False --output_folder=${RESULTS_DIR}
rosrun ze_evaluation_job experiment.py vio_benchmark_prius_apr28_svo --timestamp_result_folder=False --output_folder=${RESULTS_DIR}


