#! /bin/bash
DATASET_PATH=/home/rl_students_ss20/Peter/data/inD-dataset-v1.0/data
PICKLE_PATH=/home/rl_students_ss20/Peter/data/dataset/inD_2020

python -m commonroad_rl.utils_ind.ind_to_cr -t -1 -n -1 -i ${DATASET_PATH} -o ${PICKLE_PATH} -p -v --multiprocessing 2> conversion_log.txt
