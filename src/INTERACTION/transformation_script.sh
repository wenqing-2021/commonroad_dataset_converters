#!/bin/bash
source ~/anaconda3/etc/profile.d/conda.sh

# prints the input
function print_my_input() {
  echo 'Your input: ' $1
}

function commonroad_preprocess(){
        conda activate cr36
        python ~/Documents/Uni/000.001_WS_20_21/Praktikum/Code/commonroad_rl_preprocess.py "$@"
        conda deactivate
}

function commonroad_transform_check(){
        osm-convert $1 -o $2 -f
        conda activate cr36
        python -m commonroad_rl.tools.validate_cr -s commonroad_rl/tools/XML_commonRoad_XSD_2020a.xsd $2
        #python ../commonroad_fitter/commonroad_info.py -i $2
        conda deactivate

}

function commonroad_transform_check_all(){
        mkdir $1
        #$path = "$( cd "$( dirname "$0" )" && pwd )"
        #cd $2
        for filepath in $2/*.osm; do
                filename=$(basename $filepath)
                no_ext="${filename%.*}"
                commonroad_transform_check $filepath $1/$no_ext.xml
        done
        #cd $path

}

