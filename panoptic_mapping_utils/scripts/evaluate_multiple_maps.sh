#!/bin/bash
# Run this script to setup semantic labelling.

target_dir="/home/lukas/Documents/PanopticMapping/Data/evaluations/fixed_resolution"
target_files=("3cm" "5cm" "10cm" "20cm")


# Evaluation
for target_file in "${target_files[@]}"; do
  roslaunch panoptic_mapping_utils evaluate_panmap.launch map_file:="$target_dir/$target_file.panmap" visualize:=false
done
