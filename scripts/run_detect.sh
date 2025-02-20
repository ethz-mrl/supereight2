#!/bin/sh

base_path="/usr/wiss/juja/storage/group/srl/slamAndMapping/Autoassess/gazebo/bwt_01/meshes_2cm_numfree11/"
input_file="$base_path/edges_2359.ply"

rm -r $base_path/manholes
mkdir $base_path/manholes

rm -r $base_path/edges3d
mkdir $base_path/edges3d

rm -r $base_path/closed_edges
mkdir $base_path/closed_edges

rm -r $base_path/plane_ransac
mkdir $base_path/plane_ransac

rm -r $base_path/manhole_center_position
mkdir $base_path/manhole_center_position

./build/release/app/test_closed_edges $input_file \
    2>&1 | tee -a $base_path/output.log