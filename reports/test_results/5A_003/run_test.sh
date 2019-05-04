#!/usr/local/bin/bash

export TGT_DIR="/Users/dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Code/Animesh/reports/test_results/5A_003"
export PROJ_DIR="/Users/dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Code/Animesh"

"${PROJ_DIR}"/bin/mesher -d "/Users/dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Code/Animesh/data/CheatData/two_frames"
java -jar "${PROJ_DIR}"/java/animesh/target/animesh.jar "${TGT_DIR}" surfel_table.bin > "${TGT_DIR}"/err_0.txt

"${PROJ_DIR}"/bin/mesher -s surfel_table.bin
cp surfel_table_converged.bin "${TGT_DIR}"/st_50K.bin
java -jar "${PROJ_DIR}"/java/animesh/target/animesh.jar "${TGT_DIR}" st_50K.bin > "${TGT_DIR}"/err_50K.txt

"${PROJ_DIR}"/bin/mesher -s surfel_table_converged.bin
cp surfel_table_converged.bin "${TGT_DIR}"/st_100K.bin
java -jar "${PROJ_DIR}"/java/animesh/target/animesh.jar "${TGT_DIR}" st_100K.bin > "${TGT_DIR}"/err_100K.txt

"${PROJ_DIR}"/bin/mesher -s surfel_table_converged.bin
cp surfel_table_converged.bin "${TGT_DIR}"/st_150K.bin
java -jar "${PROJ_DIR}"/java/animesh/target/animesh.jar "${TGT_DIR}" st_150K.bin > "${TGT_DIR}"/err_150K.txt

"${PROJ_DIR}"/bin/mesher -s surfel_table_converged.bin
cp surfel_table_converged.bin "${TGT_DIR}"/st_200K.bin
java -jar "${PROJ_DIR}"/java/animesh/target/animesh.jar "${TGT_DIR}" st_200K.bin > "${TGT_DIR}"/err_200K.txt

"${PROJ_DIR}"/bin/mesher -s surfel_table_converged.bin
cp surfel_table_converged.bin "${TGT_DIR}"/st_250K.bin
java -jar "${PROJ_DIR}"/java/animesh/target/animesh.jar "${TGT_DIR}" st_250K.bin > "${TGT_DIR}"/err_250K.txt

"${PROJ_DIR}"/bin/mesher -s surfel_table_converged.bin -m -f 1
cp mat.txt "${TGT_DIR}"/st_250K_mat.txt


