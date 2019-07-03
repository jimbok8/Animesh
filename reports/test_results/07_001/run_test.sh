#!/usr/local/bin/bash

export TGT_DIR="/Users/dave/Animesh/reports/test_results/07_001"
export PROJ_DIR="/Users/dave/Animesh"
export ANIMESH_JAR="${PROJ_DIR}""/java/animesh-tools/animeshtools/target/animesh-tools-1.0-SNAPSHOT.jar"
export MEASURE="org.ddurbin.animesh.tools.MeasureError"

if [ ! -f "surfel_table.bin" ]; then
	"${PROJ_DIR}"/bin/mesher -d "/Users/dave/Animesh/data/CheatData/cloth"
fi
java -cp "${ANIMESH_JAR}" "${MEASURE}" "${TGT_DIR}" surfel_table.bin > "${TGT_DIR}"/err_0.txt

"${PROJ_DIR}"/bin/mesher -s surfel_table.bin
cp surfel_table_converged.bin "${TGT_DIR}"/st_50K.bin
java -cp "${ANIMESH_JAR}" "${MEASURE}" "${TGT_DIR}" st_50K.bin > "${TGT_DIR}"/err_50K.txt

"${PROJ_DIR}"/bin/mesher -s surfel_table_converged.bin
cp surfel_table_converged.bin "${TGT_DIR}"/st_100K.bin
java -cp "${ANIMESH_JAR}" "${MEASURE}" "${TGT_DIR}" st_100K.bin > "${TGT_DIR}"/err_100K.txt

"${PROJ_DIR}"/bin/mesher -s surfel_table_converged.bin
cp surfel_table_converged.bin "${TGT_DIR}"/st_150K.bin
java -cp "${ANIMESH_JAR}" "${MEASURE}" "${TGT_DIR}" st_150K.bin > "${TGT_DIR}"/err_150K.txt

"${PROJ_DIR}"/bin/mesher -s surfel_table_converged.bin
cp surfel_table_converged.bin "${TGT_DIR}"/st_200K.bin
java -cp "${ANIMESH_JAR}" "${MEASURE}" "${TGT_DIR}" st_200K.bin > "${TGT_DIR}"/err_200K.txt

"${PROJ_DIR}"/bin/mesher -s surfel_table_converged.bin
cp surfel_table_converged.bin "${TGT_DIR}"/st_250K.bin
java -cp "${ANIMESH_JAR}" "${MEASURE}" "${TGT_DIR}" st_250K.bin > "${TGT_DIR}"/err_250K.txt

"${PROJ_DIR}"/bin/mesher -s surfel_table_converged.bin -m -f 1
cp mat.txt "${TGT_DIR}"/st_250K_mat.txt


