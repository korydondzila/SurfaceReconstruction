################################################################
###
###  Makefile for use with OpenACC training
###  NPB SP Benchmark
###  C version ported by Center for Manycore Programming
###  Seoul National University
###  http://aces.snu.ac.kr/Center_for_Manycore_Programming/SNU_NPB_Suite.html
################################################################

BIN_DIR=bin
OBJ_DIR=obj
SRC_DIR=src
OBJS= $(OBJ_DIR)/BaseEntity.o            \
	  $(OBJ_DIR)/DynamicCamera.o         \
	  $(OBJ_DIR)/Mesh.o                  \
	  $(OBJ_DIR)/MessageDispatcher.o     \
	  $(OBJ_DIR)/PointCloud.o            \
	  $(OBJ_DIR)/Principal.o             \
	  $(OBJ_DIR)/Scene.o                 \
	  $(OBJ_DIR)/Spatial.o               \
	  $(OBJ_DIR)/StaticCamera.o          \
	  $(OBJ_DIR)/StaticEntity.o          \
      $(OBJ_DIR)/SurfaceReconstruction.o \

CC=pgc++
INC=-I$(SRC_DIR) -I$(SRC_DIR)/includes
LINK=-lGLEW -lglut -lGL
OPT=-std c++14 -w -fast
ACC=-acc -Minfo=accel -ta=nvidia,nocache
EXE= $(BIN_DIR)/SurfaceReconstruction.out

all:  setup build

build: $(OBJS) $(EXE)

$(BIN_DIR)/SurfaceReconstruction.out: $(OBJS)
	$(CC) $(INC) $(OPT) $(LINK) $(ACC) $(OBJ_DIR)/*.o -o $(BIN_DIR)/SurfaceReconstruction.out

$(OBJ_DIR)/BaseEntity.o: $(SRC_DIR)/BaseEntity.cpp
	$(CC) -c $(INC) $(OPT) $(LINK) $(ACC) -o $(OBJ_DIR)/BaseEntity.o $(SRC_DIR)/BaseEntity.cpp
	
$(OBJ_DIR)/DynamicCamera.o: $(SRC_DIR)/DynamicCamera.cpp
	$(CC) -c $(INC) $(OPT) $(LINK) $(ACC) -o $(OBJ_DIR)/DynamicCamera.o $(SRC_DIR)/DynamicCamera.cpp
	
$(OBJ_DIR)/Mesh.o: $(SRC_DIR)/Mesh.cpp
	$(CC) -c $(INC) $(OPT) $(LINK) $(ACC) -o $(OBJ_DIR)/Mesh.o $(SRC_DIR)/Mesh.cpp
	
$(OBJ_DIR)/MessageDispatcher.o: $(SRC_DIR)/MessageDispatcher.cpp
	$(CC) -c $(INC) $(OPT) $(LINK) $(ACC) -o $(OBJ_DIR)/MessageDispatcher.o $(SRC_DIR)/MessageDispatcher.cpp
	
$(OBJ_DIR)/PointCloud.o: $(SRC_DIR)/PointCloud.cpp
	$(CC) -c $(INC) $(OPT) $(LINK) $(ACC) -o $(OBJ_DIR)/PointCloud.o $(SRC_DIR)/PointCloud.cpp
	
$(OBJ_DIR)/Principal.o: $(SRC_DIR)/Principal.cpp
	$(CC) -c $(INC) $(OPT) $(LINK) $(ACC) -o $(OBJ_DIR)/Principal.o $(SRC_DIR)/Principal.cpp
	
$(OBJ_DIR)/Scene.o: $(SRC_DIR)/Scene.cpp
	$(CC) -c $(INC) $(OPT) $(LINK) $(ACC) -o $(OBJ_DIR)/Scene.o $(SRC_DIR)/Scene.cpp
	
$(OBJ_DIR)/Spatial.o: $(SRC_DIR)/Spatial.cpp
	$(CC) -c $(INC) $(OPT) $(LINK) $(ACC) -o $(OBJ_DIR)/Spatial.o $(SRC_DIR)/Spatial.cpp
	
$(OBJ_DIR)/StaticCamera.o: $(SRC_DIR)/StaticCamera.cpp
	$(CC) -c $(INC) $(OPT) $(LINK) $(ACC) -o $(OBJ_DIR)/StaticCamera.o $(SRC_DIR)/StaticCamera.cpp
	
$(OBJ_DIR)/StaticEntity.o: $(SRC_DIR)/StaticEntity.cpp
	$(CC) -c $(INC) $(OPT) $(LINK) $(ACC) -o $(OBJ_DIR)/StaticEntity.o $(SRC_DIR)/StaticEntity.cpp

$(OBJ_DIR)/SurfaceReconstruction.o: $(SRC_DIR)/SurfaceReconstruction.cpp
	$(CC) -c $(INC) $(OPT) $(LINK) $(ACC) -o $(OBJ_DIR)/SurfaceReconstruction.o $(SRC_DIR)/SurfaceReconstruction.cpp


setup: $(OBJ_DIR) $(BIN_DIR)

$(OBJ_DIR):
	mkdir $(OBJ_DIR)

$(BIN_DIR):
	mkdir $(BIN_DIR)

clean:
	rm -rf $(OBJ_DIR)/*.o
	rm -rf $(BIN_DIR)/*

reallyclean:
	rm -rf obj*
	rm -rf bin*














