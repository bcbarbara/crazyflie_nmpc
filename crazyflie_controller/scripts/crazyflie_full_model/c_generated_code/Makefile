#
# Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
# Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
# Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
# Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
#
# This file is part of acados.
#
# The 2-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.;
#

ACADOS_FLAGS = -fPIC

OBJ =
OBJ+= main_crazyflie.o

OBJ+= crazyflie_model/crazyflie_expl_ode_fun.o
OBJ+= crazyflie_model/crazyflie_expl_vde_forw.o




OBJ+= acados_solver_crazyflie.o
OBJ+= acados_sim_solver_crazyflie.o

INCLUDE_PATH = /home/barbara/catkin_ws/src/crazyflie_ros/crazyflie_controller/scripts/crazyflie_full_model/../../../acados//include
LIB_PATH = /home/barbara/catkin_ws/src/crazyflie_ros/crazyflie_controller/scripts/crazyflie_full_model/../../../acados//lib

all: clean casadi_fun main solver sim_solver example

CASADI_MODEL_SOURCE= 

CASADI_MODEL_SOURCE+= crazyflie_expl_ode_fun.c
CASADI_MODEL_SOURCE+= crazyflie_expl_vde_forw.c 


CASADI_CON_P_SOURCE=

CASADI_CON_H_SOURCE=


casadi_fun:
	( cd crazyflie_model; gcc $(ACADOS_FLAGS) -c  $(CASADI_MODEL_SOURCE))
	
	

main:
	gcc $(ACADOS_FLAGS) -c main_crazyflie.c -I $(INCLUDE_PATH)/blasfeo/include/ -I $(INCLUDE_PATH)/hpipm/include/ \
	-I $(INCLUDE_PATH) -I $(INCLUDE_PATH)/acados/ -I $(INCLUDE_PATH)/qpOASES_e/

solver:
	gcc $(ACADOS_FLAGS) -c acados_solver_crazyflie.c -I $(INCLUDE_PATH)/blasfeo/include/ -I $(INCLUDE_PATH)/hpipm/include/ \
	-I $(INCLUDE_PATH) -I $(INCLUDE_PATH)/acados/ -I $(INCLUDE_PATH)/qpOASES_e/

sim_solver:
	gcc $(ACADOS_FLAGS) -c acados_sim_solver_crazyflie.c -I $(INCLUDE_PATH)/blasfeo/include/ -I $(INCLUDE_PATH)/hpipm/include/ \
	-I $(INCLUDE_PATH) -I $(INCLUDE_PATH)/acados/ -I $(INCLUDE_PATH)/qpOASES_e/

example:
	gcc $(ACADOS_FLAGS) -o main_crazyflie $(OBJ) -L $(LIB_PATH) \
	-L $(LIB_PATH)/acados \
	-L $(LIB_PATH)/external/blasfeo \
	-L $(LIB_PATH)/external/hpipm  \
	-L $(LIB_PATH)/external/qpoases/lib \
	-lacados -lhpipm -lblasfeo -lqpOASES_e -lm \
	-I $(INCLUDE_PATH)/blasfeo/include/ \
	-I $(INCLUDE_PATH)/hpipm/include/ \
	-I $(INCLUDE_PATH) \
	-I $(INCLUDE_PATH)/acados/ \
	-I $(INCLUDE_PATH)/qpOASES_e/

shared_lib: casadi_fun solver sim_solver
	gcc $(ACADOS_FLAGS) -shared -o libacados_solver_crazyflie.so $(OBJ) -L $(LIB_PATH)/acados \
	-L $(LIB_PATH) \
	-L $(LIB_PATH)/external/blasfeo \
	-L $(LIB_PATH)/external/hpipm  \
	-L $(LIB_PATH)/external/qpoases/lib \
	-lacados -lhpipm -lblasfeo -lqpOASES_e -lm \

clean:
	rm -f *.o
	rm -f *.so
	rm -f main_crazyflie