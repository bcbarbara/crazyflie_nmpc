ACADOS_FLAGS = -fPIC

OBJ =
OBJ+= main_crazyflie_simple_model.o

OBJ+= crazyflie_simple_model_model/crazyflie_simple_model_expl_ode_fun.o
OBJ+= crazyflie_simple_model_model/crazyflie_simple_model_expl_vde_forw.o




OBJ+= acados_solver_crazyflie_simple_model.o

INCLUDE_PATH = /usr/local/include
LIB_PATH = /usr/local/lib

all: clean casadi_fun main solver example

CASADI_MODEL_SOURCE= 

CASADI_MODEL_SOURCE+= crazyflie_simple_model_expl_ode_fun.c
CASADI_MODEL_SOURCE+= crazyflie_simple_model_expl_vde_forw.c 


CASADI_CON_P_SOURCE=

CASADI_CON_H_SOURCE=


casadi_fun:
	( cd crazyflie_simple_model_model; gcc $(ACADOS_FLAGS) -c  $(CASADI_MODEL_SOURCE))
	
	

main:
	gcc $(ACADOS_FLAGS) -c main_crazyflie_simple_model.c -I $(INCLUDE_PATH)/blasfeo/include/ \
	-I $(INCLUDE_PATH) -I $(INCLUDE_PATH)/acados/ -I $(INCLUDE_PATH)/qpOASES_e/

solver:
	gcc $(ACADOS_FLAGS) -c acados_solver_crazyflie_simple_model.c -I $(INCLUDE_PATH)/blasfeo/include/ \
	-I $(INCLUDE_PATH) -I $(INCLUDE_PATH)/acados/ -I $(INCLUDE_PATH)/qpOASES_e/

example:
	gcc $(ACADOS_FLAGS) -o main_crazyflie_simple_model $(OBJ) -L $(LIB_PATH) \
	-L $(LIB_PATH)/acados \
	-L $(LIB_PATH)/external/blasfeo \
	-L $(LIB_PATH)/external/hpipm  \
	-L $(LIB_PATH)/external/qpoases/lib \
	-lacados -lhpipm -lblasfeo -lqpOASES_e -lm \
	-I $(INCLUDE_PATH)/blasfeo/include/ \
	-I $(INCLUDE_PATH) \
	-I $(INCLUDE_PATH)/acados/ \
	-I $(INCLUDE_PATH)/qpOASES_e/

shared_lib: casadi_fun solver
	gcc $(ACADOS_FLAGS) -shared -o libacados_solver_crazyflie_simple_model.so $(OBJ) -L $(LIB_PATH)/acados \
	-L $(LIB_PATH) \
	-L $(LIB_PATH)/external/blasfeo \
	-L $(LIB_PATH)/external/hpipm  \
	-L $(LIB_PATH)/external/qpoases/lib \
	-lacados -lhpipm -lblasfeo -lqpOASES_e -lm \

clean:
	rm -f *.o
	rm -f *.so
	rm -f main_crazyflie_simple_model