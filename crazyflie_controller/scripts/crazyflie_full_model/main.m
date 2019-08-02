clear all;
close all;

cd c_generated_code/
make_sfun
cd ..

mex_source = 'c_generated_code/acados_solver_sfunction_crazyflie.mexmaci64';
copyfile(mex_source,'acados_solver_sfunction_crazyflie.mexmaci64');

%%
N = 50; Ts = 0.02;

states = [0 0 0 1 0 0 0 0 0 0 0 0 0];
u = [0 0 0 0];
yref = [];

for i=1:N
    yref    = [yref states u];
end

yref_e  = states;

open('crazyflie_full_model');