
			switch(policy)
				{
				case Regulation:
					{
					// Update regulation point
					for (k = 0; k < N+1; k++)
						{
						acados_in.yref[k * NY + 0] = xq_des;	// xq
						acados_in.yref[k * NY + 1] = yq_des;	// yq
						acados_in.yref[k * NY + 2] = zq_des;	// zq
						acados_in.yref[k * NY + 3] = 1.00;		// q1
						acados_in.yref[k * NY + 4] = 0.00;		// q2
						acados_in.yref[k * NY + 5] = 0.00;		// q3
						acados_in.yref[k * NY + 6] = 0.00;		// q4
						acados_in.yref[k * NY + 7] = 0.00;		// vbx
						acados_in.yref[k * NY + 8] = 0.00;		// vby
						acados_in.yref[k * NY + 9] = 0.00;		// vbz
						acados_in.yref[k * NY + 10] = 0.00;		// wx
						acados_in.yref[k * NY + 11] = 0.00;		// wy
						acados_in.yref[k * NY + 12] = 0.00;		// wz
						acados_in.yref[k * NY + 13] = uss;		// w1
						acados_in.yref[k * NY + 14] = uss;		// w2
						acados_in.yref[k * NY + 15] = uss;		// w3
						acados_in.yref[k * NY + 16] = uss;		// w4
						}
					break;
					}
				case Tracking:
					{
					if(iter >= N_STEPS-N)
						{
						policy = Position_Hold;
						break;
						}
					// Update reference
					for (k = 0; k < N+1; k++)
						{
						acados_in.yref[k * NY + 0]  = precomputed_traj[iter + k][xq];
						acados_in.yref[k * NY + 1]  = precomputed_traj[iter + k][yq];
						acados_in.yref[k * NY + 2]  = precomputed_traj[iter + k][zq];
						acados_in.yref[k * NY + 3]  = precomputed_traj[iter + k][q1];
						acados_in.yref[k * NY + 4]  = precomputed_traj[iter + k][q2];
						acados_in.yref[k * NY + 5]  = precomputed_traj[iter + k][q3];
						acados_in.yref[k * NY + 6]  = precomputed_traj[iter + k][q4];
						acados_in.yref[k * NY + 7]  = precomputed_traj[iter + k][vbx];
						acados_in.yref[k * NY + 8]  = precomputed_traj[iter + k][vby];
						acados_in.yref[k * NY + 9]  = precomputed_traj[iter + k][vbz];
						acados_in.yref[k * NY + 10] = precomputed_traj[iter + k][wx];
						acados_in.yref[k * NY + 11] = precomputed_traj[iter + k][wy];
						acados_in.yref[k * NY + 12] = precomputed_traj[iter + k][wz];
						acados_in.yref[k * NY + 13] = precomputed_traj[iter + k][13];
						acados_in.yref[k * NY + 14] = precomputed_traj[iter + k][14];
						acados_in.yref[k * NY + 15] = precomputed_traj[iter + k][15];
						acados_in.yref[k * NY + 16] = precomputed_traj[iter + k][16];
						}
					++iter;
					break;
					}
				case Position_Hold:
					{
					ROS_INFO("Holding last position of the trajectory.");
					// Get last point of tracketory and hold
					for (k = 0; k < N+1; k++)
						{
						acados_in.yref[k * NY + 0] = precomputed_traj[N_STEPS-1][xq];
						acados_in.yref[k * NY + 1] = precomputed_traj[N_STEPS-1][yq];
						acados_in.yref[k * NY + 2] = precomputed_traj[N_STEPS-1][zq];
						acados_in.yref[k * NY + 3] = 1.00;
						acados_in.yref[k * NY + 4] = 0.00;
						acados_in.yref[k * NY + 5] = 0.00;
						acados_in.yref[k * NY + 6] = 0.00;
						acados_in.yref[k * NY + 7] = 0.00;
						acados_in.yref[k * NY + 8] = 0.00;
						acados_in.yref[k * NY + 9] = 0.00;
						acados_in.yref[k * NY + 10] = 0.00;
						acados_in.yref[k * NY + 11] = 0.00;
						acados_in.yref[k * NY + 12] = 0.00;
						acados_in.yref[k * NY + 13] = uss;
						acados_in.yref[k * NY + 14] = uss;
						acados_in.yref[k * NY + 15] = uss;
						acados_in.yref[k * NY + 16] = uss;
						}
					break;
					}
				}

