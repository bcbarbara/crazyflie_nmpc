# MIT License

# Copyright (c) 2025 Barbara Barros Carlos, Tommaso Sartor

# This file is part of the crazyflie_nmpc project.

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the “Software”), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from dataclasses import dataclass


@dataclass
class ModelParameters:
    g0: float = 9.8066      # Acceleration of gravity [m/s^2]
    mass: float = 33e-3     # Total mass (with one marker) [kg]
    Ixx: float = 1.395e-5   # Inertia moment around x-axis [kg.m^2]
    Iyy: float = 1.395e-5   # Inertia moment around y-axis [kg.m^2]
    Izz: float = 2.173e-5   # Inertia moment around z-axis [kg.m^2]
    Cd: float = 7.9379e-6   # Drag coefficient [N/krpm^2]
    Ct: float = 3.25e-4     # Thrust coefficient [N/krpm^2]
    l: float = 65e-3 / 2    # Distance between motors' center and the axis of rotation [m]

@dataclass
class SolverConfig:
    N: int = 50             # Number of shooting intervals [-]
    tf: float = 0.75        # Time horizon [s]
