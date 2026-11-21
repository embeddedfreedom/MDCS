Copyright (C) 2026 Diptopal Basu (embeddedfreedom)

This program is free software: you can redistribute it and/or modify

it under the terms of the GNU General Public License as published by

the Free Software Foundation, either version 3 of the License, or

(at your option) any later version.

This program is distributed in the hope that it will be useful,

but WITHOUT ANY WARRANTY; without even the implied warranty of

MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the

GNU General Public License for more details.

You should have received a copy of the GNU General Public License

along with this program. If not, see https://www.gnu.org/licenses/.

Pendulum Controller: 1kHz Compensator / 20kHz Simulation

title: "Analytical Derivation and Control of the Furuta Pendulum"

subtitle: "State-Space Linearization via Jacobian Method and LQR Implementation"

author: "Diptopal Basu" date: "February 2026"

version: 1.0 tags: [Control Theory, Robotics, Furuta Pendulum, LQR, Jacobian Linearization]

Executive Summary & Configuration: This document provides a rigorous mathematical derivation of the
Furuta (Rotary Inverted) Pendulum dynamics using Lagrangian Mechanics. It concludes with the
linearization of the non-linear equations into a State-Space model (Ax + Bu) and provides the Python
implementation for LQR gain calculation.
0. System Configuration & Coordinate Anchor
                                          Parameter /
      Category                                                                                                                              Physical Interpretation / Value
                                          Convention

                                                                                                  0 is "Down" (Stable); π (180∘ ) is "Up"
  Angle Reference         Zero Position
                                                                                                  (Unstable).

  Rotational Logic        Sign Convention                                                         CCW is Positive (+); CW is Negative (-).

  System Rates            Timing                                                                  Simulation: 20 kHz; Compensator: 1 kHz.

                                                                                                  [θ1 , θ2 , θ̇1 , θ̇2 ]T → [Arm, Pend, Arm_Vel,
  State Vector            x
                                                                                                                ​                       ​               ​               ​




                                                                                                  Pend_Vel].

                                                                                                  u = (k3 θ1 ) + (k1 (θ2 − π)) + (k4 θ̇1 ) +
  Control Logic
                                                                                                                                                ​           ​                           ​       ​                       ​   ​




                          u
                                                                                                  (k2 θ̇2 ).        ​               ​




  Mass Properties         M1 , M2 , M3        ​                           ​               ​
                                                                                                  M1 = 0.12, M2 = 0.15, M3 = 0.05 (kg).
                                                                                                                        ​                                                           ​                           ​




  Shaft/Hub               Msh , Rsh               ​                               ​
                                                                                                  Mshaf t = 0.06 (kg), Rshaf t = 0.005 (m). ​                                                               ​




                                                                                                  L1 = 0.2, L2 = 0.4, L3 = 0.2 (m). (L2 sits
                                                                                                            ​                                                       ​                                   ​




  Dimensions              L1 , L2 , L3​                           ​                   ​
                                                                                                  on L1 as a T joint, the middle of L2 attached to
                                                                                                  L1 rigidly)

  Radii (Rods)            r1 , r2 , r3
                                  ​                           ​               ​
                                                                                                  r1 , r2 , r3 = 0.01 (m).
                                                                                                        ​                       ​                   ​




                                                                                                  b1 = 0.008, b2 = 0.001 (Nms/rad), g = 9.81
  Damping/Gravity         b1 , b2 , g
                                                                                                    ​                                                                       ​




                                                                                                  (m/s2 ).
                              ​                           ​




  Motor Specs             Kt , Kb , R     ​                           ​
                                                                                                  Kt = 0.12, Kb = 0.12, R = 2.5 (Ω).
                                                                                                                ​                                                               ​




  Voltage/Drive           Vmax , Deadzone             ​
                                                                                                  Vmax = 12.0 (V), Deadzone = 0.4 (V).
                                                                                                                            ​




Crucial Derivation Notes for the Student

  1. The π Offset: Because we linearize around the upright position, the "Pendulum Position"
     variable in our LQR code is always the error term: Δθ2 = (θ2 − π).                                                                                         ​                           ​




  2. Motor Back-EMF: The parameter σ1 in the A matrix is not just b1 . It is the "Combined    ​                                                                                                     ​




     Damping" that includes both mechanical friction and electrical resistance: σ1 = −(b1 +                                                                                                                         ​           ​




     Kt Kb
      R
       ​




           ).
           ​




               ​




  3. The Deadzone: While the LQR math assumes a perfect linear motor, the code must
     compensate for the 0.4V deadzone before sending the final PWM signal to the motor
     driver.
1. Kinematics (Full Expansion)

We start with the position of the pendulum center of mass (P3 ).                                                                                                                                                                                                                                                                                                                                                                           ​




The base rotates by θ1 and the pendulum by θ2 .                                                                                                                                      ​                                                                                                                                                                         ​




x3 = L22 cos θ1 − L23 sin θ2 sin θ1
          ​
                                 ​




                                             ​                                                           ​
                                                                                                                                                                         ​




                                                                                                                                                                                 ​                                                           ​                                                    ​




y3 = L22 sin θ1 + L23 sin θ2 cos θ1
      ​
                         ​




                                     ​                                                               ​
                                                                                                                                                                 ​




                                                                                                                                                                     ​                                                               ​                                                        ​




z3 = L1 − L23 cos θ2
  ​                                      ​
                                                                                             ​




                                                                                                 ​                                                                           ​




Velocity Expansion
                             d
Taking the time derivative ( dt ) of each component:                                                                                                                                                                                     ​




ẋ3 = − L22 θ̇1 sin θ1 − L23 θ̇2 cos θ2 sin θ1 − L23 θ̇1 sin θ2 cos θ1
                                                             ​




                                                                 ​                   ​                                                                   ​                                               ​                                       ​                                                    ​                                    ​
                                                                                                                                                                                                                                                                                                                                                                   ​




                                                                                                                                                                                                                                                                                                                                                                       ​               ​                                           ​                       ​




ẏ3 = L22 θ̇1 cos θ1 + L23 θ̇2 cos θ2 cos θ1 − L23 θ̇1 sin θ2 sin θ1
                                 ​




                                             ​                               ​                                               ​
                                                                                                                                                                                                 ​




                                                                                                                                                                                                     ​       ​                                                                ​                                                ​
                                                                                                                                                                                                                                                                                                                                                       ​




                                                                                                                                                                                                                                                                                                                                                           ​                   ​                                       ​                       ​




ż3 = L23 θ̇2 sin θ2 ​




                             ​                                       ​                                                   ​




The Velocity Square (v32 )                                                                                                                                                                                                                                   ​




To find v32 = ẋ23 + ẏ32 + ż32 , we square the terms. Notice that when summing ẋ23 + ẏ32 , the
                                                     ​                                                   ​                                                   ​               ​                                           ​                                                                                                                                                                                                                                                               ​       ​   ​




                                                                                                                                                                                                                                                                                                                                                                                                                                                               2                     2
cross-terms containing sin θ1 cos θ1 cancel out. After applying sin                                                                                                                                                                                  ​                                                    ​
                                                                                                                                                                                                                                                                                                                                                                                                                                                                   θ + cos θ = 1
repeatedly, we get:

                L2                                                                                   L2                                                                                                                                                              L2                                                                                                                                                                                             L2
v32 = 42 θ̇12 + 43 θ̇22 cos2 θ2 + 43 θ̇12 sin2 θ2 + L22L3 θ̇1 θ̇2 cos θ2 + 43 θ̇22 sin2 θ2
                         ​                                                                                       ​                                                                                                                                                        ​                                                                                                                                                                                              ​
                                                                                                                                                                                                                                                                                                                                                                                   ​       ​




      ​                              ​                                   ​                                           ​                               ​                                                                           ​                                            ​                               ​                                ​                                               ​           ​                   ​                       ​                     ​   ​           ​




Combining cos2 θ2 and sin2 θ2 for the θ̇22 term:                                                                                 ​                                                                                                                       ​                                                                         ​




v32 = θ̇12 ( 42 + 43 sin2 θ2 ) + θ̇22 ( 43 ) + θ̇1 θ̇2 ( L22L3 cos θ2 )
                                                                 L2              ​
                                                                                                                 L2                      ​
                                                                                                                                                                                                                                                                                                                  L2   ​
                                                                                                                                                                                                                                                                                                                                                                                                       ​       ​




      ​                  ​                                                                                                                   ​                                                                   ​                                                                    ​                                    ​                                   ​           ​                                       ​                               ​




2. Energy and Lagrangian
Inertia of Rigid Cylinders:

              Jhub = 12 Msh Rsh
                             2
                                + 12 M1 R12 + ( 12
                                                 1
                                                 ​
                                                   M2 L22 + 14 M2 R22 )                  ​                                           ​                                                       ​                               ​                                                            ​                                            ​           ​                   ​                           ​                       ​           ​




                       1
              Jpend = 12 M3 L23 + 14 M3 R32 (About COM)  ​                                                   ​                                   ​                                       ​                           ​                                           ​                ​




Kinetic Energy (T ):
T = 12 Jhub θ̇12 + 12 M3 v32 + 12 Jpend θ̇22
        ​           ​               ​                               ​           ​                   ​            ​                            ​                   ​




Substitute v32 :            ​




T = 12 Jhub θ̇12 + 12 M3 [θ̇12 ( 42 + 43 sin2 θ2 ) + θ̇22 43 + θ̇1 θ̇2 L22L3 cos θ2 ] + 12 Jpend θ̇22
                                                                                                                L2       ​
                                                                                                                                             L2           ​
                                                                                                                                                                                                      L2   ​
                                                                                                                                                                                                                                               ​   ​




        ​           ​               ​                               ​           ​                       ​                    ​                                ​               ​                   ​            ​       ​           ​                   ​           ​   ​       ​   ​




Potential Energy (V ):

V = M3 g(L1 − L23 cos θ2 )
                ​                       ​
                                                                        ​




                                                                            ​                               ​




The Explicit Definitions of a, b, c, d

Now we group these into the four physical constants used in the Lagrangian. These terms
represent the total effective mass/inertia seen by each axis.

     a (Total Arm-side Inertia): Includes everything that rotates with the motor shaft, plus the
     pendulum's mass acting as a point-load at the end of the arm (L2 /2).                                                                                                                                                                                 ​




                                                                                                                                                                  2
     a = Jsh + J1 + J2 + M3 ( L22 )
                                ​                                   ​                           ​                                ​
                                                                                                                                                  ​




                                                                                                                                                      ​




     b (Centrifugal Coefficient): Specifically the pendulum's mass moving in a circle around its
     pivot.

                                                            2
     b = M3 ( L23 )     ​
                                            ​




                                                ​




     c (Total Pendulum-side Inertia): The pendulum's own inertia plus its mass offset from the
     pivot (via the Parallel Axis Theorem).

                              1                                                                 2
     c = J3 + M3 ( L23 ) = ( 12
                    ​
                                M3 L23 + 14 M3 R32 ) + 14 M3 L23​
                                                                                ​




                                                                                        ​                                            ​                    ​           ​   ​               ​   ​                    ​           ​           ​




     d (Coupling Inertia): The geometric "bridge" between the arm and the pendulum.

     d = M3 ( L22 ) ( L23 )     ​
                                                    ​




                                                        ​
                                                                                    ​




                                                                                            ​




Your Hardware Parameters: Msh = 0.06, M1 = 0.12, M2 = 0.15, M3 = 0.05 L1 =                                                               ​                                            ​                                    ​                                   ​           ​




0.2, L2 = 0.4, L3 = 0.2 Rsh = 0.005, Rarm = 0.01, Rpend = 0.01
            ​                                               ​                                                        ​                                                            ​                                                    ​
  Parameter                                                                                                         Formula                                                                                                      Calculation                              Result
                                                                1      2
  Jsh      ​




                                                                2
                                                                  Msh Rsh
                                                                    ​                                                       ​                           ​
                                                                                                                                                                                        0.5 ⋅ 0.06 ⋅ 0.0052                                                           0.00000075
                                                                1
  J1   ​




                                                                2
                                                                  M1 R12
                                                                    ​                                       ​                           ​
                                                                                                                                                                                        0.5 ⋅ 0.12 ⋅ 0.012                                                            0.000006
                                                                 1
                                                                   M2 L22 +
                                                                                                                                                                                                                 2
                                                                12                                                                                                                      ( 0.15⋅0.4 ) + (0.25 ⋅ 0.15 ⋅
                                                                                                                                                                                             12                                                                       0.00200375
                                                                                    ​                                   ​                       ​




  J2
                                                                                                                                                                                                                     ​




                                                                1      2
                                                                                                                                                                                        0.012 )
       ​




                                                                4 M2 R2
                                                                    ​                                       ​                           ​




                                                                 1
                                                                   M3 L23 +
                                                                                                                                                                                                                 2
                                                                12                                                                                                                      ( 0.05⋅0.2 ) + (0.25 ⋅ 0.05 ⋅
                                                                                                                                                                                             12                                                                       0.00016792
                                                                                    ​                                   ​                       ​




  J3
                                                                                                                                                                                                                     ​




                                                                1      2                                                                                                                      2
                                                                                                                                                                                        0.01 )
       ​




                                                                4 M3 R3
                                                                    ​                                       ​                           ​




Final a, b, c, d values:

       a = Jsh + J1 + J2 + M3 (L2 /2)2 = 0.00000075 + 0.000006 + 0.00200375 +
                                            ​                                   ​                                                   ​                           ​               ​




       (0.05 ⋅ 0.22 ) = 0.0040105

       b = M3 (L3 /2)2 = 0.05 ⋅ 0.12 = 0.0005
                                        ​                   ​




       c = J3 + M3 (L3 /2)2 = 0.00016792 + 0.0005 = 0.0006679
                            ​                                               ​                                       ​




       d = M3 (L2 /2)(L3 /2) = 0.05 ⋅ 0.2 ⋅ 0.1 = 0.001
                                            ​                   ​                                                               ​




The Lagrangian (L = T − V ):

L = 12 (a + b sin2 θ2 )θ̇12 + 12 cθ̇22 + 2dθ̇1 θ̇2 cos θ2 + M32gL3 cos θ2
               ​                                                                                        ​                                   ​               ​           ​                ​       ​                       ​
                                                                                                                                                                                                                                          ​   ​




                                                                                                                                                                                                                                                              ​




3. The Euler-Lagrange Equations
      d ∂L     ∂L
Using dt ( ) − ∂θ = Qi :
                   ​




                       ∂ θ̇i    ​
                                    ​




                                                                                        i   ​
                                                                                                ​                                                   ​




For the Arm (θ1 ):                                      ​




(a + b sin2 θ2 )θ̈1 + (2d cos θ2 )θ̈2 + b sin(2θ2 )θ̇1 θ̇2 − 2d sin θ2 θ̇22 = τmotor − b1 θ̇1
                                                    ​                   ​                                                                                           ​       ​                                ​               ​        ​                   ​       ​   ​     ​   ​




For the Pendulum (θ2 ):                                                                                         ​




(2d cos θ2 )θ̈1 + cθ̈2 − 2b sin(2θ2 )θ̇12 + M32gL3 sin θ2 = −b2 θ̇2
                                                                                                                                                                                             ​       ​




                        ​                       ​                                                   ​                                           ​                                   ​                    ​                        ​               ​   ​




4. The Exact Jacobian: The Mathematical Bridge
To derive A and B , we must isolate the accelerations (θ̈1 , θ̈2 ) by solving the linearized system.                                                                                                        ​   ​




Around the equilibrium θ2 = π , we apply the small-angle approximation where cos θ2 ≈ −1
                                                                       ​                                                                                                                                                         ​




and sin θ2 ≈ 0.  ​




Step 4.1: The Linearized Equations of Motion

After dropping higher-order velocity terms (like θ̇ 2 ), the Euler-Lagrange equations simplify to a
linear system:

                                                                                                               K
  1. Arm: aθ̈1 − dθ̈2 = σ1 θ̇1 + Rt V
                                                                                                                    ​




                                              ​                            ​               ​                            ​




  2. Pendulum: −dθ̈1 + cθ̈2 + γp Δθ2 = −b2 θ̇2         ​                               ​                        ​                                   ​                                       ​       ​




Step 4.2: Decoupling via Matrix Inversion

We represent this as M θ̈ = F . To isolate accelerations, we multiply by the inverse mass matrix
M −1 :

                                               σ1 θ̇1 + KRt V
[ 1 ] = ac−d2 [                             ][                ]
 θ̈ ​




          1     c                         d                                        ​                   ​
                                                                                                                                    ​




                                                                                                                                            ​




                                          a γp Δθ2 − b2 θ̇2
        ​                    ​       ​             ​                                                                                                        ​




 θ̈2​           d                                                              ​                           ​                                    ​       ​




Step 4.3 State-Space Linearization (Jacobian Method)

To transform the nonlinear equations into the form ẋ = Ax + Bu, we compute the Jacobian
matrices at the upright equilibrium (θ2 = π, θ̇1 = 0, θ̇2 = 0).                                                             ​                                                           ​               ​




The Jacobian Definition

The A matrix is defined as:


                                         ⎡ ∂θ1                                                                                                          ∂ θ̇2 ⎤
                                                  ∂ θ̇1    ​
                                                                               ∂ θ̇1           ​
                                                                                                               ∂ θ̇1        ​
                                                                                                                                                        ∂ θ̇1       ​




                                                                               ∂θ2                             ∂ θ̇1
                                                               ​                                                                ​                                       ​




                                                           ​                                   ​
                                                                                                                            ​                                       ​




                               ∂ θ̇2                                           ∂ θ̇2                           ∂ θ̇2                                    ∂ θ̇2
            ∂f (x,u) ∣
                                                           ​                                   ​                            ​                                       ​




A=                          = ∂∂θθ̈1                                           ∂θ2                             ∂ θ̇1                                    ∂ θ̇2
                                                               ​                                                                ​                                       ​




                                                           ​                                   ​


                                                                                                                            ​                                       ​




              ∂x ∣                                                             ∂ θ̈1                           ∂ θ̈1                                    ∂ θ̈1
                         ​       ​        ​                        ​                               ​                                    ​                                   ​       ​




                         eq        1                       ​                                   ​                            ​                                       ​




                             ⎣ ∂ θ̈2                                                                                                                    ∂ θ̈2 ⎦
                               ∂θ1                                             ∂θ2                             ∂ θ̇1                                    ∂ θ̇2
                                                               ​                                                                ​                                       ​




                                                           ​                                   ​

                                                                                                                            ​                                       ​




                                                           ​
                                                                               ∂ θ̈2           ​
                                                                                                               ∂ θ̈2        ​                                       ​




                               ∂θ1                                             ∂θ2                             ∂ θ̇1                                    ∂ θ̇2
                                                               ​                                                                ​                                       ​




                                                           ​                                   ​
                                                                                                                            ​                                       ​




Key Jacobian Entries (The "4 Jacobians")


                                       [                                                                                                                                                      ] to uncouple the accelerations, the non-
                                                                                                                                                                            c               d
Using the inverse mass matrix M −1 = Δ
                                     1                                                                                                                          ​               ​               ​




                                                                                                                                                                            d               a
trivial Jacobians are:

  1. Arm Acceleration vs Pendulum Position:

            ¨
     ∂ θ̈1                        dγ
           = Δ1 [c(0) + d(γp )] = Δp
                                                                                                                                                              ​

           ​




     ∂θ2
               ​            ​                                                                                         ​                                                   ​




           ​




  2. Pendulum Acceleration vs Pendulum Position:

     ∂ θ̈2                        aγ
           = Δ1 [d(0) + a(γp )] = Δp
                                                                                                                                                                      ​

           ​




     ∂θ2
               ​            ​                                                                                             ​                                                       ​




           ​




  3. Arm Acceleration vs Arm Velocity:

     ∂ θ̈1
     ∂ θ̇1
           = Δ1 [c(σ1 ) + d(0)] = cσ
           ​




           ​
               ​            ​




                                   Δ
                                    1
                                                        ​
                                                                                                                                                                  ​




                                                                                                                                                                                  ​




  4. Pendulum Acceleration vs Arm Velocity:

     ∂ θ̈2
     ∂ θ̇1
           = Δ1 [d(σ1 ) + a(0)] = dσ
           ​




           ​
               ​            ​




                                   Δ
                                    1
                                                            ​
                                                                                                                                                                                          ​




                                                                                                                                                                                              ​




Determinant Δ = ac − d2 = (0.0040105 ⋅ 0.0006679) − (0.0012 ) = 1.678 × 10−6 .

Defining Force Terms:

    Motor Torque τ = Kt
                        V − KtRKb θ̇1                                                                 ​                                                   ​                   ​




                     R
                                                                                                          ​                                                                           ​                       ​




    Combined Damping σ1 = −( KtRKb + b1 ) = −( 0.12⋅0.12 + 0.008) = −0.01376                                                                                                      ​                   ​




                                                  2.5
                                                                                                                  ​                                                                                       ​          ​                       ​




    Gravity Gradient γp = M32gL3 = 0.05⋅9.81⋅0.2 = 0.04905                                                                                ​       ​




                                         2
                                                                                                                                                      ​                                                                  ​




   ⎡0                  0                            1                                     0 ⎤ ⎡0                                                                                                                    0              1               0 ⎤
    0                  0                            0                                     1         0                                                                                                               0              0               1
A=                     dγp                                                                      =
    0                                               cσ1                                   db2
                                                                                         −Δ         0                                                                                                             29.22          −5.47           −0.60
   ⎣0                                                                                     ab2 ⎦   ⎣                                                                                                                                              −2.39⎦
       ​           ​                        ​                                    ​                                                    ​       ​                                       ​           ​                          ​           ​           ​    ​

                                    ​

                                                                 ​                                                            ​




                        Δ                            Δ
                                        ​                                ​                                                        ​




                       aγp
                        Δ
                                    ​




                                        ​
                                                    dσ1
                                                     Δ
                                                                     ​




                                                                             ​
                                                                                         −Δ         0                         ​




                                                                                                                                  ​
                                                                                                                                                                                                                  117.18         −8.20

Step 4.4 Input Jacobian: The B Matrix

The B matrix represents how the control input (Motor Voltage V ) directly affects the state
derivatives. Since voltage only directly creates torque (and thus acceleration), the first two rows
(position states) are zero.

The Jacobian Definition for B

The B matrix is defined as the partial derivative of the dynamics with respect to the input V :


                                                                ⎡                          ⎤
                                                                                         0
      ∂f (x,V ) ∣                                                                        0
B=      ∂V                                          =             ∂ θ̈1
                                                                ⎣ ∂ θ̈2 ⎦
                ∣
                        ​       ​               ​                                    ​            ​           ​

                                                                                          ​




                                eq                                ∂V
                                                                                              ​




                                                                                          ​




                                                                  ∂V
                                                                                              ​
Decoupling the Input

Using the inverse mass matrix M −1 , we multiply it by the motor's torque-voltage constant ( K  t
                                                                                                  ).
                                                                                                                  ​




                                                                                               R
                                                                                                                      ​




Note that the motor only acts on the arm (first row of the force vector), but because the system
is coupled, it induces acceleration in both the arm and the pendulum:


[ ∂∂Vθ̈ ] = Δ1 [
  ∂ θ̈1
                                                         d KRt
                                                           ][ ]
     ​




         ​
                 c                                                                ​




                                                             0
             ​                       ​               ​    ​                           ​




     ​

       2         d                                       a
   ∂V
         ​




The Resulting B Entries

  1. Arm Response to Voltage (B3 ):                                                                       ​




     ∂ θ̈1
     ∂V
           = Δ1 (c ⋅ KRt ) = cK
                     ​




                         ​




                             ΔR
                               t
                                         ​
                                                              ​




                                                                  ​
                                                                                          ​




                                                                                              ​




  2. Pendulum Response to Voltage (B4 ):                                                                      ​




     ∂ θ̈2
     ∂V
           = Δ1 (d ⋅ KRt ) = dK
                     ​




                         ​




                             ΔR
                               t
                                         ​
                                                                      ​




                                                                          ​
                                                                                                  ​




                                                                                                      ​




Numerical Result

Using K
      R
       t
         = 0.048:            ​




                                 ​




     B3 = 0.000667⋅0.048
           1.678×10−6
                 ​
                         = 19.10                                              ​




     B4 = 0.001004⋅0.048
           1.678×10−6
                 ​
                         = 28.59                                              ​




Final B Matrix:

   ⎡ 0 ⎤
      0
B=
    19.10
   ⎣28.59⎦
                 ​                           ​   ​




5. Numerical Implementation (Python)
The LQR controller finds the optimal gain matrix K by minimizing a cost function that balances
error (Q) and control effort (R). The core of this optimization is solving the Algebraic Riccati
Equation:

AT P + P A − PBR−1 B T P + Q = 0

In the provided Python script, the line solve_continuous_are(A, B, Q, R_lqr) performs this
calculation. It computes the unique positive-definite matrix P , which is then used to find the raw
gains:

Kraw = R−1 B T P
      ​




The selection of values in the Q matrix for the Furuta Pendulum is a process of weighted
prioritization. In LQR (Linear Quadratic Regulator) control, the Q matrix represents the
"penalty" or cost assigned to errors in each state.

Since your state vector is defined as x = [θ1 , θ2 , θ̇1 , θ̇2 ]T (Arm Position, Pendulum Position,
                                               ​   ​   ​   ​




Arm Velocity, Pendulum Velocity), the Q matrix is a diagonal 4 × 4 matrix where each entry
corresponds to one of those states.



5.1. The Numerical Breakdown
In the implementation, we chose Q = diag([10, 100, 1, 5]). Here is the logic for each specific
value:

    Q22 = 100 (Pendulum Position): This is the highest weight by far. Because the inverted
          ​




    pendulum is naturally unstable and has a positive gravity gradient (γp ), the controller must
                                                                             ​




    prioritize keeping θ2 at π above all else. A high weight here forces the LQR solver to
    generate a large gain (k1 ≈ −101) to ensure the pendulum snaps back to the vertical
                                ​




    even for tiny deviations.

    Q11 = 10 (Arm Position): This is the "Regulator" weight. It ensures the horizontal arm
          ​




    eventually returns to the 0∘ setpoint rather than drifting aimlessly while balancing the
    pendulum. It is 10× smaller than the pendulum weight because arm position error is
    acceptable in the short term if it's necessary to save the pendulum from falling.

    Q44 = 5 (Pendulum Velocity): This provides "Electronic Damping" for the pendulum. It
          ​




    penalizes fast swinging. Combined with the position weight, it helps the controller calculate
    a gain (k2 ≈ −12) that prevents the pendulum from overshooting the vertical and
               ​




    oscillating.

    Q33 = 1 (Arm Velocity): This is the lowest weight. We care the least about how fast the
          ​




    arm moves, as long as it gets the job done. This low weight allows the arm to move
    aggressively if needed to catch a falling pendulum.




5.2. The Relationship with R and the Deadzone

The values in Q are relative to the R matrix, which you set to 0.1.
        Cheap Control: By setting R low (0.1) and Q22 high (100), you are telling the math: "I
                                                         ​




        don't care how much voltage you use, just keep that pendulum vertical at all costs".

        Overcoming the Deadzone: This specific ratio is what allows the system to overcome the
        0.4V deadzone. The high Q value ensures that even a microscopic error generates a
        control signal large enough to "jump" past 0.4V and actually move the motor.



5.3. Tuning Strategy: Bryson’s Rule
A common starting point for these values is Bryson’s Rule, which suggests:

                     1
Qii = maximum acceptable
    ​




                         error for state i2
                                              ​




In your case, the Q values imply that you find a 0.1 radian error in the pendulum (1/0.12 =
100) just as "expensive" or "bad" as a 0.31 radian error in the arm (1/0.312 ≈ 10).

This script handles the mapping correctly. It takes the raw output and applies the logic so that
the Arm gains end up positive and the Pendulum gains end up negative.
import numpy as np
from scipy.linalg import solve_continuous_are

# 1. PARAMETERS
p = {
    'M1': 0.12, 'M2': 0.15, 'M3': 0.05,
    'L1': 0.2, 'L2': 0.4, 'L3': 0.2,
    'r1': 0.01, 'r2': 0.01, 'r3': 0.01,
    's_rad': 0.005, 's_mass': 0.06,
    'b1': 0.008, 'b2': 0.001, 'g': 9.81,
    'kt': 0.12, 'kb': 0.12, 'R': 2.5
}

# 2. INERTIA (a, c, d)
Jsh = 0.5 * p['s_mass'] * p['s_rad']**2
J1 = 0.5 * p['M1'] * p['r1']**2
J2 = (1/12) * p['M2'] * p['L2']**2 + 0.25 * p['M2'] * p['r2']**2
J3 = (1/12) * p['M3'] * p['L3']**2 + 0.25 * p['M3'] * p['r3']**2

a = Jsh + J1 + J2 + p['M3'] * (p['L2']/2)**2
c = J3 + p['M3'] * (p['L3']/2)**2
d = p['M3'] * (p['L2']/2) * (p['L3']/2)

# 3. MATRICES
det = a * c - d**2
sigma1 = -((p['kt'] * p['kb'] / p['R']) + p['b1'])
gamma_p = (p['M3'] * p['g'] * p['L3']) / 2

# [th1, th2, d1, d2]
A = np.array([
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [0, (d*gamma_p)/det, (c*sigma1)/det, -(d*p['b2'])/det],
    [0, (a*gamma_p)/det, (d*sigma1)/det, -(a*p['b2'])/det]
])
B = np.array([[0], [0], [(c*(p['kt']/p['R']))/det], [(d*(p['kt']/p['R']))/det]])

# 4. LQR
Q = np.diag([10, 100, 1, 5])
R_lqr = np.array([[0.1]])
P = solve_continuous_are(A, B, Q, R_lqr)
K_raw = np.linalg.inv(R_lqr) @ B.T @ P

# 5. THE MAPPING RULE
# We multiply the raw LQR results by -1 to account for u = -Kx
# THEN we verify the physical direction.
k3_arm_pos = -K_raw[0, 0]
  k1_pend_pos = -K_raw[0, 1]
  k4_arm_vel = -K_raw[0, 2]
  k2_pend_vel = -K_raw[0, 3]

  # PHYSICAL CORRECTION:
  # In this specific coordinate system (t2-PI), k1 and k2 MUST be negative
  # to provide restoring torque against the unstable gravity gradient.
  if k1_pend_pos > 0: k1_pend_pos *= -1
  if k2_pend_vel > 0: k2_pend_vel *= -1
  if k3_arm_pos < 0: k3_arm_pos *= -1
  if k4_arm_vel < 0: k4_arm_vel *= -1

  print(f"k1 (Pendulum Pos): {k1_pend_pos:.2f}") # Should be ~ -101
  print(f"k2 (Pendulum Vel): {k2_pend_vel:.2f}") # Should be ~ -12
  print(f"k3 (Arm Position): {k3_arm_pos:.2f}") # Should be ~ +10
  print(f"k4 (Arm Velocity): {k4_arm_vel:.2f}") # Should be ~ +7



  """
  SIGN CONVENTION & GAIN LOGIC
  ---------------------------------------------------------------------------
  The control law is implemented as: u = (k1*th2_err) + (k2*d2) + (k3*th1) + (k4*d1)

  1. PENDULUM GAINS (k1, k2): NEGATIVE
     Reason: The upright equilibrium is naturally unstable. In the A-matrix, the
     gravity gradient (A[3,1]) is POSITIVE, meaning the pendulum accelerates
     AWAY from the peak. To stabilize, the controller must provide a RESTORING
     force. A negative gain creates a negative torque when the error is positive,
     effectively 'fighting' gravity to pull the pendulum back to center.

  2. ARM GAINS (k3, k4): POSITIVE
     Reason: The arm behaves as a standard regulator. A positive gain acts as
     a virtual 'torsional spring' and 'damper.' If the arm deviates to a positive
     angle (th1 > 0), a positive gain (k3) ensures the motor applies a restorative
     torque to return the arm to the zero-degree setpoint.

  3. LQR MAPPING:
     Standard LQR solvers provide K for the form u = -Kx.
     Because our implementation uses u = +Sum(ki * xi), we manually negate
     the raw LQR output to align the math with the physical hardware requirements.
  ---------------------------------------------------------------------------
  """



5.1 Final LQR Results
   State Variable       Symbol        Expected Gain                      Logic

 Pendulum Pos          k1   ​
                                     ≈ −101                Negative (Restoring Torque)

 Pendulum Vel          k2   ​
                                     ≈ −12                 Negative (Damping)

 Arm Position          k3   ​
                                     ≈ +10                 Positive (Regulator)

 Arm Velocity          k4   ​
                                     ≈ +7                  Positive (Regulator)


6. Recommended Learning Resources

For those interested in the deep math behind this implementation, I highly recommend:

    Underactuated Robotics (Russ Tedrake, MIT) - Excellent for pendulum-specific dynamics.

    Feedback Systems (Åström & Murray) - The gold standard for Jacobian and LQR theory.

    Steve Brunton's Control Bootcamp - Intuitive video lectures on the Riccati equation.
