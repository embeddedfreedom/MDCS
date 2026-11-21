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


Analytical Derivation and Control of the Inverted Pendulum on a DC
Motor
Subtitle: State-Space Linearization via Jacobian Method and LQR Implementation

Author: Diptopal Basu

Date: February 2026

Version: 1.0

Tags: [Control Theory, Robotics, Inverted Pendulum, LQR, Jacobian Linearization]---


Executive Summary & Configuration
This document provides a rigorous mathematical derivation of the Inverted Pendulum dynamics
—specifically a pendulum mounted directly to a DC motor shaft—using Lagrangian
Mechanics. The derivation accounts for both the mechanical inertia of the system and the
electrical characteristics of the DC motor. It concludes with the linearization of the non-linear
equations into a State-Space model (Ax + Bu) and provides the Python implementation for
LQR gain calculation to achieve upright stability.
0.System Configuration & Coordinate Anchor

These parameters form the basis of the numerical matrices below.

               Parameter                                             Description                         Value

  M1                   ​
                                                       Pendulum Mass                         0.2 kg

  L1           ​
                                                       Pendulum Length                       0.3 m

  Itot             ​
                                                       Combined System Inertia               0.00600575 kg·m²

  b1   ​
                                                       Mechanical Friction                   0.008 Nms/rad

  kt /kb   ​                       ​
                                                       Torque / Back-EMF Constants           0.12

  R                                                    Terminal Resistance                   2.5 Ω

  g                                                    Gravitational Acceleration            9.81 m/s²

  Vdeadzone                                        ​
                                                       Minimum Motor Voltage                 0.4 V

Crucial Derivation Notes for the Student

               The π Offset: Because we linearize around the upright position, the "Pendulum Position"
               variable in our LQR code is always the error term: Δθ2 = (θ2 − π).    ​   ​




               Motor Back-EMF: The parameter σ1 in the A matrix is not just b1 . It is the "Combined
                                                                             ​                 ​




               Damping" that includes both mechanical friction and electrical resistance: σ1 = −(b1 +      ​     ​




               Kt Kb
                R ).
                               ​       ​




                                           ​




               The Deadzone: While the LQR math assumes a perfect linear motor, the code must
               compensate for the 0.4V deadzone before sending the final PWM signal to the motor
               driver.


1. Kinematics and Lagrangian
Justification: We use the Lagrangian approach because it naturally handles the energy of
rotating bodies without needing to decompose every individual force vector.

1. Kinetic Energy (T ):

T = 12 Itot θ̇ 2           ​                   ​




2. Potential Energy (V ):
Using the convention where 0 is down and π is up:

V = M1 g L21 cos(θ) ​
                                    ​




                                        ​




3. The Lagrangian (L = T − V ):

L = 12 Itot θ̇ 2 − M12gL1 cos(θ)
            ​           ​
                                                                            ​               ​




                                                                                                ​




4. The Equation of Motion:

Applying dt (                                   ) − ∂L
                        d                   ∂L
                                            ∂θ̇     ∂θ = τmotor − friction:
                            ​                                       ​                                   ​                                                               ​




Itot θ̈ − M12gL1 sin(θ) = kRt (Vin − kb θ̇ ) − b1 θ̇
                                ​           ​
                                                                                                                                    ​




   ​                                            ​                                                                                       ​       ​                           ​                       ​




2. Jacobian Linearization
Justification: LQR requires a linear system (ẋ = Ax + Bu). Since sin(θ) is non-linear, we
use a Taylor series expansion (Jacobian) around the unstable equilibrium point θ = π .

1. Define the Nonlinear State Functions:

Let x1 = θ and x2 = θ̇ .
        ​                                                               ​




ẋ1 = f1 (x, u) = x2
                ​                                                               ​




                                         1
ẋ2 = f2 (x, u) = M2I1 gL
                ​




                       tot
                           1
                             sin(x1 ) − Itot (b1 + ktRkb )x2 + RIkttot Vin              ​




                                                                                                    ​
                                                                                                            ​




                                                                                                                ​                                   ​




                                                                                                                                                                                ​
                                                                                                                                                                                    ​       ​
                                                                                                                                                                                                        ​           ​




                                                                                                                                                                                                                        ​       ​
                                                                                                                                                                                                                                            ​




                                                                                                                                                                                                                                                    ​
                                                                                                                                                                                                                                                        ​       ​




2. Compute the Jacobians at x0 = [π, 0]T :                                                                                                              ​




       For Matrix A:

             ∂f1                                                        ∂f1
       A = [ ∂f21                                                       ∂x2 ] = [
                                                                                          0                                                                                                                                 1
                                                                                                                                                                                                                                                        ]
                                                    ​




             ∂x
                                                            ​                       ​




                                                                                                                                                                                                        1
                                                        ​                       ​




                                                                        ∂f2       M   gL
                                                                                   2Itot cos(π)
                                                                                    1   1
                                                                                                                                                                                                − Itot (b1 + ktRkb )
                                                                ​                                                                                                                       ​                                                                   ​

                                                                                                                                            ​                   ​
                                                                                                                                                                                                                                    ​   ​
                                                    ​




             ∂x1                                                        ∂x2
                                                                                                                                                                    ​                                           ​           ​                   ​
                                                            ​                       ​


                                                                                                                                                            ​                                               ​
                                                        ​                       ​




       Since cos(π) = −1 and we define the error as Δθ = θ − π , the sign flips to positive for
       the restoring force calculation.

       For Matrix B:

             ∂f1
                       0
       B = [ ∂f
             ∂u ] = [       ]
                                                    ​




                                                            ​




               2       kt                           ​
                                                                ​


                                                                                                                    ​
                                                                                                                                ​




             ∂u       RItot
                                                                                                                            ​
                                                            ​



                                                                                                                        ​




3. Numerical Result:




        [                                                                                                               ]                                                   [                   ]
         0                                     1                     0
A=[                                                 ],      B=[          ]
       49.003                                −2.291                7.992
                             ​                     ​                        ​




3. LQR and the Riccati Equation
Justification: LQR finds the "optimal" balance between system performance and energy usage.
It solves the Algebraic Riccati Equation (ARE) to minimize total cost.

1. Penalty Selection (The Q Matrix):

We prioritize position error over velocity:

       1010                       0
Q=[                                   ],               R = [0.1]
        0                        15.7
                        ​                      ​




2. The Riccati Equation:

AT P + P A − PBR−1 B T P + Q = 0

Solving for P gives the energy surface of the system. The optimal gains are K = R−1 B T P .

3. Theoretical vs. Manual Gains:

The Python solver yields k1 ≈ −106 and k2 ≈ −13. However, your project uses k1 =
                                                        ​               ​            ​




−220.0 and k2 = −26.0. This doubling of "stiffness" is a deliberate engineering choice to
                                 ​




compensate for high hardware friction.


3.1 LQR Cost Weighting and Bryson’s Rule
The LQR algorithm minimizes the cost function J , which balances the importance of state
accuracy against the cost of control effort (voltage):
       ∞
J = ∫0 (xT Qx + uT Ru)dt
            ​




1. Expanding the Q Matrix

For your 2-state pendulum system, the Q matrix is a 2 × 2 diagonal matrix. Each diagonal
element represents the "penalty" for an error in that specific state:

                             0
Q = [ 11                        ]
     q          ​




      0                     q22
                    ​                    ​




                                     ​




    q11 : Penalty for Angular Error (θ − π ). A higher value makes the pendulum stiffer and
        ​




    more vertical.
       q22 : Penalty for Angular Velocity (θ̇ ). A higher value adds more electronic damping to
            ​




       prevent oscillation.

2. Bryson’s Rule for Initial Tuning

Bryson’s Rule suggests that the weights should be the inverse of the square of the maximum
acceptable error for that state:

                  1
qii = max acceptable
   ​




                     value of (x2 )
                                             ​




                                 i
                                     ​




                1
R = max acceptable value of (u2 )


3. Numerical Transition: Ideal vs. Implementation

By comparing the "Ideal" weights (theoretical) to the weights required to reach your
"Implementation" gains, we can see how Bryson's Rule was pushed to handle real-world friction.

                                                 Ideal          Implementation
  Weight        Parameter                                                               Physical Meaning
                                Selection                             Selection

                Position                                                          We tolerate almost zero
  q11                           1010                            4575
                Penalty                                                           angular error.
        ​




                Velocity
  q22                           15.7                            64                High damping to kill jitter.
                Penalty
        ​




                Control                                                           Voltage is "cheap"; use as
  R                             0.1                             0.1
                Cost                                                              much as needed.


4. Integral Justification (ki = −75.0)                  ​




Justification: Why did you add ki ? Because LQR alone is mathematically "blind" to constant
                                                    ​




disturbances.

       The 176∘ Settling Point: Without ki , gravity creates a constant torque. To counteract it,
                                                            ​




       the LQR controller needs a non-zero error to produce voltage. This results in the pendulum
       settling slightly below the top (e.g., at 176∘ ). This is the steady-state error.

       The Integrator's Job: ki sums the error over time. As long as the pendulum is even 0.001
                                         ​




       rad away from π , the sum grows. Eventually, it provides enough voltage to overcome the
       0.4V deadzone and push the pendulum to exactly 180.000∘ .
    Settling without Integrator: Without the integrator, the system settles at the point where
    k1 × Error = Gravity Torque. With your mass, that offset is several degrees.
          ​




6. Final Control Law

Output Voltage(V ) = k1 (θ − π) + k2 (θ̇ ) + ki ∫ (θ − π)dt + Deadzone
                              ​                   ​   ​    ​                   ​




                                  LQR Stability
                                                          Precision Integral


7. Numerical Implementation (Python)
The LQR controller finds the optimal gain matrix K by minimizing a cost function that balances
error (Q) and control effort (R). The core of this optimization is solving the Algebraic Riccati
Equation:

AT P + P A − PBR−1 B T P + Q = 0

In the provided Python script, the line solve_continuous_are(A, B, Q, R_lqr) performs this
calculation. It computes the unique positive-definite matrix P , which is then used to find the raw
gains:

Kraw = R−1 B T P
      ​




This script handles the mapping to the C-code implementation correctly. It takes the raw output
Kraw and applies the negation logic (k = −K ) so that the final Pendulum gains (k1 for
      ​                                                                                  ​




position and k2 for velocity) result in the negative values used in the compensator. This ensures
                ​




that a positive angular error produces a negative corrective voltage to restore the pendulum to
the upright position.
  import numpy as np
  from scipy.linalg import solve_continuous_are

  def compute_lqr():
      # Parameters from Table
      M1, L1, I_tot = 0.2, 0.3, 0.00600575
      g, b1, kt, kb, R = 9.81, 0.008, 0.12, 0.12, 2.5

      # Jacobian Matrices
      A = np.array([[0, 1], [(M1*g*L1)/(2*I_tot), -(b1 + (kt*kb/R))/I_tot]])
      B = np.array([[0], [(kt/R)/I_tot]])

      # LQR Weights
      Q = np.diag([4575, 64])
      R_mat = np.array([[0.1]])

      # Solve Riccati
      P = solve_continuous_are(A, B, Q, R_mat)
      K = np.linalg.inv(R_mat) @ B.T @ P
      print(f"LQR Gains: k1={-K[0,0]:.4f}, k2={-K[0,1]:.4f}")

  compute_lqr()



7.1 Final LQR Results
The table below summarizes the gains derived from the LQR optimization. The "Ideal" values
represent standard mathematical weights, while the "Implementation" values reflect the high-
stiffness tuning required to dominate hardware friction and stiction.

                                       Ideal
   State Variable       Symbol                     Implementation                 Logic
                                       Gain

 Pendulum                                                               Negative (Restoring
                        k1           ≈ −106        −220.0
 Position                                                               Torque)
                             ​




 Pendulum                                                               Negative (Damping
                        k2           ≈ −13         −26.0
 Velocity                                                               Force)
                             ​




8. Limitations and Future Work

The Impact of Noise and Drift

It is important to observe that in a purely theoretical environment—without sensor noise or
mechanical disturbances—the LQR balance is mathematically perfect. Under ideal
conditions, the pendulum would remain vertical indefinitely with near-zero control effort once it
reaches the equilibrium point.

However, the physical implementation reveals two primary challenges:

  1. Sensor Noise and Jitter: The raw encoder data contains high-frequency noise. When we
    derive velocity (θ̇ ) from these readings, the noise is amplified. This forces the motor to react
    to "phantom" movements, creating the micro-vibrations heard during operation.

  2. System Drift: Despite a high k1 gain, the system may experience a slow "drift" over time.
                                      ​




     This is caused by thermal changes in the motor resistance, slight misalignments in the
    center-of-gravity, or internal encoder scaling errors.

The Student Challenge

Because of these real-world constraints, even this high-stiffness derivation does not fully do
justice to the complexity of a perfect, silent balance. The student is encouraged to work on
this further by focusing on:

    Noise Rejection: Implementing a Kalman Filter to optimally estimate the state while
    ignoring sensor noise.

    Drift Compensation: Improving the z -pulse synchronization logic or adding an outer-loop
    to track and zero out long-term drift.

    System Identification: Refining the Itot and b1 parameters through empirical testing to
                                              ​       ​




    closer match the mathematical model to the physical hardware.

Final Note: The implementation of k1 = −220.0 and ki = −75.0 provides a robust "stiff"
                                          ​                  ​




balance that holds the pendulum upright despite noise, but a truly "perfect" balance requires the
student to explore advanced filtering and non-linear modeling.


9. Recommended Learning Resources

For those interested in the deep math behind this implementation, I highly recommend:

    Underactuated Robotics (Russ Tedrake, MIT) - Excellent for pendulum-specific dynamics.

    Feedback Systems (Åström & Murray) - The gold standard for Jacobian and LQR theory.

    Steve Brunton's Control Bootcamp - Intuitive video lectures on the Riccati equation.
