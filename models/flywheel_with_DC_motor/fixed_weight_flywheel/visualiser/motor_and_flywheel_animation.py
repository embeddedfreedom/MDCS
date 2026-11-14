#!/usr/bin/env python3
# Copyright (C) 2026 Diptopal Basu (embeddedfreedom)
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
Flywheel Controller: 1kHz Compensator / 20kHz Simulation
'''

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import math
import threading
import serial

# --- VISUAL SCALING & DIMENSIONS ---
MotorR, MotorL = 0.4, 1.2
ShaftR, ShaftL = 0.08, 0.4
FlywheelR = 1.5  
FlywheelH = 0.2  
GridY = 0.0                 

# --- GLOBAL STATE ---
theta_flywheel = 0.0
omega_flywheel = 0.0
use_degrees = False  
serial_connected = False

def serial_worker():
    global theta_flywheel, omega_flywheel, serial_connected
    port_path = '/home/dipto/sim_3d' 
    while True:
        try:
            ser = serial.Serial(port_path, 115200, timeout=0.05)
            serial_connected = True
            while True:
                if ser.in_waiting > 500: ser.reset_input_buffer()
                raw_line = ser.readline().decode('utf-8', errors='ignore').strip()
                if raw_line and '<' in raw_line and '>' in raw_line:
                    content = raw_line[raw_line.find("<")+1 : raw_line.find(">")]
                    parts = content.split() 
                    if len(parts) >= 2:
                        theta_flywheel = float(parts[0])
                        omega_flywheel = float(parts[1])
        except Exception:
            serial_connected = False
            pygame.time.wait(2000)

def draw_cyl(r, l, color):
    glColor3f(*color)
    q = gluNewQuadric()
    # Enable automatic normal vector generation so lighting shades the cylinders/disks correctly
    gluQuadricNormals(q, GLU_SMOOTH)
    
    # Curved wall section
    gluCylinder(q, r, r, l, 32, 1)
    # Bottom flat cap disc
    gluDisk(q, 0, r, 32, 1)
    
    # Top flat cap disc
    glPushMatrix()
    glTranslatef(0, 0, l)
    gluDisk(q, 0, r, 32, 1)
    glPopMatrix()

def draw_floor_markers():
    """ Renders high-contrast coordinate axes and native geometric ticks """
    # 1. Main Intersecting Axis Lines (Extended Long)
    glLineWidth(4.0)
    glBegin(GL_LINES)
    # 0 to 180 Line (Z-axis line running front-to-back under shaft) - Deep Blue
    glColor3f(0.1, 0.4, 0.9)
    glVertex3f(0.0, GridY + 0.005, -6.0); glVertex3f(0.0, GridY + 0.005, 6.0)
    # -90 to +90 Line (X-axis line running left-to-right under shaft) - Deep Red
    glColor3f(0.9, 0.1, 0.1)
    glVertex3f(-6.0, GridY + 0.005, 0.0); glVertex3f(6.0, GridY + 0.005, 0.0)
    glEnd()

    # 2. Native 3D Geometric Ticks (Marking Key Poles)
    glLineWidth(3.0)
    glBegin(GL_LINES)
    
    # 0° Marker (DOWN / Positive Z direction) - Double cross-bar symbol
    glColor3f(0.0, 0.7, 0.0)
    glVertex3f(-0.3, GridY + 0.01, 5.5); glVertex3f(0.3, GridY + 0.01, 5.5)
    glVertex3f(-0.15, GridY + 0.01, 5.7); glVertex3f(0.15, GridY + 0.01, 5.7)
    
    # ±180° Marker (UP / Negative Z direction) - Single cross-bar
    glColor3f(0.0, 0.0, 0.0)
    glVertex3f(-0.4, GridY + 0.01, -5.5); glVertex3f(0.4, GridY + 0.01, -5.5)
    
    # +90° Marker (RIGHT / Positive X direction) - 'T' shaped tick
    glColor3f(0.9, 0.1, 0.1)
    glVertex3f(5.5, GridY + 0.01, -0.2); glVertex3f(5.5, GridY + 0.01, 0.2)
    
    # -90° Marker (LEFT / Negative X direction) - Bracket tick
    glColor3f(0.1, 0.4, 0.9)
    glVertex3f(-5.5, GridY + 0.01, -0.2); glVertex3f(-5.3, GridY + 0.01, 0.0)
    glVertex3f(-5.5, GridY + 0.01, 0.2); glVertex3f(-5.3, GridY + 0.01, 0.0)
    
    glEnd()
    glLineWidth(1.0)

def draw_ui_overlay(display):
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity()
    glOrtho(0, display[0], 0, display[1], -1, 1)
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity()
    glDisable(GL_DEPTH_TEST)
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    header_height = 110
    glColor4f(0.0, 0.0, 0.0, 0.7)
    glBegin(GL_QUADS)
    glVertex2f(0, display[1] - header_height)
    glVertex2f(display[0], display[1] - header_height)
    glVertex2f(display[0], display[1])
    glVertex2f(0, display[1])
    glEnd()

    font = pygame.font.SysFont('Consolas', 22, bold=True)
    u_ang = "deg" if use_degrees else "rad"
    u_vel = "deg/s" if use_degrees else "rad/s"
    
    p_val = math.degrees(theta_flywheel) if use_degrees else theta_flywheel
    v_val = math.degrees(omega_flywheel) if use_degrees else omega_flywheel
    
    stat_txt = "CONNECTED" if serial_connected else "PORT NOT OPEN"
    stat_col = (0, 255, 0) if serial_connected else (255, 50, 50)
    
    surf_p = font.render(f"FLYWHEEL ANGLE: {p_val:.4f} {u_ang} | VELOCITY: {v_val:.4f} {u_vel}", True, (0, 255, 150))
    surf_s = font.render(f"SERIAL: {stat_txt}", True, stat_col)
    surf_k = font.render("FLOOR KEY: Double Green Line = 0° (DOWN) | Black Line = ±180° (UP)", True, (220, 220, 220))

    glWindowPos2d(30, display[1] - 30)
    glDrawPixels(surf_p.get_width(), surf_p.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, pygame.image.tostring(surf_p, "RGBA", True))
    
    glWindowPos2d(30, display[1] - 60)
    glDrawPixels(surf_s.get_width(), surf_s.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, pygame.image.tostring(surf_s, "RGBA", True))

    glWindowPos2d(30, display[1] - 90)
    glDrawPixels(surf_k.get_width(), surf_k.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, pygame.image.tostring(surf_k, "RGBA", True))

    glDisable(GL_BLEND); glEnable(GL_DEPTH_TEST); glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix(); glMatrixMode(GL_MODELVIEW)

def main():
    global use_degrees
    pygame.init()
    display = [1200, 800]
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL | RESIZABLE)
    pygame.display.set_caption("DC Motor with Flywheel Visualizer")
    
    glClearColor(0.85, 0.85, 0.88, 1.0)
    glEnable(GL_DEPTH_TEST)
    glShadeModel(GL_SMOOTH)
    
    # --- GLOBAL LIGHTING SETUP ---
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glEnable(GL_COLOR_MATERIAL) # Direct glColor3f calls configure ambient/diffuse material tracking
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
    
    # Define metallic specular sheen properties
    glMaterialfv(GL_FRONT, GL_SPECULAR, [0.6, 0.6, 0.6, 1.0])
    glMaterialf(GL_FRONT, GL_SHININESS, 30.0)
    
    # Set up light component attributes
    glLightfv(GL_LIGHT0, GL_AMBIENT, [0.3, 0.3, 0.3, 1.0])
    glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.7, 0.7, 0.7, 1.0])
    glLightfv(GL_LIGHT0, GL_SPECULAR, [0.8, 0.8, 0.8, 1.0])
    
    view_rot_x, view_rot_y, zoom = 25, 45, -12
    threading.Thread(target=serial_worker, daemon=True).start()
    clock = pygame.time.Clock()

    while True:
        for event in pygame.event.get():
            if event.type == QUIT: 
                pygame.quit()
                return
            if event.type == VIDEORESIZE:
                display[0], display[1] = event.w, event.h
                glViewport(0, 0, display[0], display[1])
            if event.type == KEYDOWN and event.key == K_t: 
                use_degrees = not use_degrees
            
            if event.type == MOUSEMOTION:
                mouse_buttons = pygame.mouse.get_pressed()
                if mouse_buttons[0]:
                    view_rot_y += event.rel[0] * 0.5
                    view_rot_x += event.rel[1] * 0.5
                if mouse_buttons[2]:
                    zoom += event.rel[1] * 0.1

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_PROJECTION); glLoadIdentity()
        gluPerspective(45, (display[0]/display[1]), 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW); glLoadIdentity()
        glTranslatef(0, -1, zoom); glRotatef(view_rot_x, 1, 0, 0); glRotatef(view_rot_y, 0, 1, 0)

        # Update directional light coordinates in world-space (coming from top-front-right)
        glLightfv(GL_LIGHT0, GL_POSITION, [5.0, 10.0, 5.0, 0.0])

        # Disable shading for floor lines and geometric marks
        glDisable(GL_LIGHTING)

        # Draw Floor Grid
        glBegin(GL_LINES); glColor3f(0.7, 0.7, 0.7)
        for i in range(-15, 16):
            glVertex3f(i, GridY, -15); glVertex3f(i, GridY, 15)
            glVertex3f(-15, GridY, i); glVertex3f(15, GridY, i)
        glEnd()

        # Render geometric reference lines and ticks directly onto the floor plane
        draw_floor_markers()

        # Enable shading for the 3D actuator assembly structures
        glEnable(GL_LIGHTING)

        # Vertical Assembly Layout
        glPushMatrix()
        glRotatef(-90, 1, 0, 0)
        
        # Motor Base Body
        draw_cyl(MotorR, MotorL, (0.1, 0.1, 0.15)) 
        
        # Drive Shaft
        glTranslatef(0, 0, MotorL)
        draw_cyl(ShaftR, ShaftL, (0.7, 0.7, 0.7)) 
        
        # Metal Red Flywheel (Shaded surface curves and flat disk caps)
        glTranslatef(0, 0, ShaftL)
        glRotatef(math.degrees(theta_flywheel), 0, 0, 1) 
        draw_cyl(FlywheelR, FlywheelH, (0.75, 0.0, 0.0)) 
        
        # Disable shading for the telemetry target vector line
        glDisable(GL_LIGHTING)

        # Yellow Alignment Tracking Arrow (Configured down towards 0° cross bars)
        glColor3f(1.0, 1.0, 0.0)
        glLineWidth(4.0)
        glBegin(GL_LINES)
        glVertex3f(0.0, 0.0, FlywheelH + 0.01)
        glVertex3f(0.0, -FlywheelR * 0.9, FlywheelH + 0.01)
        glVertex3f(0.0, -FlywheelR * 0.9, FlywheelH + 0.01)
        glVertex3f(-0.1, -FlywheelR * 0.75, FlywheelH + 0.01)
        glVertex3f(0.0, -FlywheelR * 0.9, FlywheelH + 0.01)
        glVertex3f(0.1, -FlywheelR * 0.75, FlywheelH + 0.01)
        glEnd()
        
        glPopMatrix()

        draw_ui_overlay(display)
        pygame.display.flip()
        clock.tick(60)

if __name__ == "__main__":
    main()
