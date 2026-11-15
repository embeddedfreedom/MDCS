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
Copyright (C) 2026 Diptopal Basu (embeddedfreedom)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

Furnace Controller: 1kHz Compensator / 20kHz Simulation
'''

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import math
import threading
import serial
import random

# --- PHYSICAL PARAMETERS ---
SLAB_L = 3.0   # Length of each slab
SLAB_W = 1.5   # Width
SLAB_H = 1.0   # Height
BASE_L = 6.6   # Insulated base length
BASE_W = 1.8   # Insulated base width
BASE_H = 0.3   # Insulated base height

# --- GLOBAL STATE ---
T1 = 25.0
T2 = 25.0
u1 = 0.0
u2 = 0.0
q12 = 0.0
Tamb = 25.0
serial_connected = False

# Particle system for thermal convection plumes
particles = []

def temp_to_color(temp):
    """
    Interpolates temperature to RGB:
    Ambient (25C) -> Cyan/Blue (0.0, 0.5, 1.0)
    Warm (45C) -> Yellow (1.0, 1.0, 0.0)
    Hot (80C+) -> Red/White (1.0, 0.2, 0.2) -> (1.0, 1.0, 1.0)
    """
    t = max(25.0, min(100.0, temp))
    if t <= 45.0:
        factor = (t - 25.0) / 20.0
        r = 0.0 + factor * 1.0
        g = 0.5 + factor * 0.5
        b = 1.0 - factor * 1.0
    else:
        factor = min(1.0, (t - 45.0) / 55.0)
        r = 1.0
        g = 1.0 - factor * 0.7
        b = 0.0 + factor * 0.5
    return (r, g, b)

def serial_worker():
    """ Handles reading streaming thermal telemetry from named pipe / serial port. """
    global T1, T2, u1, u2, q12, Tamb, serial_connected
    port_path = '/home/dipto/sim_3d'
    while True:
        try:
            ser = serial.Serial(port_path, 115200, timeout=0.05)
            serial_connected = True
            buffer = ""
            while True:
                if ser.in_waiting > 500:
                    ser.reset_input_buffer()
                raw = ser.read(ser.in_waiting).decode('utf-8', 'ignore')
                buffer += raw
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    clean = line.strip()
                    if '<' in clean and '>' in clean:
                        content = clean[clean.find("<")+1 : clean.find(">")]
                        parts = content.split()
                        if len(parts) >= 6:
                            try:
                                T1 = float(parts[0])
                                T2 = float(parts[1])
                                u1 = float(parts[2])
                                u2 = float(parts[3])
                                q12 = float(parts[4])
                                Tamb = float(parts[5])
                            except ValueError:
                                continue
        except Exception:
            serial_connected = False
            pygame.time.wait(2000)

def draw_box_colored(dx, dy, dz, c_left, c_right):
    """ Draws a 3D box fully covered on all 6 sides with gradient vertex colors across X """
    hx, hy, hz = dx/2, dy/2, dz/2
    glBegin(GL_QUADS)
    # Front face
    glColor3f(*c_left);  glVertex3f(-hx, -hy,  hz)
    glColor3f(*c_right); glVertex3f( hx, -hy,  hz)
    glColor3f(*c_right); glVertex3f( hx,  hy,  hz)
    glColor3f(*c_left);  glVertex3f(-hx,  hy,  hz)
    # Back face
    glColor3f(*c_right); glVertex3f(-hx, -hy, -hz)
    glColor3f(*c_left);  glVertex3f( hx, -hy, -hz)
    glColor3f(*c_left);  glVertex3f( hx,  hy, -hz)
    glColor3f(*c_right); glVertex3f(-hx,  hy, -hz)
    # Top face
    glColor3f(*c_left);  glVertex3f(-hx,  hy, -hz)
    glColor3f(*c_left);  glVertex3f(-hx,  hy,  hz)
    glColor3f(*c_right); glVertex3f( hx,  hy,  hz)
    glColor3f(*c_right); glVertex3f( hx,  hy, -hz)
    # Bottom face
    glColor3f(*c_left);  glVertex3f(-hx, -hy, -hz)
    glColor3f(*c_right); glVertex3f( hx, -hy, -hz)
    glColor3f(*c_right); glVertex3f( hx, -hy,  hz)
    glColor3f(*c_left);  glVertex3f(-hx, -hy,  hz)
    # Left face
    glColor3f(*c_left);  glVertex3f(-hx, -hy, -hz)
    glColor3f(*c_left);  glVertex3f(-hx, -hy,  hz)
    glColor3f(*c_left);  glVertex3f(-hx,  hy,  hz)
    glColor3f(*c_left);  glVertex3f(-hx,  hy, -hz)
    # Right face
    glColor3f(*c_right); glVertex3f( hx, -hy, -hz)
    glColor3f(*c_right); glVertex3f( hx,  hy, -hz)
    glColor3f(*c_right); glVertex3f( hx,  hy,  hz)
    glColor3f(*c_right); glVertex3f( hx, -hy,  hz)
    glEnd()

def draw_solid_box(dx, dy, dz, color):
    """ Draws a solid colored box covering all 6 sides """
    hx, hy, hz = dx/2, dy/2, dz/2
    glColor3f(*color)
    glBegin(GL_QUADS)
    # Front
    glVertex3f(-hx, -hy, hz); glVertex3f(hx, -hy, hz); glVertex3f(hx, hy, hz); glVertex3f(-hx, hy, hz)
    # Back
    glVertex3f(-hx, -hy, -hz); glVertex3f(-hx, hy, -hz); glVertex3f(hx, hy, -hz); glVertex3f(hx, -hy, -hz)
    # Top
    glVertex3f(-hx, hy, -hz); glVertex3f(-hx, hy, hz); glVertex3f(hx, hy, hz); glVertex3f(hx, hy, -hz)
    # Bottom
    glVertex3f(-hx, -hy, -hz); glVertex3f(hx, -hy, -hz); glVertex3f(hx, -hy, hz); glVertex3f(-hx, -hy, hz)
    # Left
    glVertex3f(-hx, -hy, -hz); glVertex3f(-hx, -hy, hz); glVertex3f(-hx, hy, hz); glVertex3f(-hx, hy, -hz)
    # Right
    glVertex3f(hx, -hy, -hz); glVertex3f(hx, hy, -hz); glVertex3f(hx, hy, hz); glVertex3f(hx, -hy, hz)
    glEnd()

def draw_cylinder(r, l, color):
    glColor3f(*color)
    q = gluNewQuadric()
    gluCylinder(q, r, r, l, 32, 1)
    gluDisk(q, 0, r, 32, 1)
    glPushMatrix()
    glTranslatef(0, 0, l)
    gluDisk(q, 0, r, 32, 1)
    glPopMatrix()

def draw_arrow(direction, magnitude):
    """ Draws a glowing 3D heat-flux arrow over the central thermal bridge """
    length = min(1.5, max(0.2, abs(magnitude) * 0.15))
    opacity = min(1.0, max(0.3, abs(magnitude) * 0.2))
    
    glPushMatrix()
    if direction < 0:
        glRotatef(180, 0, 1, 0)
    
    glColor4f(1.0, 0.6, 0.0, opacity)
    q = gluNewQuadric()
    gluCylinder(q, 0.06, 0.06, length, 16, 1)
    glPushMatrix()
    glTranslatef(0, 0, length)
    gluCylinder(q, 0.15, 0.0, 0.3, 16, 1)
    glPopMatrix()
    glPopMatrix()

def update_particles():
    global particles
    if T1 > Tamb + 10 or T2 > Tamb + 10:
        if len(particles) < 40 and random.random() < 0.4:
            x_pos = random.uniform(-2.5, -0.5) if random.random() < 0.5 else random.uniform(0.5, 2.5)
            particles.append({'x': x_pos, 'y': SLAB_H/2, 'z': random.uniform(-0.5, 0.5), 'vy': random.uniform(0.02, 0.05), 'alpha': 0.8})
    
    new_particles = []
    for p in particles:
        p['y'] += p['vy']
        p['alpha'] -= 0.015
        if p['alpha'] > 0:
            new_particles.append(p)
    particles = new_particles

def draw_particles():
    if not particles:
        return
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glPointSize(6.0)
    glBegin(GL_POINTS)
    for p in particles:
        glColor4f(1.0, 0.8, 0.5, p['alpha'])
        glVertex3f(p['x'], p['y'], p['z'])
    glEnd()
    glDisable(GL_BLEND)

def draw_ui_overlay(display):
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity()
    glOrtho(0, display[0], 0, display[1], -1, 1)
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity()
    glDisable(GL_DEPTH_TEST)
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    header_height = 110
    glColor4f(0.0, 0.0, 0.0, 0.75)
    glBegin(GL_QUADS)
    glVertex2f(0, display[1] - header_height)
    glVertex2f(display[0], display[1] - header_height)
    glVertex2f(display[0], display[1])
    glVertex2f(0, display[1])
    glEnd()

    font = pygame.font.SysFont('Consolas', 20, bold=True)
    stat_txt = "CONNECTED" if serial_connected else "SIMULATION OFFLINE"
    stat_col = (0, 255, 0) if serial_connected else (255, 50, 50)

    surf_t1 = font.render(f"ZONE 1 (T1): {T1:.2f}°C | u1: {u1*100:.1f}%", True, (255, 100, 100))
    surf_t2 = font.render(f"ZONE 2 (T2): {T2:.2f}°C | u2: {u2*100:.1f}%", True, (100, 150, 255))
    surf_flux = font.render(f"HEAT FLUX (Q12): {q12:.3f} W | Tamb: {Tamb:.1f}°C", True, (255, 255, 0))
    surf_s = font.render(f"STATUS: {stat_txt}", True, stat_col)

    glWindowPos2d(30, display[1] - 30)
    glDrawPixels(surf_t1.get_width(), surf_t1.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, pygame.image.tostring(surf_t1, "RGBA", True))
    glWindowPos2d(30, display[1] - 55)
    glDrawPixels(surf_t2.get_width(), surf_t2.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, pygame.image.tostring(surf_t2, "RGBA", True))
    glWindowPos2d(30, display[1] - 80)
    glDrawPixels(surf_flux.get_width(), surf_flux.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, pygame.image.tostring(surf_flux, "RGBA", True))
    glWindowPos2d(450, display[1] - 30)
    glDrawPixels(surf_s.get_width(), surf_s.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, pygame.image.tostring(surf_s, "RGBA", True))

    glDisable(GL_BLEND); glEnable(GL_DEPTH_TEST); glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix(); glMatrixMode(GL_MODELVIEW)

def main():
    pygame.init()
    display = [1200, 800]
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL | RESIZABLE)
    pygame.display.set_caption("Dual-Zone Coupled Heat Slab 3D Visualizer")
    glClearColor(0.0, 0.0, 0.0, 1.0)
    glEnable(GL_DEPTH_TEST)

    view_rot_x, view_rot_y, zoom = 25, 30, -12
    mouse_down = False
    last_mouse_pos = (0, 0)

    threading.Thread(target=serial_worker, daemon=True).start()
    clock = pygame.time.Clock()

    while True:
        for event in pygame.event.get():
            if event.type == QUIT: pygame.quit(); return
            if event.type == VIDEORESIZE:
                display[0], display[1] = event.w, event.h
                glViewport(0, 0, display[0], display[1])
            if event.type == MOUSEBUTTONDOWN: mouse_down = True; last_mouse_pos = event.pos
            if event.type == MOUSEBUTTONUP: mouse_down = False
            if event.type == MOUSEMOTION and mouse_down:
                dx, dy = event.pos[0] - last_mouse_pos[0], event.pos[1] - last_mouse_pos[1]
                if pygame.mouse.get_pressed()[0]: view_rot_y += dx * 0.5; view_rot_x += dy * 0.5
                if pygame.mouse.get_pressed()[2]: zoom += dy * 0.1
                last_mouse_pos = event.pos

        update_particles()

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_PROJECTION); glLoadIdentity()
        gluPerspective(45, (display[0]/display[1]), 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW); glLoadIdentity()
        glTranslatef(0, -1.0, zoom); glRotatef(view_rot_x, 1, 0, 0); glRotatef(view_rot_y, 0, 1, 0)

        # Draw Grid
        glLineWidth(1.0)
        glBegin(GL_LINES); glColor3f(0.5, 0.5, 0.5)
        for i in range(-15, 16):
            glVertex3f(i, -1.0, -15); glVertex3f(i, -1.0, 15)
            glVertex3f(-15, -1.0, i); glVertex3f(15, -1.0, i)
        glEnd()

        # Insulated Base
        glPushMatrix()
        glTranslatef(0, -0.65, 0)
        draw_solid_box(BASE_L, BASE_H, BASE_W, (0.8, 0.8, 0.8))
        glPopMatrix()

        # Zone 1 and Zone 2 Slabs touching at X = 0
        c_left_1 = temp_to_color(T1 - 5.0)
        c_right_1 = temp_to_color(T1)
        c_left_2 = temp_to_color(T2)
        c_right_2 = temp_to_color(T2 + 5.0)

        # Zone 1 Metal Slab
        glPushMatrix()
        glTranslatef(-1.5, 0, 0)
        draw_box_colored(SLAB_L, SLAB_H, SLAB_W, c_left_1, c_right_1)
        glPopMatrix()

        # Zone 2 Metal Slab
        glPushMatrix()
        glTranslatef(1.5, 0, 0)
        draw_box_colored(SLAB_L, SLAB_H, SLAB_W, c_left_2, c_right_2)
        glPopMatrix()

        # Contact line / separation indicator between the two slabs at X = 0
        glPushMatrix()
        glTranslatef(0, 0, 0)
        glLineWidth(3.0)
        glBegin(GL_LINES)
        glColor3f(0.1, 0.1, 0.1)
        glVertex3f(0.0, -SLAB_H/2 - 0.01, -SLAB_W/2 - 0.01)
        glVertex3f(0.0,  SLAB_H/2 + 0.01, -SLAB_W/2 - 0.01)
        glVertex3f(0.0,  SLAB_H/2 + 0.01,  SLAB_W/2 + 0.01)
        glVertex3f(0.0, -SLAB_H/2 - 0.01,  SLAB_W/2 + 0.01)
        glEnd()
        glPopMatrix()

        # Heater Cartridge 1 (Embedded at left end of Zone 1)
        glPushMatrix()
        glTranslatef(-3.1, 0, 0)
        glRotatef(90, 0, 1, 0)
        draw_cylinder(0.2, 0.2, (1.0, 0.2 + 0.8 * u1, 0.1))
        glPopMatrix()

        # Heater Cartridge 2 (Embedded at right end of Zone 2)
        glPushMatrix()
        glTranslatef(2.9, 0, 0)
        glRotatef(90, 0, 1, 0)
        draw_cylinder(0.2, 0.2, (1.0, 0.2 + 0.8 * u2, 0.1))
        glPopMatrix()

        # Dynamic Heat Flux Arrow over center bridge
        glPushMatrix()
        glTranslatef(0, SLAB_H/2 + 0.3, 0)
        draw_arrow(1 if q12 >= 0 else -1, q12)
        glPopMatrix()

        draw_particles()
        draw_ui_overlay(display)

        pygame.display.flip()
        clock.tick(60)

if __name__ == "__main__":
    main()