'''
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
Pendulum Controller: 1kHz Compensator / 20kHz Simulation
'''

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import math
import threading
import serial

# --- PHYSICAL PARAMETERS ---
Lshaft, Rshaft = 1.0, 0.15
L1, R1 = 2.0, 0.1
L2, R2 = 4.0, 0.1
L3, R3 = 2.3, 0.08
BaseS, BaseH = 2.0, 0.6

# --- GLOBAL STATE ---
theta_arm, theta_pend = 0.0, 0.0
use_degrees = False  
serial_connected = False

def serial_worker():
    """ Strictly handles the <val\tval...> format with buffer clearing. """
    global theta_arm, theta_pend, serial_connected
    port_path = '/home/dipto/sim_3d' 
    while True:
        try:
            ser = serial.Serial(port_path, 115200, timeout=0.05)
            serial_connected = True
            while True:
                if ser.in_waiting > 500:
                    ser.reset_input_buffer()
                    
                raw_line = ser.readline().decode('utf-8', errors='ignore').strip()
                if raw_line and '<' in raw_line and '>' in raw_line:
                    content = raw_line[raw_line.find("<")+1 : raw_line.find(">")]
                    parts = content.split() 
                    if len(parts) >= 2:
                        try:
                            theta_arm = float(parts[0])
                            theta_pend = float(parts[1])
                        except ValueError:
                            continue
        except Exception:
            serial_connected = False
            pygame.time.wait(2000)

def draw_cyl(r, l, color):
    glColor3f(*color)
    q = gluNewQuadric()
    gluCylinder(q, r, r, l, 32, 1) # Tube
    gluDisk(q, 0, r, 32, 1) # Bottom Cap
    glPushMatrix()
    glTranslatef(0, 0, l)
    gluDisk(q, 0, r, 32, 1) # Top Cap
    glPopMatrix()

def draw_motor_base():
    s, h = BaseS/2, BaseH
    glBegin(GL_QUADS)
    glColor3f(0.4, 0.4, 0.45); glVertex3f(-s, h, -s); glVertex3f(s, h, -s); glVertex3f(s, h, s); glVertex3f(-s, h, s)
    glColor3f(0.1, 0.1, 0.1); glVertex3f(-s, 0, -s); glVertex3f(s, 0, -s); glVertex3f(s, 0, s); glVertex3f(-s, 0, s)
    glColor3f(0.2, 0.2, 0.25)
    glVertex3f(-s, 0, s); glVertex3f(s, 0, s); glVertex3f(s, h, s); glVertex3f(-s, h, s)
    glVertex3f(-s, 0, -s); glVertex3f(-s, h, -s); glVertex3f(s, h, -s); glVertex3f(s, 0, -s)
    glVertex3f(-s, 0, -s); glVertex3f(-s, 0, s); glVertex3f(-s, h, s); glVertex3f(-s, h, -s)
    glVertex3f(s, 0, -s); glVertex3f(s, h, -s); glVertex3f(s, h, s); glVertex3f(s, 0, s)
    glEnd()

def draw_ui_overlay(display):
    """ Renders the green text HUD with black background bar """
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity()
    glOrtho(0, display[0], 0, display[1], -1, 1)
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity()
    glDisable(GL_DEPTH_TEST)
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    # Black background bar anchored to top
    header_height = 100
    glColor4f(0.0, 0.0, 0.0, 0.7)
    glBegin(GL_QUADS)
    glVertex2f(0, display[1] - header_height)
    glVertex2f(display[0], display[1] - header_height)
    glVertex2f(display[0], display[1])
    glVertex2f(0, display[1])
    glEnd()

    font = pygame.font.SysFont('Consolas', 22, bold=True)
    u = "deg" if use_degrees else "rad"
    a_val = math.degrees(theta_arm) if use_degrees else theta_arm
    p_val = math.degrees(theta_pend) if use_degrees else theta_pend
    
    stat_txt = "CONNECTED" if serial_connected else "PORT NOT OPEN"
    stat_col = (0, 255, 0) if serial_connected else (255, 50, 50)
    
    surf_t = font.render(f"PENDULUM: {p_val:.4f} {u} | ARM: {a_val:.4f} {u} ", True, (0, 255, 150))
    surf_s = font.render(f"SERIAL: {stat_txt}", True, stat_col)

    # Position text relative to window top
    glWindowPos2d(30, display[1] - 40)
    glDrawPixels(surf_t.get_width(), surf_t.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, pygame.image.tostring(surf_t, "RGBA", True))
    
    glWindowPos2d(30, display[1] - 75)
    glDrawPixels(surf_s.get_width(), surf_s.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, pygame.image.tostring(surf_s, "RGBA", True))

    glDisable(GL_BLEND); glEnable(GL_DEPTH_TEST); glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix(); glMatrixMode(GL_MODELVIEW)

def main():
    global use_degrees
    pygame.init()
    display = [1200, 800]
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL | RESIZABLE)
    pygame.display.set_caption("Furuta Pendulum Animation")
    glClearColor(0.82, 0.82, 0.85, 1.0) 
    glEnable(GL_DEPTH_TEST)
    
    view_rot_x, view_rot_y, zoom = 20, 45, -18
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
            if event.type == KEYDOWN and event.key == K_t:
                use_degrees = not use_degrees
            
            if event.type == MOUSEBUTTONDOWN: mouse_down = True; last_mouse_pos = event.pos
            if event.type == MOUSEBUTTONUP: mouse_down = False
            if event.type == MOUSEMOTION and mouse_down:
                dx, dy = event.pos[0] - last_mouse_pos[0], event.pos[1] - last_mouse_pos[1]
                if pygame.mouse.get_pressed()[0]: view_rot_y += dx * 0.5; view_rot_x += dy * 0.5
                if pygame.mouse.get_pressed()[2]: zoom += dy * 0.1
                last_mouse_pos = event.pos

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_PROJECTION); glLoadIdentity()
        gluPerspective(45, (display[0]/display[1]), 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW); glLoadIdentity()
        glTranslatef(0, -3, zoom); glRotatef(view_rot_x, 1, 0, 0); glRotatef(view_rot_y, 0, 1, 0)

        # Draw Grid
        glBegin(GL_LINES); glColor3f(0.6, 0.6, 0.6)
        for i in range(-15, 16):
            glVertex3f(i, 0, -15); glVertex3f(i, 0, 15)
            glVertex3f(-15, 0, i); glVertex3f(15, 0, i)
        glEnd()

        draw_motor_base()

        # ASSEMBLY - UNTOUCHED
        glPushMatrix()
        glRotatef(math.degrees(-theta_arm), 0, 1, 0) 

        # Vertical
        glPushMatrix(); glTranslatef(0, 0.01, 0); glRotatef(-90, 1, 0, 0)
        draw_cyl(Rshaft, Lshaft, (0.1, 0.1, 0.1)); glTranslatef(0, 0, Lshaft)
        draw_cyl(R1, L1, (0.3, 0.3, 0.3)); glPopMatrix()

        # Horizontal Arm
        glPushMatrix(); glTranslatef(0, Lshaft + L1, -L2/2); draw_cyl(R2, L2, (0.0, 0.3, 0.7)); glPopMatrix()

        # Pendulum
        glPushMatrix(); glTranslatef(0, Lshaft + L1, L2/2)
        glRotatef(math.degrees(theta_pend), 0, 0, 1); glRotatef(90, 1, 0, 0); draw_cyl(R3, L3, (0.7, 0.0, 0.0)); glPopMatrix()
        glPopMatrix()

        # UI OVERLAY
        draw_ui_overlay(display)
        
        pygame.display.flip()
        clock.tick(60)

if __name__ == "__main__":
    main()
