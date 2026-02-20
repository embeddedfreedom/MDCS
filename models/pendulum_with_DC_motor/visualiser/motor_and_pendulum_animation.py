import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import math
import threading
import serial

# --- VISUAL SCALING & DIMENSIONS ---
TableW, TableH, TableD = 8.0, 4.0, 4.0
LegR = 0.15
MotorR, MotorL = 0.4, 1.2
ShaftR, ShaftL = 0.08, 0.5
PendR, PendL = 0.1, 2.5      
GridY = -5.0                 

# --- GLOBAL STATE ---
theta_pend = 0.0
use_degrees = False  
serial_connected = False

def serial_worker():
    global theta_pend, serial_connected
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
                    if len(parts) >= 1:
                        theta_pend = float(parts[0])
        except Exception:
            serial_connected = False
            pygame.time.wait(2000)

def draw_cyl(r, l, color):
    glColor3f(*color)
    q = gluNewQuadric()
    gluCylinder(q, r, r, l, 32, 1)
    gluDisk(q, 0, r, 32, 1)
    glPushMatrix()
    glTranslatef(0, 0, l)
    gluDisk(q, 0, r, 32, 1)
    glPopMatrix()

def draw_lab_table():
    w, h, d = TableW/2, TableH, TableD/2
    thickness = 0.2
    glColor3f(0.6, 0.6, 0.65)
    glBegin(GL_QUADS)
    glVertex3f(-w, 0, -d); glVertex3f(w, 0, -d); glVertex3f(w, 0, d); glVertex3f(-w, 0, d)
    glVertex3f(-w, -thickness, -d); glVertex3f(w, -thickness, -d); glVertex3f(w, -thickness, d); glVertex3f(-w, -thickness, d)
    glColor3f(0.5, 0.5, 0.55)
    glVertex3f(-w, 0, d); glVertex3f(w, 0, d); glVertex3f(w, -thickness, d); glVertex3f(-w, -thickness, d)
    glVertex3f(-w, 0, -d); glVertex3f(w, 0, -d); glVertex3f(w, -thickness, -d); glVertex3f(-w, -thickness, -d)
    glEnd()

    leg_c = (0.2, 0.2, 0.2)
    inset = 0.3
    positions = [(-w+inset, d-inset), (w-inset, d-inset), (-w+inset, -d+inset), (w-inset, -d+inset)]
    for pos in positions:
        glPushMatrix()
        glTranslatef(pos[0], -thickness, pos[1])
        glRotatef(90, 1, 0, 0)
        draw_cyl(LegR, h-thickness, leg_c)
        glPopMatrix()

def draw_ui_overlay(display):
    # Anchor logic: All Y coordinates are relative to display[1] (window top)
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity()
    glOrtho(0, display[0], 0, display[1], -1, 1)
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity()
    glDisable(GL_DEPTH_TEST)
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    # Header Bar Background (Black Overlay)
    header_height = 90
    glColor4f(0.0, 0.0, 0.0, 0.7)
    glBegin(GL_QUADS)
    glVertex2f(0, display[1] - header_height)
    glVertex2f(display[0], display[1] - header_height)
    glVertex2f(display[0], display[1])
    glVertex2f(0, display[1])
    glEnd()

    font = pygame.font.SysFont('Consolas', 24, bold=True)
    u = "deg" if use_degrees else "rad"
    p_val = math.degrees(theta_pend) if use_degrees else theta_pend
    
    stat_txt = "CONNECTED" if serial_connected else "PORT NOT OPEN"
    stat_col = (0, 255, 0) if serial_connected else (255, 50, 50)
    
    surf_p = font.render(f"PENDULUM: {p_val:.4f} {u}", True, (0, 255, 150))
    surf_s = font.render(f"SERIAL: {stat_txt}", True, stat_col)

    # Dynamic Text Placement
    glWindowPos2d(30, display[1] - 35)
    glDrawPixels(surf_p.get_width(), surf_p.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, pygame.image.tostring(surf_p, "RGBA", True))
    
    glWindowPos2d(30, display[1] - 70)
    glDrawPixels(surf_s.get_width(), surf_s.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, pygame.image.tostring(surf_s, "RGBA", True))

    glDisable(GL_BLEND); glEnable(GL_DEPTH_TEST); glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix(); glMatrixMode(GL_MODELVIEW)

def main():
    global use_degrees
    pygame.init()
    display = [1200, 800]
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL | RESIZABLE)
    pygame.display.set_caption("DC Motor with pendulum - Animation")
    
    glClearColor(0.85, 0.85, 0.88, 1.0)
    glEnable(GL_DEPTH_TEST)
    
    view_rot_x, view_rot_y, zoom = 15, 30, -16
    threading.Thread(target=serial_worker, daemon=True).start()
    clock = pygame.time.Clock()

    while True:
        for event in pygame.event.get():
            if event.type == QUIT: 
                pygame.quit()
                return
            
            # Handle Window Resizing
            if event.type == VIDEORESIZE:
                display[0], display[1] = event.w, event.h
                glViewport(0, 0, display[0], display[1])

            if event.type == KEYDOWN and event.key == K_t: 
                use_degrees = not use_degrees
            
            # --- MOUSE LOGIC ---
            if event.type == MOUSEMOTION:
                mouse_buttons = pygame.mouse.get_pressed()
                if mouse_buttons[0]: # Left Click: Rotation
                    view_rot_y += event.rel[0] * 0.5
                    view_rot_x += event.rel[1] * 0.5
                if mouse_buttons[2]: # Right Click: Zoom
                    zoom += event.rel[1] * 0.1

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_PROJECTION); glLoadIdentity()
        gluPerspective(45, (display[0]/display[1]), 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW); glLoadIdentity()
        glTranslatef(0, -1, zoom); glRotatef(view_rot_x, 1, 0, 0); glRotatef(view_rot_y, 0, 1, 0)

        # Draw Grid
        glBegin(GL_LINES); glColor3f(0.7, 0.7, 0.7)
        for i in range(-15, 16):
            glVertex3f(i, GridY, -15); glVertex3f(i, GridY, 15)
            glVertex3f(-15, GridY, i); glVertex3f(15, GridY, i)
        glEnd()

        draw_lab_table()

        # Render Motor and Pendulum Assembly
        glPushMatrix()
        glTranslatef(0, MotorR + 0.05, TableD/2 - 0.4) 
        draw_cyl(MotorR, MotorL, (0.1, 0.1, 0.15)) # Motor Body
        glTranslatef(0, 0, MotorL)
        glRotatef(math.degrees(theta_pend), 0, 0, 1) # Applied Dynamic Rotation
        draw_cyl(ShaftR, ShaftL, (0.7, 0.7, 0.7)) # Shaft
        glTranslatef(0, 0, ShaftL - 0.05)
        glRotatef(90, 1, 0, 0) 
        draw_cyl(PendR, PendL, (0.8, 0.1, 0.1)) # Pendulum Arm
        glPopMatrix()

        draw_ui_overlay(display)
        pygame.display.flip()
        clock.tick(60)

if __name__ == "__main__":
    main()
