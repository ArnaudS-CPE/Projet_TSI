#!/usr/bin/env python3

import OpenGL.GL as GL
import glfw
import pyrr
import numpy as np
from cpe3d import Object3D
import random
from time import sleep


T = 0
Vie = 4
Vitesse = 0.04


valRenz = True
valRens = True
valRenq = True
valRend = True
direct = [0,0]

pointp=[]
def onSegment(p, q, r):
    if ( (q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and
        (q.z <= max(p.z, r.z)) and (q.z >= min(p.z, r.z))):
        return True
    return False
def orientation(p, q, r):  
    val = (float(q.z - p.z) * (r.x - q.x)) - (float(q.x - p.x) * (r.z - q.z))
    if (val > 0):
        return 1
    elif (val < 0):
        return 2
    else:
        return 0
 
def doIntersect(p1,q1,p2,q2):
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)
 
    if ((o1 != o2) and (o3 != o4)):
        return True
    if ((o1 == 0) and onSegment(p1, p2, q1)):
        return True
    if ((o2 == 0) and onSegment(p1, q2, q1)):
        return True
    if ((o3 == 0) and onSegment(p2, p1, q2)):
        return True
    if ((o4 == 0) and onSegment(p2, q1, q2)):
        return True
    return False

class Point:
    def __init__(self, x, z):
        self.x = x
        self.z = z 



class ViewerGL:
    def __init__(self):
        self.collisions=[[],[]]
        # initialisation de la librairie GLFW
        glfw.init()
        # paramétrage du context OpenGL
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, GL.GL_TRUE)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
        # création et paramétrage de la fenêtre
        glfw.window_hint(glfw.RESIZABLE, False)
        self.window = glfw.create_window(800, 800, 'OpenGL', None, None)
        # paramétrage de la fonction de gestion des évènements
        glfw.set_key_callback(self.window, self.key_callback)
        # activation du context OpenGL pour la fenêtre
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)
        # activation de la gestion de la profondeur
        GL.glEnable(GL.GL_DEPTH_TEST)
        # choix de la couleur de fond
        GL.glClearColor(0.5, 0.6, 0.9, 1.0)
        print(f"OpenGL: {GL.glGetString(GL.GL_VERSION).decode('ascii')}")

        self.objs = []
        self.touch = {}








    def run(self):

        global Vitesse

        debut = 0
        compteur = 0
        L = [0, 0]

        vie = 5

        # boucle d'affichage
        while not glfw.window_should_close(self.window):
            # nettoyage de la fenêtre : fond et profondeur
            GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
            
            #self.update_key(debut)

            for obj in self.objs:
                GL.glUseProgram(obj.program)
                if isinstance(obj, Object3D):
                    self.update_camera(obj.program)
                    
                obj.draw()
 


            self.init_ennemi(debut)

            self.update_key(debut)




            debut+=1

            # Déplacement de l'ennemi et des coeurs au dessus de l'ennemi
            timer = glfw.get_time()
            if timer <= (3 + (compteur*12)) :
                self.objs[2].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, Vitesse]))
                self.objs[9].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, Vitesse]))
                self.objs[10].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, Vitesse]))
                self.objs[11].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, Vitesse]))
                self.objs[12].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, Vitesse]))
                c = 1
            elif (timer > 3+(compteur*12)) and (timer <= 6+(compteur*12)) :
                self.objs[2].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, Vitesse]))      
                self.objs[9].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, Vitesse]))
                self.objs[10].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, Vitesse]))
                self.objs[11].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, Vitesse]))
                self.objs[12].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, Vitesse]))
                c = 2
            elif (timer > 6+(compteur*12)) and (timer <= 9+(compteur*12)) :
                self.objs[2].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, Vitesse]))      
                self.objs[9].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, Vitesse]))
                self.objs[10].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, Vitesse]))
                self.objs[11].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, Vitesse]))
                self.objs[12].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, Vitesse]))
                c = 3
            elif (timer > 9+(compteur*12)) and (timer <= 12+(compteur*12)) :
                self.objs[2].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, Vitesse]))      
                self.objs[9].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, Vitesse]))
                self.objs[10].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, Vitesse]))
                self.objs[11].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, Vitesse]))
                self.objs[12].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, Vitesse]))
                c = 4
            else :
                compteur += 1

            # Teste si l'ennemi doit tourner
            L[0] = L[1]
            L[1] = c
            if (L[0] < L[1]) or (L[0] == 4 and L[1] == 1) :
                self.objs[2].transformation.rotation_euler[pyrr.euler.index().yaw] += random.uniform(-np.pi, np.pi)

            # Limite murs
            if (abs(self.objs[2].transformation.translation[0]) > 23) or (abs(self.objs[2].transformation.translation[2]) > 23) :
                self.objs[2].transformation.rotation_euler[pyrr.euler.index().yaw] += np.pi


            # Rotation des coeurs
            self.objs[9].transformation.rotation_euler[pyrr.euler.index().yaw] += 0.03
            self.objs[10].transformation.rotation_euler[pyrr.euler.index().yaw] += 0.03
            self.objs[11].transformation.rotation_euler[pyrr.euler.index().yaw] += 0.03
            self.objs[12].transformation.rotation_euler[pyrr.euler.index().yaw] += 0.03


            # changement de buffer d'affichage pour éviter un effet de scintillement
            glfw.swap_buffers(self.window)
            # gestion des évènements
            glfw.poll_events()
        

    def key_callback(self, win, key, scancode, action, mods):
        # sortie du programme si appui sur la touche 'échappement'
        if key == glfw.KEY_ESCAPE and action == glfw.PRESS:
            glfw.set_window_should_close(win, glfw.TRUE)
        self.touch[key] = action




    def init_ennemi (self, debut) :
        rand_x = random.uniform(-22, 22)
        rand_z = random.uniform(-22, 22)
        rand_rot = random.uniform(-np.pi, np.pi)

        if debut == 0 :
            # Ennemi
            self.objs[2].transformation.translation += \
                pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([rand_x, 0, rand_z]))
            self.objs[2].transformation.rotation_euler[pyrr.euler.index().yaw] += rand_rot
            # Coeurs
            self.objs[9].transformation.translation += \
                pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([rand_x-1.5, 0, rand_z]))
            self.objs[10].transformation.translation += \
                pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([rand_x-0.5, 0, rand_z]))
            self.objs[11].transformation.translation += \
                pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([rand_x+0.5, 0, rand_z]))
            self.objs[12].transformation.translation += \
                pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([rand_x+1.5, 0, rand_z]))
            
        self.objs[8].bottomLeft = np.array([0.6, 0.2], np.float32)



    
    def add_object(self, obj):
        self.objs.append(obj)


    def set_camera(self, cam):
        self.cam = cam
        self.cam.transformation.rotation_euler[pyrr.euler.index().yaw] += np.pi


    def update_camera(self, prog):
        GL.glUseProgram(prog)
        # Récupère l'identifiant de la variable pour le programme courant
        loc = GL.glGetUniformLocation(prog, "translation_view")
        # Vérifie que la variable existe
        if (loc == -1) :
            print("Pas de variable uniforme : translation_view")
        # Modifie la variable pour le programme courant
        translation = -self.cam.transformation.translation
        GL.glUniform4f(loc, translation.x, translation.y, translation.z, 0)

        # Récupère l'identifiant de la variable pour le programme courant
        loc = GL.glGetUniformLocation(prog, "rotation_center_view")
        # Vérifie que la variable existe
        if (loc == -1) :
            print("Pas de variable uniforme : rotation_center_view")
        # Modifie la variable pour le programme courant
        rotation_center = self.cam.transformation.rotation_center
        GL.glUniform4f(loc, rotation_center.x, rotation_center.y, rotation_center.z, 0)

        rot = pyrr.matrix44.create_from_eulers(-self.cam.transformation.rotation_euler)
        loc = GL.glGetUniformLocation(prog, "rotation_view")
        if (loc == -1) :
            print("Pas de variable uniforme : rotation_view")
        GL.glUniformMatrix4fv(loc, 1, GL.GL_FALSE, rot)
    
        loc = GL.glGetUniformLocation(prog, "projection")
        if (loc == -1) :
            print("Pas de variable uniforme : projection")
        GL.glUniformMatrix4fv(loc, 1, GL.GL_FALSE, self.cam.projection)




    def update_key(self, debut):


        global T
        global Vie
        global Vitesse

        global pointp
        global direct
        global valRenz ; global valRens
        global valRenq ; global valRend

        
        def centrer_cam () :
            #self.cam.transformation.rotation_euler = self.objs[0].transformation.rotation_euler.copy() 
            #self.cam.transformation.rotation_euler[pyrr.euler.index().yaw] += np.pi
            self.cam.transformation.rotation_center = self.objs[0].transformation.translation + self.objs[0].transformation.rotation_center
            self.cam.transformation.translation = self.objs[0].transformation.translation + pyrr.Vector3([0, 1, 5])

        pointp = [self.objs[0].transformation.translation.x,self.objs[0].transformation.translation.z]

        centrer_cam()


        if glfw.KEY_W in self.touch and self.touch[glfw.KEY_W] > 0 and valRenz == True:
            self.objs[0].transformation.translation += \
                pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([0, 0, 0.3]))
            valRens = True
            centrer_cam()

        if glfw.KEY_S in self.touch and self.touch[glfw.KEY_S] > 0 and valRens == True: 
            self.objs[0].transformation.translation -= \
                pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([0, 0, 0.3]))
            valRenz = True
            centrer_cam()

        if glfw.KEY_A in self.touch and self.touch[glfw.KEY_A] > 0 and valRenq == True:
            self.objs[0].transformation.translation += \
                pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([0.1, 0, 0]))
            valRend = True
            valRenz = True
            centrer_cam()

        if glfw.KEY_D in self.touch and self.touch[glfw.KEY_D] > 0 and valRend == True: 
            self.objs[0].transformation.translation -= \
                pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([0.1, 0, 0]))
            valRenq = True
            valRenz = True
            centrer_cam()




        if glfw.KEY_UP in self.touch and self.touch[glfw.KEY_UP] > 0:
            if self.cam.transformation.rotation_euler[pyrr.euler.index().roll] > -np.pi/10: #limites du déplacement de la caméra
                self.cam.transformation.rotation_euler[pyrr.euler.index().roll] -= 0.04
                #self.objs[0].transformation.rotation_euler[pyrr.euler.index().roll] -= 0.04

        if glfw.KEY_DOWN in self.touch and self.touch[glfw.KEY_DOWN] > 0:
            if self.cam.transformation.rotation_euler[pyrr.euler.index().roll] < np.pi/3: #limites du déplacement de la caméra
                self.cam.transformation.rotation_euler[pyrr.euler.index().roll] += 0.04
                #self.objs[0].transformation.rotation_euler[pyrr.euler.index().roll] += 0.04

        if glfw.KEY_LEFT in self.touch and self.touch[glfw.KEY_LEFT] > 0:
            self.cam.transformation.rotation_euler[pyrr.euler.index().yaw] -= 0.04
            self.objs[0].transformation.rotation_euler[pyrr.euler.index().yaw] -= 0.04
            #self.objs[6].transformation.rotation_euler[pyrr.euler.index().yaw] -= 0.04

        if glfw.KEY_RIGHT in self.touch and self.touch[glfw.KEY_RIGHT] > 0:
            self.cam.transformation.rotation_euler[pyrr.euler.index().yaw] += 0.04
            self.objs[0].transformation.rotation_euler[pyrr.euler.index().yaw] += 0.04
            #self.objs[6].transformation.rotation_euler[pyrr.euler.index().yaw] += 0.04



        def collision(posX,posZ,collisions):
            global valRenz
            global valRens
            global valRenq
            global valRend
            global direct 

        
            for k in range(0,len(collisions[0])):
                p1 = Point(collisions[0][k][0],collisions[0][k][2])
                q1 = Point(collisions[1][k][0], collisions[1][k][2])
                p2 = Point(pointp[0],pointp[1])
                q2 = Point(posX,posZ)
                if doIntersect(p1, q1, p2, q2):
                    direct = [p2.x -q2.x , p2.z - q2.z]
                    if glfw.KEY_W in self.touch and self.touch[glfw.KEY_W] > 0 and valRenz == True:
                        self.objs[0].transformation.translation += \
                            pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([0, 0,-0.15]))
                        valRenz = False
                    if glfw.KEY_S in self.touch and self.touch[glfw.KEY_S] > 0 and valRens == True: 
                        self.objs[0].transformation.translation -= \
                            pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([0, 0, -0.15]))
                        valRens = False
                    if glfw.KEY_D in self.touch and self.touch[glfw.KEY_D] > 0 and valRend == True: 
                        self.objs[0].transformation.translation += \
                            pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([0.3, 0, 0]))
                        valRend = False
                    if glfw.KEY_A in self.touch and self.touch[glfw.KEY_A] > 0 and valRenq == True:
                        self.objs[0].transformation.translation -= \
                            pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([0.3, 0, 0]))
                        valRenq = False
             
        collision(self.objs[0].transformation.translation.x,self.objs[0].transformation.translation.z,self.collisions)






        # Tir
        if glfw.KEY_E in self.touch and self.touch[glfw.KEY_E] > 0:

            # Fréquence de tir
            freq = True
            if ((glfw.get_time() - T) < 0.2) : # On test la durée entre 2 tirs
                freq = False
            T = glfw.get_time()

            #on calcul les cooordonnées de l'ennemi par rapport au personnage
            nouv_coord = [self.objs[2].transformation.translation[0] - self.objs[0].transformation.translation[0], self.objs[2].transformation.translation[2] - self.objs[0].transformation.translation[2]]

            # On calcul l'angle de l'ennemi par rapport au personnage
            angle_ennemi = np.arctan(nouv_coord[0]/nouv_coord[1])

            angle_tir = self.objs[0].transformation.rotation_euler[2]
            if angle_tir > 0 :
                while angle_tir > np.pi/2 :
                    angle_tir -= np.pi/2
            else :
                while angle_tir < -np.pi/2 :
                    angle_tir += np.pi/2

            #print("tir : ", angle_tir)
            #print("ennemi : ", angle_ennemi)

            #if freq == True :
                #print("/////////")
                #print("tir")
            

            if (abs(angle_tir) > abs(angle_ennemi)-0.2) and (abs(angle_tir) < abs(angle_ennemi)+0.2) :
                
                if freq == True :
                    #print("Touché !")
                    rand_rot = random.uniform(-np.pi, np.pi)
                    self.objs[2].transformation.rotation_euler[pyrr.euler.index().yaw] += rand_rot

                    Vie -= 1
                    Vitesse += 0.04

                    if Vie <= 0 :
                        self.objs[2].visible = False


        if Vie == 3 :
            self.objs[12].visible = False
        if Vie == 2 :
            self.objs[11].visible = False
        if Vie == 1 :
            self.objs[10].visible = False
        if Vie <= 0 :
            self.objs[9].visible = False
            self.objs[8].bottomLeft = np.array([-0.6, -0.2], np.float32)                














        #if glfw.KEY_SPACE in self.touch and self.touch[glfw.KEY_SPACE] > 0:
        #    self.cam.transformation.rotation_euler = self.objs[0].transformation.rotation_euler.copy() 
        #    self.cam.transformation.rotation_euler[pyrr.euler.index().yaw] += np.pi
        #    self.cam.transformation.rotation_center = self.objs[0].transformation.translation + self.objs[0].transformation.rotation_center
        #    self.cam.transformation.translation = self.objs[0].transformation.translation + pyrr.Vector3([0, 1, 5])

    

