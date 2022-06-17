#!/usr/bin/env python3

import OpenGL.GL as GL
import glfw
import pyrr
import numpy as np
from cpe3d import Object3D
import random



class ViewerGL:
    def __init__(self):
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

            # Déplacement de l'ennemi
            timer = glfw.get_time()

            if timer <= (3 + (compteur*12)) :
                self.objs[2].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, 0.05]))
                c = 1
            elif (timer > 3+(compteur*12)) and (timer <= 6+(compteur*12)) :
                self.objs[2].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, 0.05]))      
                c = 2
            elif (timer > 6+(compteur*12)) and (timer <= 9+(compteur*12)) :
                self.objs[2].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, 0.05]))      
                c = 3
            elif (timer > 9+(compteur*12)) and (timer <= 12+(compteur*12)) :
                self.objs[2].transformation.translation += \
                    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[2].transformation.rotation_euler), pyrr.Vector3([0, 0, 0.05]))      
                c = 4
            else :
                compteur += 1

            # Teste si l'ennemi doit tourner
            L[0] = L[1]
            L[1] = c
            if (L[0] < L[1]) or (L[0] == 4 and L[1] == 1) :
                self.objs[2].transformation.rotation_euler[pyrr.euler.index().yaw] += np.pi/2


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
        rand_x = random.uniform(-23, 23)
        rand_z = random.uniform(-23, 23)
        rand_rot = random.uniform(-np.pi, np.pi)

        if debut == 0 :
            self.objs[2].transformation.translation += \
                pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([rand_x, 0, rand_z]))
            self.objs[2].transformation.rotation_euler[pyrr.euler.index().yaw] += rand_rot
            #print(self.objs[2].transformation.translation)

            #self.objs[5].visible = False



    
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

        
        def centrer_cam () :
            #self.cam.transformation.rotation_euler = self.objs[0].transformation.rotation_euler.copy() 
            #self.cam.transformation.rotation_euler[pyrr.euler.index().yaw] += np.pi
            self.cam.transformation.rotation_center = self.objs[0].transformation.translation + self.objs[0].transformation.rotation_center
            self.cam.transformation.translation = self.objs[0].transformation.translation + pyrr.Vector3([0, 1, 5])



        if glfw.KEY_W in self.touch and self.touch[glfw.KEY_W] > 0:
            self.objs[0].transformation.translation += \
                pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([0, 0, 0.2]))
            #self.objs[0].transformation.translation += \
            #    pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([0, 0.01, 0]))
            
            #centrer_cam()

        if glfw.KEY_S in self.touch and self.touch[glfw.KEY_S] > 0:
            self.objs[0].transformation.translation -= \
                pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([0, 0, 0.2]))
            #centrer_cam()

        if glfw.KEY_A in self.touch and self.touch[glfw.KEY_A] > 0:
            self.objs[0].transformation.translation += \
                pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([0.06, 0, 0]))
            #centrer_cam()

        if glfw.KEY_D in self.touch and self.touch[glfw.KEY_D] > 0:
            self.objs[0].transformation.translation -= \
                pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([0.06, 0, 0]))
            #centrer_cam()




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

        #centrer_cam()
        #self.objs[6].transformation.rotation_center =  -self.objs[0].transformation.rotation_center
        #self.objs[6].transformation.translation = self.objs[0].transformation.translation  
        #self.objs[6].transformation.rotation_euler = self.objs[0].transformation.rotation_euler


        # Tir
        if glfw.KEY_E in self.touch and self.touch[glfw.KEY_E] > 0:

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

            if (abs(angle_tir) > abs(angle_ennemi)-0.2) and (abs(angle_tir) < abs(angle_ennemi)+0.2) :
                print("Touché !")

                #rand_x = random.uniform(-23, 23)
                #rand_z = random.uniform(-23, 23)
                rand_rot = random.uniform(-np.pi, np.pi)
                self.objs[2].transformation.rotation_euler[pyrr.euler.index().yaw] += rand_rot

            #for i in range (len(self.objs)) :
            #    print(self.objs[i])









        #if glfw.KEY_SPACE in self.touch and self.touch[glfw.KEY_SPACE] > 0:
        #    self.cam.transformation.rotation_euler = self.objs[0].transformation.rotation_euler.copy() 
        #    self.cam.transformation.rotation_euler[pyrr.euler.index().yaw] += np.pi
        #    self.cam.transformation.rotation_center = self.objs[0].transformation.translation + self.objs[0].transformation.rotation_center
        #    self.cam.transformation.translation = self.objs[0].transformation.translation + pyrr.Vector3([0, 1, 5])

    

