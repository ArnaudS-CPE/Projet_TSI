from viewerGL import ViewerGL
import glutils
from mesh import Mesh
from cpe3d import Object3D, Camera, Transformation3D, Text
import numpy as np
import OpenGL.GL as GL
import pyrr



def main():
    viewer = ViewerGL()

    viewer.set_camera(Camera())
    viewer.cam.transformation.rotation_center = viewer.cam.transformation.translation.copy()
    viewer.cam.transformation.rotation_euler[pyrr.euler.index().yaw] += 0
    program3d_id = glutils.create_program_from_file('shader.vert', 'shader.frag')
    programGUI_id = glutils.create_program_from_file('gui.vert', 'gui.frag')


    # Personnage
    m = Mesh.load_obj('perso2.obj')
    m.normalize()
    m.apply_matrix(pyrr.matrix44.create_from_scale([0.8, 0.8, 0.8, 1]))
    tr = Transformation3D()
    tr.translation.y = -np.amin(m.vertices, axis=0)[1]
    tr.translation.z = -3
    tr.rotation_center.y = np.pi/2
    texture = glutils.load_texture('couleur_perso.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, tr)
    viewer.add_object(o)


    # Sol
    m = Mesh()
    p0, p1, p2, p3 = [-25, 0, -25], [25, 0, -25], [25, 0, 25], [-25, 0, 25]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c + t2], [p3 + n + c + t3]], np.float32)
    m.faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    texture = glutils.load_texture('lava.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_object(o)


    # Ennemi
    m = Mesh.load_obj('stegosaurus.obj')
    m.normalize()
    m.apply_matrix(pyrr.matrix44.create_from_scale([3, 3, 3, 1]))
    tr = Transformation3D()
    tr.translation.y = -np.amin(m.vertices, axis=0)[1]
    texture = glutils.load_texture('stegosaurus.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, tr)
    viewer.add_object(o)


    # Murs
    # arrière
    m = Mesh()
    p0, p1, p2, p3= [-25, 0, -25], [25, 0, -25], [-25, 10, -25], [25, 10, -25]
    liste = [p0 , p1 , p2 , p3]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p3 + n + c + t3], [p1 + n + c + t1], [p2 + n + c + t2],[p0 + n + c + t0]], np.float32)
    m.faces = np.array([[1, 0, 2],[1,3,2]], np.uint32)
    i= 0
    for k in range(0,len(liste)):
        if liste[k][1] == 0 and i == 0:
            i=1
            viewer.collisions[0].append(liste[k]) #p1
        if i == 1 and liste[k][1] == 0:
            viewer.collisions[1].append(liste[k]) #p2
    texture = glutils.load_texture('Slime2.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_object(o)

    # avant
    m = Mesh()
    p0, p1, p2, p3= [25, 0, 25], [-25, 0, 25], [25, 10, 25], [-25, 10, 25]
    liste = [p0 , p1 , p2 , p3]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p3 + n + c + t3], [p1 + n + c + t1], [p2 + n + c + t2],[p0 + n + c + t0]], np.float32)
    m.faces = np.array([[1, 0, 2],[1,3,2]], np.uint32)
    i= 0
    for k in range(0,len(liste)):
        if liste[k][1] == 0 and i == 0:
            i=1
            viewer.collisions[1].append(liste[k]) #p1
        if i == 1 and liste[k][1] == 0:
            viewer.collisions[0].append(liste[k]) #p2
    texture = glutils.load_texture('Slime2.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_object(o)

    # gauche
    m = Mesh()
    p0, p1, p2, p3= [25, 0, 25], [25, 0, -25], [25, 10, 25], [25, 10, -25]
    liste = [p0 , p1 , p2 , p3]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p3 + n + c + t3], [p1 + n + c + t1], [p2 + n + c + t2],[p0 + n + c + t0]], np.float32)
    m.faces = np.array([[1, 0, 2],[1,3,2]], np.uint32)
    i= 0
    for k in range(0,len(liste)):
        if liste[k][1] == 0 and i == 0:
            i=1
            viewer.collisions[0].append(liste[k]) #p1
        if i == 1 and liste[k][1] == 0:
            viewer.collisions[1].append(liste[k]) #p2
    texture = glutils.load_texture('Slime2.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_object(o)

    # droite
    m = Mesh()
    p0, p1, p2, p3= [-25, 0, -25], [-25, 0, 25], [-25, 10, -25], [-25, 10, 25]
    liste = [p0 , p1 , p2 , p3]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p3 + n + c + t3], [p1 + n + c + t1], [p2 + n + c + t2],[p0 + n + c + t0]], np.float32)
    m.faces = np.array([[1, 0, 2],[1,3,2]], np.uint32)
    i= 0
    for k in range(0,len(liste)):
        if liste[k][1] == 0 and i == 0:
            i=1
            viewer.collisions[1].append(liste[k]) #p1
        if i == 1 and liste[k][1] == 0:
            viewer.collisions[0].append(liste[k]) #p2
    texture = glutils.load_texture('Slime2.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_object(o)


    # Toit
    p0, p1, p2, p3= [-25, 10, -25], [25, 10, -25], [25, 10, 25], [-25, 10, 25]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c + t2], [p3 + n + c + t3]], np.float32)
    m.faces = np.array([[0, 1, 2],[0,2,3]], np.uint32)
    texture = glutils.load_texture('FIREBLU2.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_object(o)


    # Texte victoire
    vao = Text.initalize_geometry()    
    texture = glutils.load_texture('fontB.jpg')
    o = Text('VICTOIRE !', np.array([-0.6, -0.2], np.float32), np.array([0.6, 0.2], np.float32), vao, 2, programGUI_id, texture)
    viewer.add_object(o)


    # Coeurs de vie
    m = Mesh.load_obj('heart.obj')
    m.normalize()
    m.apply_matrix(pyrr.matrix44.create_from_scale([0.3, 0.3, 0.3, 1]))
    vao = m.load_to_gpu()
    texture = glutils.load_texture('rouge.jpg')
    for i in range(4):
        tr = Transformation3D()
        tr.translation.y = -np.amin(m.vertices, axis=0)[1] + 3
        o = Object3D(vao, m.get_nb_triangles(), program3d_id, texture, tr)
        viewer.add_object(o)


    # Projectile
    m = Mesh.load_obj('boule.obj')
    m.normalize()
    m.apply_matrix(pyrr.matrix44.create_from_scale([0.2, 0.2, 0.2, 1]))
    tr = Transformation3D()
    tr.translation.y = -100
    tr.translation.z = -5
    tr.rotation_center.z = 1
    texture = glutils.load_texture('stegosaurus.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, tr)
    viewer.add_object(o)


    viewer.run()


if __name__ == '__main__':
    main()