from viewerGL import ViewerGL
import glutils
from mesh import Mesh
from cpe3d import Object3D, Camera, Transformation3D, Text
import numpy as np
import OpenGL.GL as GL
import pyrr
import random


def main():
    viewer = ViewerGL()

    viewer.set_camera(Camera())
    #viewer.cam.transformation.translation.y = 2
    viewer.cam.transformation.rotation_center = viewer.cam.transformation.translation.copy()
    viewer.cam.transformation.rotation_euler[pyrr.euler.index().yaw] += 0
    program3d_id = glutils.create_program_from_file('shader.vert', 'shader.frag')
    programGUI_id = glutils.create_program_from_file('gui.vert', 'gui.frag')





    munition = 10
    vie = 99





    m = Mesh.load_obj('stegosaurus.obj')
    m.normalize()
    m.apply_matrix(pyrr.matrix44.create_from_scale([1.5, 1.5, 1.5, 1]))
    tr = Transformation3D()
    tr.translation.y = -np.amin(m.vertices, axis=0)[1]
    tr.translation.z = -5
    tr.rotation_center.z = 1
    texture = glutils.load_texture('stegosaurus.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, tr)
    viewer.add_object(o)
    print(m)



    #sol
    m = Mesh()
    p0, p1, p2, p3 = [-25, 0, -25], [25, 0, -25], [25, 0, 25], [-25, 0, 25]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c + t2], [p3 + n + c + t3]], np.float32)
    m.faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    texture = glutils.load_texture('lava.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_object(o)




    #Ennemi
    m = Mesh()
    p0, p1, p2, p3 = [-0.5, 2, 0], [0.5, 2, 0], [0.5, 0, 0], [-0.5, 0, 0]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c + t2], [p3 + n + c + t3]], np.float32)
    m.faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    texture = glutils.load_texture('grass.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_object(o)






    #Interface

    #Vie
    m = Mesh()
    p0, p1, p2, p3 = [-2, -1, -1.5], [-1.3, -1, -1.5], [-1.3, -0.1, -1.5], [-2, -0.1, -1.5]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c + t2], [p3 + n + c + t3]], np.float32)
    m.faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    texture = glutils.load_texture('grass.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, tr)
    viewer.add_object(o)

    vao = Text.initalize_geometry()    
    texture = glutils.load_texture('fontB.jpg')
    o = Text('Vie:', np.array([0.74, -0.7], np.float32), np.array([1, -0.62], np.float32), vao, 2, programGUI_id, texture)
    viewer.add_object(o)
    o = Text(str(vie), np.array([0.7, -1], np.float32), np.array([1, -0.7], np.float32), vao, 2, programGUI_id, texture)
    viewer.add_object(o)

    #Munition
    m = Mesh()
    p0, p1, p2, p3 = [2, -0.1, -1.5], [1.3, -0.1, -1.5], [1.3, -1, -1.5], [2, -1, -1.5]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c + t2], [p3 + n + c + t3]], np.float32)
    m.faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    texture = glutils.load_texture('grass.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, tr)
    viewer.add_object(o)

    vao = Text.initalize_geometry()    
    texture = glutils.load_texture('fontB.jpg')
    o = Text('Mun.:', np.array([-1, -0.7], np.float32), np.array([-0.73, -0.62], np.float32), vao, 2, programGUI_id, texture)
    viewer.add_object(o)
    o = Text(str(munition), np.array([-1, -1], np.float32), np.array([-0.7, -0.7], np.float32), vao, 2, programGUI_id, texture)
    viewer.add_object(o)

    #murs
    p0, p1, p2, p3= [-25, 0, -25], [-25, 10, -25], [25, 0, -25], [25, 10, -25]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p3 + n + c + t3], [p1 + n + c + t1], [p2 + n + c + t2],[p0 + n + c + t0]], np.float32)
    m.faces = np.array([[1, 0, 2],[1,3,2]], np.uint32)
    texture = glutils.load_texture('Slime2 (1).jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_object(o)

    #toit
    p0, p1, p2, p3= [-25, 10, -25], [25, 10, -25], [25, 10, 25], [-25, 10, 25]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c + t2], [p3 + n + c + t3]], np.float32)
    m.faces = np.array([[0, 1, 2],[0,2,3]], np.uint32)
    texture = glutils.load_texture('FIREBLU2.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_object(o)

    #hitbox
    m = Mesh.load_obj('cubax.obj')
    m.normalize()
    m.apply_matrix(pyrr.matrix44.create_from_scale([2, 2, 2, 1]))
    tr = Transformation3D()
    tr.translation.y = -np.amin(m.vertices, axis=0)[1]
    tr.translation.z = -10
    tr.rotation_center.z = 0.2










    viewer.run()


if __name__ == '__main__':
    main()