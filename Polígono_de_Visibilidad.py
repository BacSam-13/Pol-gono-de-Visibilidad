# Autor: Baruc Samuel Cabrera Garcia
from vispy import app
import sys
from vispy.scene import SceneCanvas
from vispy.scene.visuals import Polygon, Ellipse, Rectangle, RegularPolygon, Line
from shapely.geometry import Point
from shapely.geometry import Polygon as Polygon_geometry
from vispy.color import Color
import numpy as np
from vispy.visuals.transforms.linear import MatrixTransform
import copy

INF = float('inf')

#Colores 
white = Color("#ecf0f1")
gray = Color("#121212")
red = Color("#e74c3c")
blue = Color("#2980b9")
orange = Color("#e88834")
rosa_hex = Color("#FFC0CB")

#Dimensiones de ventana
scene_width = 700
scene_height = 700

step = np.array([.01, 0.02])
centro = np.array([scene_width/2,scene_height/2])
scale = 1

poligono_visibilidad = None
############################ Funciones
"""
    Idea:
El poligono de visibilidad consiste en mostrar lo que un robot puede ver dentro de un polígono.
Para esto, se hace uso de los vertices para proyectar "rayos" y saber el alcance de la vista del robot.
Debido a los rayos, la idea es colorear triangulos para formar la vista del robot, 
para esto ocuparemos las sharped vertex, ya que con estos vertices definimos 
los rayos que delimitan el polígono de visibilidad.

Para esto, almacenaremos los sharped vertex y la interseccion del rayo del punto al vertex con el polígono.
Recorreremos cada arista empezando desde un vertice cercano que no sea sharped vertex, 
y veremos si en ella hay una interseccion asociada a un rayo de un sharped
vertex, en ese caso, añadiremos tal interseccion e ignoraremos los demas vertices del poligono hasta encontrar
el sherped vertex asociado, y así seguiremos.

Tambien se tiene que hacer al revez, es decir, si en el inicio de la arista esta el sharped_vertex, ignoramos
los demas vertices hasta encontrar la proyeccion
Obviamente, solo incluiremos los rayos cuando el punto y el vertex_point tengan camino directo entre si
"""

def distancia(A, B):
    return np.sqrt( (B[0] - A[0])** 2 + (B[1] - A[1])**2)

def is_in_V(punto, V):
    return any((punto == vec).all() for vec in V)

def on_mouse_press(event):
    global center_circle
    global view
    global obstaculo
    global circle

    newCenter = event.pos
    center_circle = newCenter

    draw_visibility_polygon(obstaculo)

    if circle is not None:
        circle.parent = None   

    circleIdxs = np.array([(center_circle[0] + radio*np.cos(theta), center_circle[1] + radio*np.sin(theta)) for theta in np.linspace(0,2*np.pi, 50)])
    circulo = Polygon(circleIdxs, color=blue, border_width=3)
    view.add(circulo)


    view.camera.center = (centro[0],centro[1])
    circle = circulo



#Considerando un rayo que sale de v_1 a v_2, buscamos alguna intersección en w_1, w_2
def interseccion_rectas(v_1, v_2, w_1, w_2):
    """Se asume que las rectas (v_1 , v_2) y (w_1 y w_2) no son paralelas, 
    pero no sabemos si alguna tiene pendiente cero"""
    result = np.array([0,0])

    if (v_1[0] == v_2[0]):#Si la recta (v_1 , v_2) tiene pendiente cero
        x = v_1[0]
        #y = mx + b
        m_w = (w_1[1] - w_2[1]) / (w_1[0] - w_2[0])
        b_w = w_1[1]- (m_w * w_1[0])

        result[0] = x
        result[1] = m_w*x + b_w

    elif (w_1[0] == w_2[0]):#Si la recta (w_1 , w_2) tiene pendiente cero
        x = w_1[0]
        #y = mx + b
        m_v = (v_1[1] - v_2[1]) / (v_1[0] - v_2[0])
        b_v = v_1[1]- (m_v * v_1[0])

        result[0] = x
        result[1] = m_v*x + b_v

    else: #Ninguna recta es de pendiente cero

        #y = mx + b
        m_v = (v_1[1] - v_2[1]) / (v_1[0] - v_2[0])
        b_v = v_1[1] - (m_v * v_1[0])

        m_w = (w_1[1] - w_2[1]) / (w_1[0] - w_2[0])
        b_w = w_1[1]- (m_w * w_1[0])


        result[0] = (b_w - b_v) / (m_v - m_w)
        result[1] = m_v * result[0] + b_v

    return result, is_in_AB(w_1,result,w_2)

def is_in_AB(A,punto,B):#Determina si punto se encuentra en el segmento A,B
    distancia_total = distancia(A, B)
    dA = distancia(punto, A)
    dB = distancia(punto, B)

    if abs(dA + dB - distancia_total) < 1:
        return True
    else:
        return False

def is_parallel(v_1, v_2, w_1, w_2):
    m_v = (v_1[1] - v_2[1]) / (v_1[0] - v_2[0])
    m_w = (w_1[1] - w_2[1]) / (w_1[0] - w_2[0])

    if abs(m_v - m_w) < 0.1 or (abs(m_v) == INF and abs(m_w) == INF):#Para detectar pendientes iguales, o ambas infinitas
        return True
    else:
        return False

def look_for_interseccion(A,B, obstaculo):
    """
    Dado un rayo que sale de A a B, buscamos determinar 
    si dicho rayo intersecta con alguna arista de obstaculos
    """
    dist_min = INF
    for i in range( len(obstaculo) - 1):
        C = obstaculo[i]
        D = obstaculo[i+1]

        if is_parallel(A,B,C,D) == False:#Si AB y CD no son paralelas
            punto, flag = interseccion_rectas(A,B,C,D)#Buscamos su interseccion
            if flag:#Si la interseccion esta en el segmento CD
                d_B = distancia(punto, B)
                d_A = distancia(punto, A)


                if d_B < d_A:#Si la interseccion esta en la direccion A->B
                    if d_B < dist_min and d_B > 2:#Si este punto es el mas cercano
                        if is_in_AB(A,punto,B) == False:#Si la interseccion no esta entre A y B
                            point_optimo = punto
                            dist_min = d_B

    if dist_min == INF:#Si no encontramos ninguna interseccion en la direaccion A->B
        return B, False
    
    if dist_min == 0:#Si B es la interseccion
        
        return B, False

    #Si la linea que va de B a point_optimo esta fuera del poligono
    #Para esto utilizamos el punto medio de la supuesta linea formada por B y point_optimo  
    if is_in_polygon(B, point_optimo, obstaculo) == False:
        return B, False
    

    return point_optimo, True

def is_in_polygon(A,B, obstaculo):#Determina si la arista que une a A,B esta contenida en el poligono
    poligono = Polygon_geometry(obstaculo)
    punto_medio = (A + B)/2
    
    return poligono.contains(Point(punto_medio[0],punto_medio[1]))


def get_sharp_vertex(obstaculo):#Recibe el conjunto de vertices cerrado
    global view
    # 0 1 2 3 4 5 6 7 8 9 0 (1)
    #Agregamos el vertice 1 para asi poder anaizar los vecinos de los veritces de en medio de aux
    aux = np.concatenate((obstaculo, [obstaculo[1]]))
    n = len(aux)
    sharped_vertex = np.empty((0, 2), dtype=float)

    
    for i in range(1,n-1):
        vertice_a = aux[i-1]
        vertice_b = aux[i]
        vertice_c = aux[i+1]

        p_ab, flag_ab = look_for_interseccion(vertice_a, vertice_b, obstaculo)
        p_cb, flag_cb = look_for_interseccion(vertice_c, vertice_b, obstaculo)

        if flag_ab and flag_cb:#Guardamos el sharped verte

            sharped_vertex = np.append(sharped_vertex, [vertice_b], axis = 0)

    return sharped_vertex

#
def get_new_cycle(obstaculo, sharped_vertex):
    global center_circle

    n = len(obstaculo)
    obstaculo_new = obstaculo[0:n-1] #Eliminamos el ultimo elemento
    A = copy.deepcopy(center_circle) 

    min = INF
    for i in range(n-1):
        a = obstaculo[i]
        d_a = distancia(A, a)
        if d_a < min and is_in_V(a, sharped_vertex) == False:#Si rebasa el minimo, no esta en sharped_Vertex y aA esta en el poligono
            if is_in_polygon(a,A,obstaculo):
                min = d_a
                idx = i

    #Ahora, generamos un nuevo cilo de vertices, donde empezamos desde 
    #uno que se incluya en el poligono de visibilidad, es decir, el mas cercano a A y no sea un sharped_vertex
    new_cycle = np.empty((0, 2), dtype=float)
    for i in range(idx, n-1):
        new_cycle = np.append(new_cycle, [obstaculo_new[i]], axis = 0)

    for i in range(idx):
        new_cycle = np.append(new_cycle, [obstaculo_new[i]], axis = 0)

    new_cycle = np.append(new_cycle, [obstaculo_new[idx]], axis = 0)#Volvemos el nuevo ciclo cerrado
    print(f"El nuevo ciclo es: {new_cycle}")
    return new_cycle    

def is_arista(A,B, obstaculo):
    for i in range(len(obstaculo)-1):
        a = obstaculo[i]
        b = obstaculo[i+1]
        if (a == A).all() and (b == B).all():
            return True
        if (b == A).all() and (a == B).all():
            return True

def draw_visibility_polygon(obstaculo):
    global view
    global center_circle
    global poligono_visibilidad

    A = copy.deepcopy(center_circle)
    sharped_vertex = get_sharp_vertex(obstaculo)
    n_sharped_vertex = len(sharped_vertex)
    proyection_A = np.empty((0, 2), dtype=float)#Para guardar la interseccion del rayo de A hacia los vertex
    flags_proyection = np.array([]) #Para guardar las banderas que dicen si la proyeccion es valida

    for i in range(n_sharped_vertex):
        v = sharped_vertex[i]

        point, flag = look_for_interseccion(A, v, obstaculo)
        proyection_A = np.append(proyection_A, [point], axis = 0)
        if is_arista(v, point, obstaculo):
            flag = False

        flags_proyection = np.append(flags_proyection, [flag])

    
    #Procedemos a crear el polígono de visibilidad
    visibility_polython = np.empty((0, 2), dtype=float)

    new_obstaculo = get_new_cycle(obstaculo, sharped_vertex)#Conjunto de vertices analogo, donde empezamos desde un vertice seguro

    #n = len(new_obstaculo)
    start = new_obstaculo[0]
    idx = 0
    flag_skip = False #Nos dice si estamos saltando vertices
    while True:#Mientras no demos una vuelta completa
        a = new_obstaculo[idx]
        print(f"Añadimos a {a}")
        visibility_polython = np.append(visibility_polython, [a], axis = 0)

        for j in range(n_sharped_vertex):#Vemos si a es un sharp vertex
            if (a == sharped_vertex[j]).all() and flags_proyection[j]:#Si es un vertex con proyeccion valida
                look_for = proyection_A[j]#Buscaremos su proyeccion
                flag_skip = True#Saltaremos vertices
                break

        if flag_skip == False:#Si a no es un vertex
            #Veremos si su arista tiene una proyeccion valida
            d_min = INF
            for j in range(n_sharped_vertex):
                proy = proyection_A[j]
                if is_in_AB(a, proy, new_obstaculo[idx+1]) and flags_proyection[j]:#Si hay una proyeccion valida
                    dist = distancia(a,proy)
                    if dist < d_min:#Tomamos la proyeccion mas cercana a a
                        d_min = dist
                        closest_proy = proy
                        closest_j = j

            if d_min != INF:#Si detectamos algo, agregamos la que tenga la distancia menor a A
                print(f"Añadimos a {closest_proy}")
                visibility_polython = np.append(visibility_polython, [closest_proy], axis = 0)#La agregamos
                look_for = sharped_vertex[closest_j]#Buscamos por su vertice
                flag_skip = True

        
        if flag_skip:#Si buscaremos algo
            print(f"\tBuscamos a {look_for}")
            while True:
                idx += 1
                a = new_obstaculo[idx]


                if (a == look_for).all():#Si a es el vertex que buscamos
                    flag_skip = False
                    print(f"Añadimos a {a}")
                    visibility_polython = np.append(visibility_polython, [a], axis = 0)#Lo añadimos

                    #Antes de volver al primer while, hay que revisar si hay una proyeccion entre a y el vertice siguiente
                    b = new_obstaculo[idx+1]
                    for j in range(n_sharped_vertex):
                            proy = proyection_A[j]
                            if is_in_AB(a, proy, b) and flags_proyection[j]:#Si hay una proyeccion valida
                                print(f"Añadimos a {proy} entre {a} y {b}")
                                visibility_polython = np.append(visibility_polython, [proy], axis = 0)#La agregamos
                                look_for = sharped_vertex[j]#Buscamos por su vertice
                                flag_skip = True
                                break
                    idx += 1
                    break

                b = new_obstaculo[idx+1]
                if is_in_AB(a, look_for, b) and (b == look_for).all() == False:#Si look_for es una proy en ab y no es b
                    flag_skip = False
                    idx += 1
                    print(f"Añadimos a {look_for}")
                    visibility_polython = np.append(visibility_polython, [look_for], axis = 0)#La agregamos
                    break
            print("\n")
        else:
            idx += 1

        if (start == new_obstaculo[idx]).all():#Si ya volvimos al inicio
            break
          
    if poligono_visibilidad is not None:
        poligono_visibilidad.parent = None   

    poligono = Polygon(visibility_polython, color=rosa_hex) #Para mostrar

    view.add(poligono)
    poligono_visibilidad = poligono
    

###########################
canvas = SceneCanvas(keys='interactive', title='Polygon Example',
                     show=True, size = (scene_width, scene_height), autoswap=False, vsync=True)
canvas.connect(on_mouse_press)

view = canvas.central_widget.add_view()
view.bgcolor = gray

#Poligono(s) a considerar
vertices_1 = np.array([[100.0, 100.0], [500.0, 100.0], [550.0, 250.0], [500.0, 400.0], [450.0, 400.0], 
                       [400.0, 300.0], [350.0, 400.0], [250.0, 400.0], [200.0, 300.0], [150.0, 400.0], 
                       [100.0, 400.0], [50.0, 250.0], [100.0, 100.0]])

vertices_2 = np.array([[200.0,100.0], [300,100.0], [300,200.0], [400,200.0], 
                       [400.0,100.0], [500,100.0], [500,300.0], [200,300.0], [200,100.0]])

chosen = vertices_1 #Modificar si quieres otro polígono

obstaculo_1 = Polygon(chosen, color=orange,parent = view.scene) #Para mostrar

obstaculo = chosen

for i in range(len(obstaculo)-1):
    print(f"({obstaculo[i][0]}, - {obstaculo[i][1]})")

#Punto a dibujar
radio = 10
center_circle = np.array([300,250])

draw_visibility_polygon(obstaculo)

circleIdxs = np.array([(center_circle[0] + radio*np.cos(theta), center_circle[1] + radio*np.sin(theta)) for theta in np.linspace(0,2*np.pi, 50)])
circle = Polygon(circleIdxs, color=blue, border_width=3)
view.add(circle)
circle.transform = MatrixTransform()

timer = app.Timer()

timer.start()

if __name__ == '__main__':
    if sys.flags.interactive != 1:
        app.run()