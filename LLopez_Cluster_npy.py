# -*- coding: utf-8 -*-
import random
import numpy as np
import os
import argparse
import gc
import time

# Librerias-------------------------------------------------------------------------------

# CLASES -----------------------------------------------------------------------------------------

class grilla:
    def __init__(self, tam_X, tam_Y):
        self.mapa= np.zeros((tam_X,tam_Y),dtype=np.uint8)
        self.tamX=tam_X
        self.tamY=tam_Y
    def colocar_ave(self, bird):
        self.mapa[bird.last_placex,bird.last_placey]=1
    def limpiar_ave(self,bird):
        self.mapa[bird.last_placex,bird.last_placey]=0
    def limpiar(self):
        del self.mapa
        del self.tamX
        del self.tamY
        gc.collect()
    def Counting_Box(self, nro_div):
        suma = 0
        tam_celda_X = self.tamX // nro_div
        tam_celda_Y = self.tamY // nro_div

        for i in range(nro_div):
            for j in range(nro_div):
                encontrado = False
                for x in range(i * tam_celda_X, (i + 1) * tam_celda_X):
                    for y in range(j * tam_celda_Y, (j + 1) * tam_celda_Y):
                        if self.mapa[x, y] == 1:
                            suma += 1
                            encontrado = True
                            break
                    if encontrado:
                        break

        return suma
                
    def copia(self):
        aux=grilla(self.tamX,self.tamY)
        aux.mapa=self.mapa.copy()
        return aux
    def encontrar_colonia (self,dist_colonia):
        vec_colonias=[]
        mapa_aux=self.copia()
        for i in range (self.tamX):
            for j in range (self.tamY):
                if(mapa_aux.mapa[i,j]!=0):
                    new_colony= Colonia (0,[])
                    flood_distancia_n([i,j],mapa_aux,new_colony,dist_colonia)
                    vec_colonias.append(new_colony)

        count=0
        for i in range (self.tamX):
            for j in range (self.tamY):
                if (mapa_aux.mapa[i,j]!=0):
                    count += 1
        if (count==0):
            print("Se registraron todas las aves en colonias")
        else:
            
            print("NO se registraron todas las aves en colonias")
        mapa_aux.limpiar()
        return vec_colonias
    
        
class Ave:
    def __init__(self, edad, edad_max, mapon, primera_tanda,dist_max,P_base): #creo el ave con algunas propiedades
        self.edad=edad
        self.vivo=True
        self.edad_max=edad_max
        self.fitness=(self.edad_max-self.edad)
        self.dist_max=dist_max
        nro_random_prob=0.00
        if primera_tanda:
            self.last_placex=random.randint(0,mapon.tamX-1)
            self.last_placey=random.randint(0,mapon.tamY-1)
            while (mapon.mapa[self.last_placex,self.last_placey]==1):
                self.last_placex=random.randint(0,mapon.tamX-1)
                self.last_placey=random.randint(0,mapon.tamY-1)
        else:
            self.last_placex=random.randint(0,mapon.tamX-1)
            self.last_placey=random.randint(0,mapon.tamY-1)
            nro_random_prob=random.random()     #tiro nro random, si es mayor a la probabilidad de poner nido, entonces busca otro lugar
            FA=self.fn_atraccion(dist_max,mapon)       # Se impuso una probabilidad minima de 0.0001 de formar nido
            while (mapon.mapa[self.last_placex,self.last_placey]==1 or ((nro_random_prob>FA) and (FA>P_base)) or ((nro_random_prob>P_base) and (FA<P_base))):
                        self.last_placex=random.randint(0,mapon.tamX-1)
                        self.last_placey=random.randint(0,mapon.tamY-1)
                        nro_random_prob=random.random() 
                        FA=self.fn_atraccion(dist_max,mapon)
    def envejecer(self):            #agrego la funcion para envejecer
        self.edad=self.edad+1
        if (self.edad>=self.edad_max):
            self.vivo=False
    def actual_ult_nido(self,X,Y):  #agrego una memoria del nido anterior
        self.last_nest=(X,Y)
    def nro_vecinos(self, i, mapita):
        suma = 0.0
        tamX, tamY = mapita.tamX, mapita.tamY
        x, y = self.last_placex, self.last_placey
        
        # Recorremos el anillo de vecinos a distancia i
        for dx in range(-i, i + 1):
            for dy in range(-i, i + 1):
                if abs(dx) == i or abs(dy) == i:  # Solo los bordes del cuadrado
                    nx = (x + dx) % tamX  # Aritmética modular para bordes toroidales
                    ny = (y + dy) % tamY
                    suma += mapita.mapa[nx, ny]
                    
        return suma
    def fn_atraccion(self, dist_max, mapita):       #creo la funcion atraccion
        C=0                                         #establesco la constante de normalizacion
        mini_prob=0
        nro_sitios=0
        i=1
        while (i<=dist_max):
            C=C+1/i
            nro_sitios=2*4*i
            mini_prob=(self.nro_vecinos(i,mapita)/(nro_sitios)*(1/i))+mini_prob 
            i=i+1
        probabilidad=C*mini_prob
        return probabilidad
    def limpiar(self):
        del self.edad
        del self.vivo
        del self.edad_max
        del self.fitness
        del self.dist_max
        del self.last_placex
        del self.last_placey
        gc.collect()

class Colonia:
    def __init__(self, tam, vec_pos):
        self.tam=tam
        self.vec_pos=vec_pos
    def agregar_ave(self, new_bird_pos):
        self.tam += 1
        self.vec_pos.append(new_bird_pos)
    def nro_nidos(self):
        return self.tam
    def limpiar(self):
        del self.tam
        del self.vec_pos
        gc.collect()

# FUNCIONES -----------------------------------------------------------------------------------

def nueva_tanda(mapita,edad_max_aves, nro_aves,parvada,P_base):
    primera_tanda=False
    rg_vision=parvada[0].dist_max
    X=0
    Y=0
    for i in range (nro_aves):
        pajaro=parvada.pop(0)
        mapita.limpiar_ave(pajaro)
        nueva_ave=Ave(0,edad_max_aves,mapita,primera_tanda,rg_vision,P_base)
        parvada.append(nueva_ave)
        mapita.colocar_ave(nueva_ave)


def Full_Counting_Box_2(mapilla,pot_max_div):
    i=2
    max=2**pot_max_div
    Vector=[]
    while (i<=max):
        Vector.append((mapilla.tamX/i,mapilla.Counting_Box(i)))
        i=i*2
    print("Counting Box: OK")
    return Vector

def flood_distancia_n(coord, mapita, colonia, n):
    x,y=coord[0],coord[1]
    if mapita.mapa[x,y] == 0:
        return
    
    colonia.agregar_ave(coord)
    mapita.mapa[coord] = 0  # Marca la celda como visitada

    tamX, tamY = mapita.tamX, mapita.tamY

    # Recorre todos los vecinos dentro de la distancia n
    for dx in range(-n, n + 1):
        for dy in range(-n, n + 1):
            if dx == 0 and dy == 0:
                continue  # Salta la celda actual
            
            vecino = ((coord[0] + dx) % tamX, (coord[1] + dy) % tamY)

            if mapita.mapa[vecino] == 1:
                flood_distancia_n(vecino, mapita, colonia, n)
    

def obtener_vec_tam_colonias(vec_colonias):  #funcion que devuelve un vector con el tamaño de las colonias a partir de un vector de colonias
    vec_aux=[]
    for i in range(len(vec_colonias)):
        vec_aux.append(vec_colonias[i].nro_nidos())
    return vec_aux

def obtener_freq (vec_colonias): #funcion en donde pide un vector de colonias y devuelve un vector conjuntos de colonias y sus freq
    vec_out=[]
    vec_aux=[0]   #registro en este vector los tamaño de colonias ya analizados
    vec_agregar=[]
    normalizacion=len(vec_colonias)
    nro_NO_repetido=True
    count=0
    for i in range (len(vec_colonias)):
        j=i
        k=0
        for k in range (len(vec_aux)):
            if (vec_colonias[i].nro_nidos()==vec_aux[k]):
                nro_NO_repetido=False
        while(j<len(vec_colonias) and nro_NO_repetido):
            if (vec_colonias[i].nro_nidos()==vec_colonias[j].nro_nidos()):
                count += 1
            j += 1
        if nro_NO_repetido:
            vec_aux.append(vec_colonias[i].nro_nidos())
            vec_agregar=[vec_colonias[i].nro_nidos(),count]
            vec_out.append(vec_agregar)
        nro_NO_repetido=True
        count=0
    return vec_out



def f(nro_obj,bin_min,bin_max,tot_cluster):
    aux=nro_obj/((bin_max-bin_min)*tot_cluster)
    return aux

def encontrar_bines(vec_colonias):
    flag=True
    aux_Y=2
    n=0  #Tamaño maximo de colonia
    q=0
    Hay_bines_vacios=False
    aux=0
    terminar=False
    for j in range(len(vec_colonias)):
        if (vec_colonias[j].nro_nidos()>n):
            n=vec_colonias[j].nro_nidos()
    print(f"La colonia de mayor tamaño tiene {n} nidos")
    while(flag):
        Hay_bines_vacios=False
        pot_max=0       #nro de bines es igual a la potencia maxima, por como se planea graficar
        while(aux_Y**pot_max<n):
            pot_max += 1
        if(pot_max<=1):aux_Y=1;pot_max=n;vec_bines_count=np.zeros((pot_max),dtype=int)  #Para graficar un solo bin mejor no graficarlo
        else:
            if(n-(aux_Y**pot_max)>=0):
                vec_bines_count=np.zeros((pot_max+1),dtype=int)
            else:
                vec_bines_count=np.zeros((pot_max),dtype=int)
        terminar= False
        while(q<(len(vec_colonias))):
            for aux in range(len(vec_bines_count)):           #busco a que bin pertenece y una vez lo encuentro dejo de buscar
                tam_aux=vec_colonias[q].nro_nidos()
                if ((tam_aux<(aux_Y**(aux+1))) and (tam_aux>=(aux_Y**aux))):
                    vec_bines_count[aux] += 1
                    aux=0
                    terminar=True
                if terminar:break
            q += 1
            terminar=False
        for k in range(len(vec_bines_count)):
            if (vec_bines_count[k]==0):
                Hay_bines_vacios= True  #La idea es que el histograma se haga con los bines de menor ancho posible sin que quede alguno vacio
        if (not Hay_bines_vacios):flag=False
        else: aux_Y += 1;q=0
    vec_bines_out=np.zeros((len(vec_bines_count),2),dtype=float)
    if (len(vec_bines_count)==n):
        for i in range(len(vec_bines_out)):
            vec_bines_out[i]=[i,f(vec_bines_count[i],aux_Y**i,aux_Y**(i+1),len(vec_colonias))]
    else:
        for i in range(len(vec_bines_out)):
            vec_bines_out[i]=[10**((np.log10(aux_Y**i)+np.log10(aux_Y**(i+1)-1))/2),f(vec_bines_count[i],aux_Y**i,aux_Y**(i+1),len(vec_colonias))]
            #print(f'se esta obtuvo el siguiente histograma :{vec_bines_out[i]}')
    print("f(x): Ok")
    return vec_bines_out

# MAIN ---------------------------------------------------------------------------------------------------------------------------

def main (pot_tam_grilla=10, nro_tandas=3 ,proporcion_aves_iniciales=0.05, proporcion_aves=0.01, edad_max_aves=5, rango_vision=3, distancia_colonia=1, Prob_base=0.0001 ,nombre_gral="Carpeta Grande", nombre_chiquito="Carpeta Chiquita"):
        
    inicio_carga = time.time()
    tam_grilla=2**pot_tam_grilla
    Counting_Box_divisiones=pot_tam_grilla
    nro_aves=int(proporcion_aves*tam_grilla*tam_grilla)
    mapita = grilla(tam_grilla,tam_grilla) #creo el mapa
    parvada=[]
    vec_colonias=[]
    vec_mapas=[]
    primera_tanda = True
    for i in range(nro_aves):
        if (i<=int(nro_aves*proporcion_aves_iniciales)):
            parvada.append(Ave(0,edad_max_aves,mapita,primera_tanda,rango_vision,Prob_base))
            mapita.colocar_ave(parvada[i])
        else:
            primera_tanda = False
            parvada.append(Ave(0,edad_max_aves,mapita,primera_tanda,rango_vision,Prob_base))
            mapita.colocar_ave(parvada[i])

    ruta_actual = os.path.dirname(os.path.abspath(__file__))  # Carpeta del script
    carpetita = os.path.join(ruta_actual,nombre_gral)

    # Intentamos crear "carpetita"
    try:
        os.makedirs(carpetita, exist_ok=True)
        print(f"Carpeta creada/existente: {carpetita}")
    except Exception as e:
        print(f"Error creando carpetita: {e}")

    # Definir la carpeta de salida
    try:
        carpeta = os.path.join(carpetita, nombre_chiquito)

        os.makedirs(carpeta, exist_ok=True)
        print(f"Carpeta creada/existente: {carpeta}")
    except Exception as e:
        print(f"Error creando carpeta: {e}")

    # Guardar archivo
    try:
        archivo_salida = os.path.join(carpeta, f"Mapa{0}.npy")
        np.save(archivo_salida,mapita.mapa)
        print(f"Archivo guardado en: {archivo_salida}")
    except Exception as e:
        print(f"Error guardando archivo: {e}")

    vec_mapas.append(mapita)

    for _ in range(nro_tandas):  # Simula X numero de generaciones
        print(_)
        nueva_tanda(mapita,edad_max_aves ,nro_aves, parvada,Prob_base)  # Actualiza la grilla
        np.save(os.path.join(carpeta, f"Mapa{_+1}.npy"), mapita.mapa) 
        vec_mapas.append(mapita)

    for _ in range (nro_tandas+1):
        mapa_aux=grilla(tam_grilla,tam_grilla)
        mapa_aux.mapa=np.load(os.path.join(carpeta, f"Mapa{_}.npy"))
        Datos_Box_2=Full_Counting_Box_2(mapa_aux,Counting_Box_divisiones) #Crea un vector con: (longitud del lado del box, cantidad de box con aves)
        np.savetxt(os.path.join(carpeta, f"Datos_Box_{_}.csv"), Datos_Box_2, delimiter="\t", fmt="%d") 
        mapa_aux.limpiar()

    for _ in range(nro_tandas+1):
        colonia_aux = grilla(tam_grilla, tam_grilla)
        colonia_aux.mapa = np.load(os.path.join(carpeta, f"Mapa{_}.npy"))
        vec_colonias = colonia_aux.encontrar_colonia(distancia_colonia)

        # Verifica si hay una sola colonia o si todas las colonias tienen tamaño 1
        if len(vec_colonias) <= 1 or all(col.nro_nidos() == 1 for col in vec_colonias):
            print(f"Se saltea el cálculo de vec_fx para Mapa{_}.npy (colonias insuficientes)")
        else:
            vec_fx = encontrar_bines(vec_colonias)
            np.savetxt(os.path.join(carpeta, f"Vec_fx_{_}.csv"), vec_fx, delimiter="\t", fmt="%f")

        colonia_aux.limpiar()

    #Limpieza de variables
    mapita.limpiar()
    fin_carga = time.time()  
    print(f"------------------------------------------------------")
    print(f"Tiempo de carga: {int((fin_carga - inicio_carga)/60)} min {(fin_carga - inicio_carga)%60} seg")
    print(f"-------------------------------------------------------")

# DATOS POR CONSOLA ----------------------------------------------------------------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--rango_vision", type=int, default=3, help="Valor de rango_vision")
    parser.add_argument("--proporcion_aves", type=float, default=0.01, help="Densidad de aves por area")
    parser.add_argument("--pot_tam_grilla", type=int, default=10, help="Potencia del tamaño de la grilla en base 2, por ejemplo 10=>1024")   
    parser.add_argument("--nro_tandas", type=int, default=3, help="Número de tandas")
    parser.add_argument("--proporcion_aves_iniciales", type=float, default=0.05, help="Proporcion de aves semilla")
    parser.add_argument("--edad_max_aves", type=int, default=5, help="Edad maxima de las aves")
    parser.add_argument("--distancia_colonia", type=int, default=1, help="Distancia entre aves que entran en la definicion de colonia")
    parser.add_argument("--Prob_base", type=float, default=0.0001, help="Probabilidad base de formar nido")
    parser.add_argument("--nombre_gral", type=str, default="Carpeta Grande", help="Nombre de la carpeta del analisis a realizar")
    parser.add_argument("--nombre_chiquito", type=str, default="Carpeta Chiquita", help="Nombre de la carpeta con el parametro cambiado")

    args = parser.parse_args()
    # Llamar a main con los valores de argparse
    main(pot_tam_grilla=args.pot_tam_grilla, 
     nro_tandas=args.nro_tandas, 
     proporcion_aves_iniciales=args.proporcion_aves_iniciales, 
     proporcion_aves=args.proporcion_aves, 
     edad_max_aves=args.edad_max_aves, 
     rango_vision=args.rango_vision, 
     distancia_colonia=args.distancia_colonia, 
     Prob_base=args.Prob_base,
     nombre_gral=args.nombre_gral, 
     nombre_chiquito=args.nombre_chiquito)