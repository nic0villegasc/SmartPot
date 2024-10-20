# IMPORTANTE: Unicamente edite aqui su apellido (un apellido) y (un nombre), sin acentos ni caracteres especiales
NOMBRE = "Catalina"
APELLIDO = "Maldonado"

#!pip install pulp
# IMPORTANTE: Si usa Google Colab, no olvide ejecutar "!pip install pulp" antes de ejecutar este codigo

import pulp as PLP  # IMPORTANTE: No elimine esta linea
from pulp import * 
import traceback

t_ej = [0,2,11,15,30] # IMPORTANTE: No elimine esta linea
p_ej = [0,6,1,2,4]       # IMPORTANTE: No elimine esta linea
d_ej = [10,40,20,12,70]    # IMPORTANTE: No elimine esta linea
c_ej = [0,5,5,5,5]      # IMPORTANTE: No elimine esta linea
T_ej = [400]               # IMPORTANTE: No elimine esta linea

''' IMPORTANTE: Complete aqui las funciones LecturaDatosTransporte() y ModeloTransporte(), ... que lean los datos desde el archivo  "datos-Transporte.txt" y luego
generen el modelo de Transporte asociado a dichos datos (tal como en LecturaDatosEjemplo(), ModeloEjemplo())
Dentro de cada funcion no hay mayores limitaciones respecto a python, lo unico importante es no cambiar el nombre de cada funcion y lo que estas funciones retornan
'''
# IMPORTANTE: Complete esta funcion de forma de que lea los datos del archivo "datos-Transporte.txt" y los guarde en las listas t, p, d, c y T. 
# Un ejemplo de como hacer esto se encuentra en LecturasDatosEjemplo(), linea 50 de este codigo.
# NOTA: Puede borrar todo el contenido de esta funcion si lo desea, excepto las lineas expresamente indicadas
def LecturaDatosTransporte(): # No modifique esta linea. 
    
    nombre_archivo =  "datos-Transporte.txt"    # IMPORTANTE: Recuerde no cambiar el nombre del archivo a leer.
    try:
        with open (nombre_archivo, 'r') as archivo:
            lineas = archivo.readlines()     
    except IOError:
        print ("ERROR: archivo ", nombre_archivo, " fue eliminado del directorio")   

    # Aqui debe leer t, p, d c y T de las 5 lineas que contiene el archivo datos-Transporte.txt.
    
    t = lineas[0].strip().split(" ")
    
    for i in range(len(t)):
      t[i] = int(t[i])
    
    p = lineas[1].strip().split(" ")
    
    for i in range(len(p)):
      p[i] = int(p[i])
    
    d = lineas[2].strip().split(" ")
    
    for i in range(len(d)):
      d[i] = int(d[i])
    
    c = lineas[3].strip().split(" ")
    
    for i in range(len(c)):
      c[i] = int(c[i])
    
    T = lineas[4].strip().split(" ")
    
    for i in range(len(T)):
      T[i] = int(T[i])

    return t, p, d, c, T # No modifique esta linea. 


def ModeloTransporte(): # No modifique esta linea
  model = LpProblem('modeloTransporte', LpMaximize)
  t, p, d, c, T = LecturaDatosTransporte()   # No modifique esta linea
  # cree aqui su modelo que use los datos F, h, c y d.
  
  cant_tiendas = len(t)
  
  cant_productos = []
  cant_demanda = []
  
  for i in range(cant_tiendas):
    cant_productos.append(f"x{i}")
    cant_demanda.append(f"s{i}")
  
  x = LpVariable.dicts("x", cant_productos, lowBound=0, cat=const.LpContinuous)
  s = LpVariable.dicts("s", cant_demanda, lowBound=0, cat=const.LpContinuous)
  
  model += lpSum([x[cant_productos[i]] * t[i] + s[cant_demanda[i]] * p[i] for i in range(cant_tiendas)]), "Costos de transporte"
  
  for i in range(cant_tiendas):
    
    model += x[cant_productos[i]] + s[cant_demanda[i]] == d[i]
    
    model += s[cant_demanda[i]] <= c[i]
  
  model += lpSum(x) <= T
  
  model += x[cant_productos[1]] >= x[cant_productos[2]]
  
  return model   # No modifique esta linea (salvo si quiere renombrar la variable que contiene su modelo)

# Estas dos funciones son de ejemplo
def LecturaDatosEjemplo():
    nombre_archivo = "datos-ejemplo.txt"
    try:
        with open (nombre_archivo, 'r') as archivo:
            linea = archivo.readlines()     
    except IOError:
        print ("ERROR: archivo ", nombre_archivo, " fue eliminado del directorio")

    datos = linea[0].split(" ")
    W = int(datos[0])
    n = int(datos[1])
    return W, n

def ModeloEjemplo():
  # Si no quiere ver el output de este modelo de ejemplo, no elimine la funcion, pero si comente el codigo que define al modelo.
  model = LpProblem('ModeloEjemplo', LpMaximize)
  W,n = LecturaDatosEjemplo()  # aqui se leen los datos del modelo desde el archivo datos-ejemplo.txt
  indices = range(1,n+1,1) # aqui se crea una lista de 1,2,3...n, que representa al conjunto de n objetos
  x = LpVariable.dicts("x", indices, 0, None, LpBinary)
  #Creacion funcion objetivo 
  model += lpSum([x[i] for i in indices]) , "Costo Total"
  model += lpSum([i * x[i] for i in indices]) <= W, "Peso"  # aqui se usa el valor de W leido desde el archivo
  model.solve()
  return model



# IMPORTANTE: De aqui en adelante, no debiese ser necesario modificar ninguna otra linea de codigo."

# IMPORTANTE: NO MODIFIQUE ESTA FUNCION. Esta funcion resume la entrega del alumno. Si su tarea esta completa, no debe haber mensajes de error
import itertools
import random
def verificador():
  if NOMBRE == "Alejandro" and APELLIDO == "Martinez":
    print("OPTI: ERROR Edite valor de variables NOMBRE y APELLIDO. Revise instrucciones de tarea")
    return  

  print("OPTI: Obteniendo y resolviendo los modelos" )
  functions = [ModeloEjemplo, ModeloTransporte]
  funcNames = list(map (lambda func: func.__name__, functions))  

  try:
    models = list(map (lambda func: func(), functions))
  except BaseException:    
    print ("OPTI: ERROR en alguna funcion que debe implementar. Mensaje de error de python:")
    print(traceback.format_exc())
    return 

  for model in models:
     if isinstance (model,PLP.LpProblem):
        try:
            model.solve(PLP.PULP_CBC_CMD(msg=0))
        except:
            print("OPTI: ERROR: Alguno de los modelos no se pudo resolver")
            return

  print ()
  print("OPTI: INICIO VERIFICACION. Si su tarea esta completa, no debe haber ERROR hasta que aparezca FIN VERIFICACION " )
  print("OPTI: NOMBRE Y APELLIDO ALUMNO :", NOMBRE, " ", APELLIDO)

  t, p, d, c, T  = LecturaDatosTransporte()
  if t == t_ej or p == p_ej or d == d_ej or c == c_ej or T==T_ej:
    print("OPTI: ERROR: La lectura de archivos de datos no ha sido implementada. Complete la funcion Lectura_datos-Transporte de forma que retorne los datos t, p, d, c, T desde el archivo datos-Transporte.txt")
    nombre_archivo =  "datos-Transporte.txt"
    try:
        with open (nombre_archivo, 'r') as archivo:
            lineas = archivo.readlines()     
    except IOError:
        print ("OPTI: ERROR: archivo ", nombre_archivo, " fue eliminado del directorio")   
    #print (lineas)

  for idx in range(len(functions)):
    funcname = funcNames[idx] 

    model = models[idx]    
    if isinstance (model,PLP.LpProblem) and model.objective is not None:      
      status = PLP.LpStatus[model.status]      
      if status == PLP.LpStatus[1]:     
        obj = model.objective
        value = PLP.value(obj)
        print ("OPTI: Modelo ", funcname, "tiene optimo finito, su valor es: ", value)
      elif status == PLP.LpStatus[-1]:
        print ("OPTI: Modelo ", funcname, " es infactible (no tiene solucion)")
      elif status == PLP.LpStatus[-2]:    
        print ("OPTI: Modelo ", funcname, "es no acotado (optimo es +infinito o -infinito)")
      else:
        print ("OPTI: ERROR en ", funcname, ": model.solve() no entrega una solucion. Es probable que el modelo este mal implementado")
    else:
        print("OPTI: ERROR en ", funcname, ": Complete la funcion ", funcNames[idx], "() y asegurese de que retorne el modelo")

  
  print("OPTI: FIN VERIFICACION")



# IMPORTANTE: No modifique esta seccion. 
import importlib
module_spec = importlib.util.find_spec('check')
if module_spec is not None:
  module = importlib.util.module_from_spec(module_spec)
  module_spec.loader.exec_module(module)
  verificador = module.verificador


# IMPORTANTE: No modifique esta seccion.
def main():
  try:
    verificador() # Puede comentar esta linea si lo desea. Pero al terminar su tarea, debe chequear que su tarea funcione .
  except:
    print ("OPTI: ERROR Verificacion no se ejecuto correctamente.")
if __name__ == "__main__":
  main()