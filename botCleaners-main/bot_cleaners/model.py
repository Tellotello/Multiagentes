from mesa.model import Model
from mesa.agent import Agent
from mesa.space import MultiGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector
import random

import numpy as np


class Celda(Agent):
    def __init__(self, unique_id, model, suciedad: bool = False):
        super().__init__(unique_id, model)
        self.sucia = suciedad


class Mueble(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class Cargador(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.carga = 100

class RobotLimpieza(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.sig_pos = None
        self.movimientos = 0
        self.carga = 100
        self.available_neighbors = []
        self.recargas = 0

    
    def limpiar_una_celda(self, lista_de_celdas_sucias): # Selecciona una celda y actualiza la celda a limpar para el conocimiento de los demas robots
        celdas_preferidas = [celda for celda in lista_de_celdas_sucias if celda.pos not in self.model.celdas_objetivo]
        if celdas_preferidas:
            celda_a_limpiar = self.random.choice(celdas_preferidas)
        else:
            celda_a_limpiar = self.random.choice(lista_de_celdas_sucias)
        
        if celda_a_limpiar.pos not in self.model.celdas_objetivo:
            self.model.celdas_objetivo.append(celda_a_limpiar.pos)
        
        celda_a_limpiar.sucia = False
        self.sig_pos = celda_a_limpiar.pos

        if celda_a_limpiar.pos in self.model.celdas_objetivo:
            self.model.celdas_objetivo.remove(celda_a_limpiar.pos)

 
    def seleccionar_nueva_pos(self, lista_de_vecinos): # Selecciona una nueva posicion para el robot  
        vecinos_sin_muebles = [vecino for vecino in lista_de_vecinos if not isinstance(vecino, Mueble) and not isinstance(vecino, Cargador)] # Selecciona los vecinos que no son muebles
        vecinos = self.model.grid.get_neighbors(
            self.pos, moore=True, include_center=False)

        if self.carga < 20: # Si la carga es menor a 20, busca un cargador
            cargadores = self.model.get_posiciones_cargadores() # Obtiene las posiciones de los cargadores
            if cargadores: # Si hay cargadores
                min_distance = float('inf') # Se inicializa la distancia minima como infinito
                nearest_cargador = None # Se inicializa el cargador mas cercano como None

                for cargador in cargadores:  # Se recorren los cargadores
                    distance = abs(self.pos[0] - cargador[0]) + abs(self.pos[1] - cargador[1]) # Se calcula la distancia entre el robot y el cargador para elegir el que tenga menos distancia
                    if distance < min_distance: # Si la distancia es menor a la minima distancia
                        min_distance = distance # Se actualiza la minima distancia
                        nearest_cargador = cargador # Se actualiza el cargador mas cercano
            
                if nearest_cargador: # Despues de que ya se sabe cual es el cargador mas cercano

                    next_x = self.pos[0] + (1 if nearest_cargador[0] > self.pos[0] else -1) # Se calcula la siguiente posicion en x porque se sabe la posicion final, pero no el camino
                    next_y = self.pos[1] + (1 if nearest_cargador[1] > self.pos[1] else -1)

                    next_x = max(0, min(self.model.grid.width - 1, next_x)) # Se asegura que la siguiente posicion este dentro de la grid
                    next_y = max(0, min(self.model.grid.height - 1, next_y))

                    self.sig_pos = (next_x, next_y) # Se actualiza la siguiente posicion
                    print("BUSCANDO CARGADOR") # Se imprime que se esta buscando un cargador
                    print("SIG POS PARA CARGADOR: ", self.sig_pos) # Se imprime la siguiente posicion
                    return
    


        if vecinos_sin_muebles: # Si hay vecinos sin muebles
            self.sig_pos = self.random.choice(vecinos_sin_muebles).pos
        else:
            self.sig_pos = self.pos

        sucias = self.model.get_all_dirty_cells() # Obtiene todas las celdas sucias para escoger la sucia mas cerca a la cual ir

        if sucias: # Si hay celdas sucias, se repite la misma logica que con los cargadores pero con sucias
            min_distance = float('inf')
            nearest_dirty_cell = None

            for celda in sucias: # Se recorren todas las posiciones de las sucias
                distance = abs(self.pos[0] - celda.pos[0]) + abs(self.pos[1] - celda.pos[1]) # Se calcula la distancia entre el robot y la celda sucia
                if distance < min_distance: # Si la distancia es menor a la minima distancia
                    min_distance = distance # Se actualiza la minima distancia
                    nearest_dirty_cell = celda # Se actualiza la celda sucia mas cercana

            if nearest_dirty_cell: # Despues de que ya se sabe cual es la celda sucia mas cercana
                next_x = self.pos[0] + (1 if nearest_dirty_cell.pos[0] > self.pos[0] else -1) # Se calcula la siguiente posicion en x porque se sabe la posicion final, pero no el camino
                next_y = self.pos[1] + (1 if nearest_dirty_cell.pos[1] > self.pos[1] else -1)

                next_x = max(0, min(self.model.grid.width - 1, next_x)) # Se asegura que la siguiente posicion este dentro de la grid
                next_y = max(0, min(self.model.grid.height - 1, next_y))

                self.sig_pos = (next_x, next_y)
                print("SIG POS: ", self.sig_pos)



        


    
    

        
    

    @staticmethod
    def buscar_celdas_sucia(lista_de_vecinos):
        # #Opción 1
        return [vecino for vecino in lista_de_vecinos
        if isinstance(vecino, Celda) and vecino.sucia]
       
        

    def initialize_available_neighbors(self): # Inicializa los vecinos disponibles
        neighbors = self.model.grid.get_neighbors(
            self.pos, moore=True, include_center=False)
        self.available_neighbors = [
            vecino for vecino in neighbors if not isinstance(vecino, Mueble) and not isinstance(vecino, Cargador)
        ]



    def step(self): # Se ejecuta cada vez que se llama al metodo step del modelo
        vecinos = self.model.grid.get_neighbors(
            self.pos, moore=True, include_center=False)

        vecinos_sin_muebles = [vecino for vecino in vecinos if not isinstance(vecino, Mueble)] # Lista con los vecinos que no son muebles
        vecinos = vecinos_sin_muebles 

        for vecino in vecinos: # Se recorren los vecinos
            if isinstance(vecino, (Mueble, RobotLimpieza)):
                vecinos.remove(vecino) # Si el vecino es un mueble o un robot, se elimina de la lista de vecinos
            if isinstance(vecino,Cargador) and self.carga < 15: # Para recargar. 
                # Se tiene que los robots solo van a recargas su bateria cuando esta sea menor a 15.
                # Esto es porque apesar de que están cerca de un cargador, puede ser que no necesite recargar
                # Si asumimos que cada vez que se hace una recarga se usa la misma cantidad de energía, entonces 
                # Solo deberian recargarse cuando tengan bateria baja para maximizar la eficiencia de la energia
                self.carga = 100
                self.recargas += 1 # Se actualiza el numero de recargas


        if self.carga < 20: # Si la carga es menor a 20, busca un cargador
            cargadores = [vecino for vecino in vecinos if isinstance(vecino, Cargador)]
            if cargadores:
                self.sig_pos = self.random.choice(cargadores).pos
            else:
                self.initialize_available_neighbors()
                self.seleccionar_nueva_pos(self.available_neighbors)
        else:

            clean_neighbors = [ # Selecciona los vecinos que no son muebles ni celdas sucias
                neighbor for neighbor in self.available_neighbors if not isinstance(neighbor, Celda) or not neighbor.sucia
            ]

            if len(clean_neighbors) > 0: # Si hay vecinos que no son muebles ni celdas sucias
                self.seleccionar_nueva_pos(clean_neighbors)
            else:
                self.seleccionar_nueva_pos(self.available_neighbors)

            celdas_sucias = self.buscar_celdas_sucia(vecinos) # Se buscan las celdas sucias

            if len(celdas_sucias) == 0:
                self.seleccionar_nueva_pos(vecinos)
            else:
                self.limpiar_una_celda(celdas_sucias)

    def advance(self):
        if self.pos != self.sig_pos:
            self.movimientos += 1

        if self.carga > 0:
            self.carga -= 1
            self.model.grid.move_agent(self, self.sig_pos)


class Habitacion(Model):
    def __init__(self, M: int, N: int,
                 num_agentes: int = 5,
                 porc_celdas_sucias: float = 0.6,
                 porc_muebles: float = 0.1,
                 modo_pos_inicial: str = 'Fija',
                 

                
                 ):
        
        self.celdas_objetivo = []

        self.num_agentes = num_agentes
        self.porc_celdas_sucias = porc_celdas_sucias
        self.porc_muebles = porc_muebles

        self.grid = MultiGrid(M, N, False)
        self.schedule = SimultaneousActivation(self)

        posiciones_disponibles = [pos for _, pos in self.grid.coord_iter()]

        # Se intentan esparcir los cargadores lo mas posible
        third_width = M // 3
        two_thirds_width = 2 * M // 3
        third_height = N // 3
        two_thirds_height = 2 * N // 3

        # Se tienen 4 posiciones de cargadores
        posiciones_cargadores = [
            (third_width, third_height),             
            (two_thirds_width, third_height),       
            (third_width, two_thirds_height),       
            (two_thirds_width, two_thirds_height)   
        ]



        for pos in posiciones_cargadores:
            posiciones_disponibles.remove(pos)

        # Posicionamiento de muebles
        num_muebles = int(M * N * porc_muebles)
        posiciones_muebles = self.random.sample(posiciones_disponibles, k=num_muebles)
        
        numCargadores = 4
        posiciones_cargadores = self.random.sample(posiciones_disponibles, k=numCargadores)

        for id, pos in enumerate(posiciones_muebles):
            mueble = Mueble(int(f"{num_agentes}0{id}") + 1, self)
            self.grid.place_agent(mueble, pos)
            posiciones_disponibles.remove(pos)

        # Posicionamiento de celdas sucias
        self.num_celdas_sucias = int(M * N * porc_celdas_sucias)
        posiciones_celdas_sucias = self.random.sample(
            posiciones_disponibles, k=self.num_celdas_sucias)

        for id, pos in enumerate(posiciones_disponibles):
            suciedad = pos in posiciones_celdas_sucias
            celda = Celda(int(f"{num_agentes}{id}") + 1, self, suciedad)
            self.grid.place_agent(celda, pos)

        # Posicionamiento de agentes robot
        if modo_pos_inicial == 'Aleatoria':
            pos_inicial_robots = self.random.sample(posiciones_disponibles, k=num_agentes)
        else:  # 'Fija'
            pos_inicial_robots = [(1, 1)] * num_agentes

        for id in range(num_agentes):
            robot = RobotLimpieza(id, self)
            self.grid.place_agent(robot, pos_inicial_robots[id])
            self.schedule.add(robot)

        # Posicionamiento de cargadores
        
        for id, pos in enumerate(posiciones_cargadores.copy()): 
            cargador = Cargador(int(f"{num_agentes}0{id}") + 1, self)
            self.grid.place_agent(cargador,pos)
            self.schedule.add(cargador)
            posiciones_cargadores.remove(pos)

        
       

        self.celdas_sucias = []
        self.datacollector = DataCollector(
            model_reporters={"Grid": get_grid, "Cargas": get_cargas,
                             "CeldasSucias": get_sucias},
        )

    def get_all_dirty_cells(self): # Obtiene una lista de todas las celdas sucias
        dirty_cells = [] 
        for (content, _) in self.grid.coord_iter():
            for obj in content:
                if isinstance(obj, Celda) and obj.sucia:
                    dirty_cells.append(obj)
        return dirty_cells


    def get_posiciones_cargadores(self): # Obtiene una lista de las posiciones de los cargadores
        cargadores_pos = []
        for (content, _) in self.grid.coord_iter():
            for obj in content:
                if isinstance(obj, Cargador):
                    cargadores_pos.append(obj.pos)
        return cargadores_pos
    def step(self): # Se ejecuta cada vez que se llama al metodo step del modelo
        sucias_actual = [celda.pos for celda in self.get_all_dirty_cells()]
        self.celdas_objetivo = [pos for pos in self.celdas_objetivo if pos in sucias_actual] # Se actualizan las celdas objetivo

        self.datacollector.collect(self)
        self.schedule.step()

    def todoLimpio(self):
        for (content, pos) in self.grid.coord_iter():
            x,y = pos
            for obj in content:
                if isinstance(obj, Celda) and obj.sucia:
                    return False
        return True

    def model_get_sucias(self):
        return self.celdas_sucias


def get_grid(model: Model) -> np.ndarray:
    """
    Método para la obtención de la grid y representarla en un notebook
    :param model: Modelo (entorno)
    :return: grid
    """
    grid = np.zeros((model.grid.width, model.grid.height))
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        x, y = pos
        for obj in cell_content:
            if isinstance(obj, RobotLimpieza):
                grid[x][y] = 2
            elif isinstance(obj, Celda):
                grid[x][y] = int(obj.sucia)
    return grid


def get_cargas(model: Model):
    return [(agent.unique_id, agent.carga) for agent in model.schedule.agents]


def get_sucias(model: Model) -> int:
    """
    Método para determinar el número total de celdas sucias
    :param model: Modelo Mesa
    :return: número de celdas sucias
    """
    sum_sucias = 0
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        for obj in cell_content:
            if isinstance(obj, Celda) and obj.sucia:
                sum_sucias += 1
    return sum_sucias / model.num_celdas_sucias


def get_movimientos(agent: Agent) -> dict:
    if isinstance(agent, RobotLimpieza):
        return {agent.unique_id: agent.movimientos}
    # else:
    #    return 0


# Prueba de modelo
M, N = 20, 20  #  Prueba con 20 y 20
model = Habitacion(M, N)

steps = 0
while not model.todoLimpio():
    model.step()
    steps += 1

movimientos_totales = sum([agent.movimientos for agent in model.schedule.agents if isinstance(agent, RobotLimpieza)])
recargas_totales = sum([agent.recargas for agent in model.schedule.agents if isinstance(agent, RobotLimpieza)])

print(f"Tiempo necesario para limpiar todas las celdas sucias: {steps} pasos")
print(f"Numero total de movimientos necesarios para todos los agentes: {movimientos_totales}")
print(f"Numero total de veces que los robots tuvieron que recargar: {recargas_totales}")
