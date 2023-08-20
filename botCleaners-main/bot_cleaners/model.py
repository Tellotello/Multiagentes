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

    def limpiar_una_celda(self, lista_de_celdas_sucias):
        # Prioritize cells not in shared knowledge base
        celdas_preferidas = [celda for celda in lista_de_celdas_sucias if celda.pos not in self.model.celdas_objetivo]
        if celdas_preferidas:
            celda_a_limpiar = self.random.choice(celdas_preferidas)
        else:
            celda_a_limpiar = self.random.choice(lista_de_celdas_sucias)
        
        if celda_a_limpiar.pos not in self.model.celdas_objetivo:
            self.model.celdas_objetivo.append(celda_a_limpiar.pos)
        
        celda_a_limpiar.sucia = False
        self.sig_pos = celda_a_limpiar.pos
        # Update shared knowledge base after cleaning
        if celda_a_limpiar.pos in self.model.celdas_objetivo:
            self.model.celdas_objetivo.remove(celda_a_limpiar.pos)

 
    def seleccionar_nueva_pos(self, lista_de_vecinos):
        vecinos_sin_muebles = [vecino for vecino in lista_de_vecinos if not isinstance(vecino, Mueble) and not isinstance(vecino, Cargador)]
        vecinos = self.model.grid.get_neighbors(
            self.pos, moore=True, include_center=False)

        if self.carga < 20:
            cargadores = self.model.get_posiciones_cargadores()
            if cargadores:
                min_distance = float('inf')
                nearest_cargador = None

                for cargador in cargadores:
                    distance = abs(self.pos[0] - cargador[0]) + abs(self.pos[1] - cargador[1])
                    if distance < min_distance:
                        min_distance = distance
                        nearest_cargador = cargador
            
                if nearest_cargador:

                    next_x = self.pos[0] + (1 if nearest_cargador[0] > self.pos[0] else -1)
                    next_y = self.pos[1] + (1 if nearest_cargador[1] > self.pos[1] else -1)

                    next_x = max(0, min(self.model.grid.width - 1, next_x))
                    next_y = max(0, min(self.model.grid.height - 1, next_y))

                    self.sig_pos = (next_x, next_y)
                    print("BUSCANDO CARGADOR")
                    print("SIG POS PARA CARGADOR: ", self.sig_pos)
                    return
    


        if vecinos_sin_muebles:
            self.sig_pos = self.random.choice(vecinos_sin_muebles).pos
        else:
            self.sig_pos = self.pos

        sucias = self.model.get_all_dirty_cells()

        if sucias:
            min_distance = float('inf')
            nearest_dirty_cell = None

            for celda in sucias:
                distance = abs(self.pos[0] - celda.pos[0]) + abs(self.pos[1] - celda.pos[1])
                if distance < min_distance:
                    min_distance = distance
                    nearest_dirty_cell = celda

            if nearest_dirty_cell:
                # Calculate the next position based on the current position
                next_x = self.pos[0] + (1 if nearest_dirty_cell.pos[0] > self.pos[0] else -1)
                next_y = self.pos[1] + (1 if nearest_dirty_cell.pos[1] > self.pos[1] else -1)

                # Ensure the next position is within grid bounds
                next_x = max(0, min(self.model.grid.width - 1, next_x))
                next_y = max(0, min(self.model.grid.height - 1, next_y))

                self.sig_pos = (next_x, next_y)
                print("SIG POS: ", self.sig_pos)



        


    
    

        
    

    @staticmethod
    def buscar_celdas_sucia(lista_de_vecinos):
        # #Opción 1
        return [vecino for vecino in lista_de_vecinos
        if isinstance(vecino, Celda) and vecino.sucia]
       
        

    def initialize_available_neighbors(self):
        neighbors = self.model.grid.get_neighbors(
            self.pos, moore=True, include_center=False)
        self.available_neighbors = [
            vecino for vecino in neighbors if not isinstance(vecino, Mueble) and not isinstance(vecino, Cargador)
        ]



    def step(self):
        vecinos = self.model.grid.get_neighbors(
            self.pos, moore=True, include_center=False)

        vecinos_sin_muebles = [vecino for vecino in vecinos if not isinstance(vecino, Mueble)]
        vecinos = vecinos_sin_muebles  # Update the vecinos list to exclude

        for vecino in vecinos:
            if isinstance(vecino, (Mueble, RobotLimpieza)):
                vecinos.remove(vecino)
            if isinstance(vecino,Cargador):
                self.carga = 100


        if self.carga < 20:
            cargadores = [vecino for vecino in vecinos if isinstance(vecino, Cargador)]
            if cargadores:
                self.sig_pos = self.random.choice(cargadores).pos
            else:
                self.initialize_available_neighbors()
                self.seleccionar_nueva_pos(self.available_neighbors)
        else:

            clean_neighbors = [
                neighbor for neighbor in self.available_neighbors if not isinstance(neighbor, Celda) or not neighbor.sucia
            ]

            if len(clean_neighbors) > 0:
                self.seleccionar_nueva_pos(clean_neighbors)
            else:
                self.seleccionar_nueva_pos(self.available_neighbors)

            celdas_sucias = self.buscar_celdas_sucia(vecinos)

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
     
        quad_width = M // 2
        quad_height = N // 2

        
        posiciones_cargadores = [
            (quad_width // 2, quad_height // 2),                    # Top-left quadrant
            (quad_width + quad_width // 2, quad_height // 2),       # Top-right quadrant
            (quad_width // 2, quad_height + quad_height // 2),      # Bottom-left quadrant
            (quad_width + quad_width // 2, quad_height + quad_height // 2)  # Bottom-right quadrant
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

    def get_all_dirty_cells(self):
        dirty_cells = []
        for (content, _) in self.grid.coord_iter():
            for obj in content:
                if isinstance(obj, Celda) and obj.sucia:
                    dirty_cells.append(obj)
        return dirty_cells


    def get_posiciones_cargadores(self):
        cargadores_pos = []
        for (content, _) in self.grid.coord_iter():
            for obj in content:
                if isinstance(obj, Cargador):
                    cargadores_pos.append(obj.pos)
        return cargadores_pos
    def step(self):
        # Update the shared knowledge base
        sucias_actual = [celda.pos for celda in self.get_all_dirty_cells()]
        self.celdas_objetivo = [pos for pos in self.celdas_objetivo if pos in sucias_actual]

        self.datacollector.collect(self)
        self.schedule.step()

    def todoLimpio(self):
        for (content, x, y) in self.grid.coord_iter():
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
