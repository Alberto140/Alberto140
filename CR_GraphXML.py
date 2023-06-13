from os import stat
import reader
import State
import TreeNode
import math
import re
from queue import PriorityQueue

class CR_GraphXML():
    
    def __init__(self, g:dict):
        self.graph = g
        self.min_distance12 = float('inf')
        
    def succ(self, state:State, lista:list,depth:int):
        childs = []
        adyList = sorted(lista)
        nodesToVisit = state.get_nodesToVisit()

        i = 0
        nodes = list()
        for adyNode in adyList:
            if adyNode not in nodes:
                nodes.append(adyNode)
            else:
                childs.pop(i-1)
            if adyNode in nodesToVisit:
                nodesList = nodesToVisit.copy()
                nodesList.remove(adyNode)
                md5 = State.State.calculate_md5(self, adyNode, nodesList)
                states = State.State(adyNode, nodesList, md5)
                action = "{}->{}".format(state.id, adyNode)
                childs.append([float(reader.getCost(state.id, adyNode)), states, action])
            else:
                md5 = State.State.calculate_md5(self, adyNode, nodesToVisit)
                action = "{}->{}".format(state.id, adyNode)
                childs.append([float(reader.getCost(state.id, adyNode)), State.State(adyNode, nodesToVisit, md5), action])

            i += 1

        return childs

    def heuristica_longitud_arista(self, state: State, inicial):
        # Obtener la longitud mínima de la arista del grafo (A1)
        min_edge_length = reader.getMinimumLength()

        # Calcular la heurística Hedge
        hedge_heuristic = min_edge_length * len(state.get_nodesToVisit())

        return hedge_heuristic
    
    def heuristica_euclidiana(self, state: State, inicial):
        nodesToVisit = state.get_nodesToVisit()
        if not nodesToVisit:
            return 0
        
        node = state.get_stateId()
        x_node, y_node = reader.getNodeCoordinates(node)

        min_distance = float('inf')

        for v in nodesToVisit:
            x_v, y_v = reader.getNodeCoordinates(v)
            distance = math.sqrt((x_node - x_v) ** 2 + (y_node - y_v) ** 2)
            min_distance = min(min_distance, distance)
        if inicial == 1:
            for i in range(len(nodesToVisit)):
                for j in range(len(nodesToVisit)):
                    if nodesToVisit[i] != nodesToVisit[j]:
                        x1_v, y1_v = reader.getNodeCoordinates(nodesToVisit[i])
                        x2_v, y2_v = reader.getNodeCoordinates(nodesToVisit[j])
                        distance = math.sqrt((x1_v - x2_v) ** 2 + (y1_v - y2_v) ** 2)
                        self.min_distance1 = min(self.min_distance1, distance)

        return min(min_distance, self.min_distance1) * len(nodesToVisit)


    def search(self, graph, initialState:State, strategy, max_depth):  # function for all the strategies
        visited_Id = list()  # List for visited nodes.
        visited = list()
        frontier = list()  # Initialize a queue
        if strategy == "B":
            value = 0
        elif strategy == "D":
            value = 1
        elif strategy == "U":
            value = 0
        elif strategy == "G":
            value = self.heuristica_longitud_arista(initialState, 1)
        elif strategy == "A":
            value = self.heuristica_longitud_arista(initialState, 1)
        else:
            exit("Strategy not correct")
        node = TreeNode.Node(0,0.00,initialState,None,None,0,self.heuristica_longitud_arista(initialState, 1),value)
        #print(node.ToString())
        visited.append(node)
        frontier.append(node)
        Ids = 0
        solution = False
        while frontier and not solution:  # Creating loop to visit each node
            Id = Ids
            m = frontier.pop(0)

            if m.depth < max_depth and m.state.get_md5() not in visited_Id:
                if len(m.state.get_nodesToVisit()) == 0 and m.state.get_stateId() in initialState.nodesToVisit:
                    # print(node.ToString())
                    CR_GraphXML.printSolution(self, visited, m)
                    solution = True

                else:
                    #print(m.ToString())
                    successors = self.succ(m.state, graph[m.state.get_stateId()],6)
                    for x in range(len(successors)):
                        Id = Id + 1
                        if strategy == "B":
                            value = m.depth + 1
                        elif strategy == "D":
                            value = (1/(m.depth + 2))
                        elif strategy == "U":
                            value = successors[x][0] + m.cost
                        elif strategy == "G":
                            value = self.heuristica_longitud_arista(successors[x][1], 0)
                        elif strategy == "A":
                            value = successors[x][0] + m.cost + self.heuristica_longitud_arista(successors[x][1], 0)
                        node = TreeNode.Node(Id, successors[x][0] + m.cost, successors[x][1], m.get_NodeId(), successors[x][2], m.depth + 1, self.heuristica_longitud_arista(successors[x][1], 0), value)
                        #print(node.ToString())
                        frontier.append(node)
                        visited.append(node)

                    frontier = sorted(frontier, key=lambda node : (node.value, node.id_node))
                    visited_Id.append(m.state.get_md5())
                    Ids = Id




    def printSolution(self, visited:list(), node:TreeNode.Node):
        if node.id_node != 0:
            self.printSolution(visited, visited[node.parent])
        print(node.ToString())

    def run(self):
        self.path = [310,772,1155]
        self.path = sorted(self.path, key=abs)
        if not self.path:
            print("Not a valid initial state, the list is empty")
        else:
            md5 = State.State.calculate_md5(self, 1177, self.path)
            self.initialState = State.State(1177, self.path, md5)

        self.search(self.graph, self.initialState, "U", 109) # The strategies are (B,D,G,U,A)
        
    
if __name__ == '__main__':
    
    graph = dict()
    graph = reader.passGraph()
    #reader.printGraph(graph)
    
    problem = CR_GraphXML(graph)
    problem.run()
    print('exit')