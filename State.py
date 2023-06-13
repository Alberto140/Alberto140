import hashlib


class State():

    def __init__(self, id:int, nodes:list(), md5:str):
        self.id = id
        self.nodesToVisit = nodes
        self.md5 = md5
        #print(self.nodesToVisit)
        
        idToString = 205
        listoToString = [1185,1252,1314]
        string = " "
       
        idToString = str(idToString)
        listoToString = [str(x) for x in listoToString]
        listoToString = ",".join(listoToString)
        string = idToString + listoToString
       
        self.md5Id = hashlib.md5(string.encode()).hexdigest()
        #print(self.md5Id)
          
    def set_nodesToVisit(self, lista:list()):
        self.nodesToVisit = lista
    
    def get_nodesToVisit(self):
        return self.nodesToVisit
    
    def get_stateId(self):
        return self.id

    def get_md5(self):
        return self.md5

    def get_object(self):
        return self.id, self.nodesToVisit, self.md5[-6:]

    def calculate_md5(self, id, nodes):
        if len(nodes) > 0:
            listoToString = [str(x) for x in nodes]
            listoToString = ",".join(listoToString)
            string = "({},[{}])".format(id, listoToString)
        else:
            string = "({},[])".format(id)

        return hashlib.md5(string.encode()).hexdigest()
    

    