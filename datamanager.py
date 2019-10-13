import json
import numpy as np

class DataManager():
    def __init__(self):
        self.data_x = []
        self.data_z = []
        self.n_data = 0

    def push(self, x, z):
        if self._isValidInput(x):
            self.data_x.append(x)
            self.data_z.append(z)
            self.n_data += 1
            print "pushed"
            print x
    
    def dump(self, filename = "tmp.json"):
        x_list = [[elem[0], elem[1]] for elem in self.data_x]
        data = {"X": x_list, "Z": self.data_z}
        json_file = open(filename, "w")
        json.dump(data, json_file)
        json_file.close()

    def dumps(self): 
        if self.n_data == 0:
            return ""

        x_list = [[elem[0], elem[1]] for elem in self.data_x]
        data = {"X": x_list, "Z": self.data_z}
        return json.dumps(data)

    def load(self, filename = "tmp.json"):
        json_file = open(filename, "r")
        json_object = json.load(json_file)
        X = json_object["X"]
        Z = json_object["Z"]
        self.data_x = [np.array(e) for e in X]
        self.data_z = Z

    def loads(self, json_string):
        json_object = json.loads(json_string)
        X = json_object["X"]
        Z = json_object["Z"]
        self.data_x = [np.array(e) for e in X]
        self.data_z = Z

    def show(self):
        x_list, y_list = [[e[i] for e in self.data_x] for i in [0, 1]]
        plt.scatter(x_list, y_list)

    def _isValidInput(self, x, tau = 0.5):
        if self.n_data == 0:
            return True
        x0_vec = np.array([elem[0] for elem in self.data_x])
        x1_vec = np.array([elem[1] for elem in self.data_x])

        dists = np.sqrt((x0_vec - x[0]) ** 2 + (x1_vec - x[1]) ** 2)
        dist_min = dists.min()
        isValid = (dist_min > tau)
        return isValid
