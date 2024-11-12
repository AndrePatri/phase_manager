from horizon.problem import Problem
import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase
import phase_manager.pytimeline as pytimeline
import time
import numpy as np

ns = 15
dt = 0.01
prb = Problem(ns, receding=True)
prb.setDt(dt)

class FakeItem:
    def __init__(self, par, weight):

        self.name = 'fake_item'
        self.ref = par
        self.nodes = list(range(50))
        self.weights = weight

    def getDim(self):

        return self.ref.getDim()
    def getName(self):

        return self.name

    def setNodes(self, nodes, erasing=True):

        self.nodes = nodes

    def assign(self, val, nodes=None):

        # print(f'    fake item: calling "assing" with values: \n {val} \n at nodes {nodes}')
        # necessary method for using this task as an item + reference in phaseManager
        self.ref.assign(val, nodes)
        return 1

    def getValues(self):
        # necessary method for using this task as an item + reference in phaseManager
        return self.ref.getValues()

    def getWeight(self):
        # necessary method for using this task as an item + reference in phaseManager
        return self.weights.getValues()

    def setWeight(self, val, nodes=None):
        # necessary method for using this task as an item + reference in phaseManager
        self.weights.assign(val, nodes)


a = prb.createStateVariable('a', 1)
par = prb.createParameter('par', 3)
weight = prb.createParameter('weight', 1)

# print('initial values: \n', par.getValues())
# print('initial nodes: \n', par.getNodes())

fake_item = FakeItem(par, weight)
pm = pymanager.PhaseManager(ns)
timeline_1 = pm.createTimeline('timeline_1')
phase_1 = timeline_1.createPhase(5, 'phase_1')

traj_1 = np.array([[1, 2, 3, 4, 5],
                   [1, 2, 3, 4, 5],
                   [1, 2, 3, 4, 5]])

phase_1.addItemReference(fake_item, traj_1)

weights_1 = np.array([[0.1, 0.1, 0.5, 0.1, 0.1]])

phase_1.addItemWeight(fake_item, weights_1)

timeline_1.addPhase(phase_1)

print('values after adding phase: \n', par.getValues())
# print('nodes after adding phase: \n', par.getNodes())
print('weights after adding phase: \n', weight.getValues())


# print('==========================================================')
for i in range(2):
    pm.shift()
    print('values after shifting: \n', par.getValues())
    # print('nodes after adding phase: \n', par.getNodes())
    print('weights after adding phase: \n', weight.getValues())

print("modifying phase: ")
timeline_1.getPhases()[0].setItemReference('fake_item', np.array([[10, 20, 30, 40, 50],
                                                                  [10, 20, 3, 40, 50],
                                                                  [10, 2, 3, 40, 5]]))
timeline_1.getPhases()[0].update()
print('values after modifying phase: ')
print(par.getValues())


timeline_1.getPhases()[0].setItemWeight('fake_item', np.array([[0.99, 3.5, 2.2, 1.1, 3.1]]))
timeline_1.getPhases()[0].update()
print('values after modifying phase: ')
print(weight.getValues())

















