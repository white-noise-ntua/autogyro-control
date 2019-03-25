from sklearn.cluster import KMeans
from sklearn.model_selection import train_test_split
from torch.autograd import Variable
import numpy as np
import torch
import torch.nn as nn
import sys

class LinearRegressionModel(nn.Module):

    def __init__(self, input_dim, output_dim):
        super(LinearRegressionModel, self).__init__()
        self.linear1 = nn.Linear(input_dim, input_dim)

    def forward(self, x):
        out = self.linear1(x)
        return out

kmeans = KMeans(n_clusters=20, random_state=0)

moments = np.loadtxt('moments.csv', delimiter=',')
angles = np.loadtxt('angles.csv', delimiter=',')

kmeans.fit(moments)

X_train = kmeans.cluster_centers_
y_train = np.zeros(shape=(kmeans.n_clusters, 3), dtype=np.float64)

for j, x in enumerate(X_train):

    argmin = 0
    minimum = sys.maxsize

    for i in range(moments.shape[0]):
        temp = np.linalg.norm(x - moments[i])
        if temp < minimum:
            minimum = temp
            argmin = i
    y_train[j, :] = angles[argmin, :]

input_dim = moments.shape[-1]
output_dim = angles.shape[-1]



_, X_test, _, y_test = train_test_split(moments, angles, test_size=0.33, random_state=42)


model = LinearRegressionModel(input_dim,output_dim)

criterion = nn.MSELoss()
l_rate = 0.1
momentum = 0.9
weight_decay = 1e-6
optimiser = torch.optim.SGD(model.parameters(), lr = l_rate, momentum=momentum, weight_decay=weight_decay, nesterov=True)

epochs = 300000

for epoch in range(epochs):

    inputs = torch.from_numpy(X_train).float()
    gold_outputs = torch.from_numpy(y_train).float()


    optimiser.zero_grad()
    outputs = model(inputs)

    loss = criterion(outputs, gold_outputs)
    loss.backward()
    optimiser.step()

    if epoch % 1000 == 0:
        print('Epoch {}, Loss {}'.format(epoch + 1, str(loss)))



inputs = torch.from_numpy(X_test).float()
gold_outputs = torch.from_numpy(y_test).float()

outputs = model(inputs)

loss = criterion(outputs, gold_outputs)


print('Test Loss {}'.format(str(loss)))

print(model.state_dict())
