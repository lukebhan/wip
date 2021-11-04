import numpy as np
import torch
from autoencoder import AE
import matplotlib.pyplot as plt


model = AE()

l = torch.nn.MSELoss()

optimizer = torch.optim.Adam(model.parameters(), 
        lr = 1e-1, 
        weight_decay = 1e-8)


data = np.loadtxt('data')
data = np.concatenate([data, np.loadtxt('data1')])
data = np.concatenate([data, np.loadtxt('data2')])
data = np.concatenate([data, np.loadtxt('data3')])
data = np.concatenate([data, np.loadtxt('data4')])
data = np.concatenate([data, np.loadtxt('data5')])
print(data.shape)
tensor = torch.from_numpy(data).float()

epochs = 20
outputs = []
losses = []
for epoch in range(epochs):
    running_loss = 0.0
    output = model(data)

    loss = l(output, data)
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()
    losses.append(loss.item() / len(data))
    outputs.append((epochs, data, output))
    print(loss.item())
np.savetxt('loss', losses)
plt.plot(losses)
plt.show()

