### 1) Design the Model (Design input size and output size + Forward Pass)
### 2) Construct the Loss and Optimizer Functions
### 3) Training Loop
###     - Forward Pass: Compute Predictions
###     - Backward Pass: Gradient Creation
###     - Update Weights

import torch
import torch.nn as nn
import numpy as np

from sklearn import datasets
import matplotlib.pyplot as plt

device: str = ""
if torch.cuda.is_available():
    device = torch.device("cuda")
else:
    device = "cpu"

# 0) Prepare Data
X_numpy, y_numpy = datasets.make_regression(n_samples=100, n_features=1, noise=20, random_state=4)

# cast to float Tensor
X = torch.from_numpy(X_numpy.astype(np.float32)).to(device=device)
y = torch.from_numpy(y_numpy.astype(np.float32)).to(device=device)
y = y.view(y.shape[0], 1)

n_samples, n_features = X.shape

# 1) Design the Model
# Linear model f = wx + b
input_size = n_features
output_size = 1
model = nn.Linear(input_size, output_size).to(device=device)

# 2) Define Loss + Optimizer
learning_rate = 0.01
criterion = nn.MSELoss()
optimizer = torch.optim.SGD(params=model.parameters(), lr=learning_rate)  

# 3) Training Loop
num_epochs = 100
for epoch in range(num_epochs):
    # Forward pass and loss
    y_predicted = model(X)
    loss = criterion(y_predicted, y)
    
    # Backward pass and update
    loss.backward()
    optimizer.step()

    # zero grad before new step
    optimizer.zero_grad()

    if (epoch+1) % 10 == 0:
        print(f'epoch: {epoch+1}, loss = {loss.item():.4f}')

# Plot
predicted = model(X).detach().cpu().numpy()

plt.plot(X_numpy, y_numpy, 'ro', label="Actual")
plt.plot(X_numpy, predicted, 'b', label="Predicted")
plt.legend()
plt.savefig('plot.png')  # Save the plot as a PNG file
print("Plot saved as 'plot.png'")