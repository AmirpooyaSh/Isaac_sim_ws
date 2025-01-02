### 1) Design the Model (Design input size and output size + Forward Pass)
### 2) Construct the Loss and Optimizer Functions
### 3) Training Loop
###     - Forward Pass: Compute Predictions
###     - Backward Pass: Gradient Creation
###     - Update Weights

import torch

#Importing Neural Network
import torch.nn as nn

learning_rate = 0.01
n_iters = 4000

if torch.cuda.is_available():
    device = torch.device("cuda")

    X = torch.tensor([[1], [2], [3], [4]], device=device, dtype=torch.float32)
    Y = torch.tensor([[2], [4], [6], [8]], device=device, dtype=torch.float32)

    n_samples, n_features = X.shape
    print(f'#samples: {n_samples}, #features: {n_features}')

    # Creating Test Sample (Number 5)
    X_test = torch.tensor([5], device=device, dtype=torch.float32)

    # Calling the Model with Samples
    input_size = n_features
    output_size = n_features
    model = nn.Linear(in_features = input_size, 
                      out_features = output_size).to(device=device)
    
    '''
    class LinearRegression(nn.Module):
        def __init__(self, input_dim, output_dim):
            super(LinearRegression, self).__init__()
            # define diferent layers
            self.lin = nn.Linear(input_dim, output_dim)

        def forward(self, x):
            return self.lin(x)

    model = LinearRegression(input_size, output_size)
    '''

    #Loss Function
    #Mean Square Root Loss Function !!!
    loss = nn.MSELoss()
    #Stocastic Gradient Descent 
    optimizer = torch.optim.SGD(params=model.parameters(), 
                                lr=learning_rate)

    print(f'Prediction before training: f(5) = {model(X_test).item():.3f}')

    for epoch in range(n_iters):
        # prediction = forward pass with our model
        y_pred = model(X)

        # loss
        l: torch.tensor = loss(Y, y_pred)

        # gradients = backward pass
        l.backward() # dl/dw

        # update weights
        optimizer.step()

        # zero gradients
        optimizer.zero_grad()

        if epoch % 100 == 0:
            [w, b] = model.parameters() # unpack parameters
            print('epoch ', epoch+1, ': w = ', w[0][0].item(), ' loss = ', l)

    print(f'Prediction after training: f(5) = {model(X_test).item():.3f}')
