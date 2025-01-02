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
n_iters = 100

def forward(x: torch.Tensor,
            w: torch.Tensor) -> torch.Tensor:
    return w * x

if torch.cuda.is_available():
    device = torch.device("cuda")
    X = torch.tensor([1, 2, 3, 4], device=device, dtype=torch.float32)
    Y = torch.tensor([2, 4, 6, 8], device=device, dtype=torch.float32)

    w = torch.tensor(0.0, device=device, dtype=torch.float32, requires_grad=True)

    #Loss Function
    #Mean Square Root Loss Function !!!
    loss = nn.MSELoss()
    #Stocastic Gradient Descent 
    optimizer = torch.optim.SGD(params=[w], 
                                lr=learning_rate)

    print(f'Prediction Before Training: f(5) = {forward(5, w):0.3f}')


    for epoch in range(n_iters):
        # prediction = forward pass
        y_pred = forward(X, w)

        # loss
        l: torch.tensor = loss(Y, y_pred)

        # gradients = backward pass
        l.backward() # dl/dw

        # update weights
        optimizer.step()

        # zero gradients
        optimizer.zero_grad()

        if epoch % 10 == 0:
            print(f'epoch {epoch+1}: w = {w:.3f}, loss = {l:.8f}')

    print(f'Prediction After Training: f(5) = {forward(5, w):0.3f}')



