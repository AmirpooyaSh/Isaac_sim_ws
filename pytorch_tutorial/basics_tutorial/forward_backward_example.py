import torch

# This is a example inspired by the tutorial below
# https://www.youtube.com/watch?v=3Kb0QS6z7WA&list=PLqnslRFeH2UrcDBWF5mfPGpqQDSta6VK4&index=4


# Doing Forward/Backward Optimization in Pytorch (Backpropagation)

if torch.cuda.is_available():
    # Creating a GPU Based Code
    device = torch.device("cuda")

    x = torch.tensor(1.0, device=device, requires_grad=True)
    y = torch.tensor(2.0, device=device)

    #requires_grad=True : Will let you do Backpropagation

    w = torch.tensor(1.0, device=device, requires_grad=True)

    #forward pass and compute the loss
    y_hat = w * x
    loss = (y_hat - y)**2

    print(loss)

    # backward pass (Does the Backward Pass)
    loss.backward()
    print(w.grad)
    print(x.grad)

    ###
    ### Make sure to empty out the grad values before iterating
    ### To avoid suming the Gradient Values upon iterations in backward
    ### function
    w.grad.zero_()
    x.grad.zero_()

    # Update Weights
    # Next Forward Backward Pass .......
