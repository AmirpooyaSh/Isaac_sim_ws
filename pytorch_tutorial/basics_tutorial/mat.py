import torch
import numpy as np

# In this tutorial, a tensor will be created on GPU and been transferred to cpu

if torch.cuda.is_available():
    device = torch.device("cuda")

    
    # creating a GPU tensor
    x = torch.ones(5, device=device)
    # creating a CPU tensor
    y = torch.ones(5)

    # converting from CPU to GPU
    y = y.to(device=device)

    z = x + y

    # converting the result to CPU
    z = z.to(device="cpu")

    # print
    print(z)


