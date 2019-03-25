import numpy as np
import sys

data = np.loadtxt(sys.argv[1], delimiter=',')
name = sys.argv[1].split('.')[-2]
with open(name + '.mat', 'w+') as f:
    f.write('{} {} [{}][{}] = '.format(sys.argv[2], name, data.shape[0], data.shape[1]))
    f.write('{\n')    
    for x in data:
        f.write('\t{')
        f.write(','.join(x.astype(str)))
        f.write('},\n')

    f.write('};\n')


