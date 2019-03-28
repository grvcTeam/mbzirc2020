import argparse

parser = argparse.ArgumentParser(description='this script generates a xacro macro for a pile of bricks model using urdf/includes/brick_link.xacro. The macro ' \
                                 'will be stored in the folder and file urdf/[model_name].xacro. Eg: python xacro_brick_pile.py orange_pile 2.0 Gazebo/Orange 3 3 3 1.8 0.2 0.2')
parser.add_argument("model_name", help="name for the pile model")
parser.add_argument("mass", help="brick mass", type=float)
parser.add_argument("material", help="brick gazebo material")
parser.add_argument("num_x", help="number of bricks in X axis", type=int)
parser.add_argument("num_y", help="number of bricks in Y axis", type=int)
parser.add_argument("num_z", help="number of bricks in Z axis", type=int)
parser.add_argument("sx", help="x component of brick scale vector", type=float)
parser.add_argument("sy", help="y component of brick scale vector", type=float)
parser.add_argument("sz", help="z component of brick scale vector", type=float)

args = parser.parse_args()

def brick_string(id,mass,material,sx,sy,sz,x,y,z):
    s = '<brick_link id="{id}" id2="${{id}}" type="brick" sx="{sx}" sy="{sy}" sz="{sz}" mass="{mass}" material="{material}" pose="{x} {y} {z} 0 0 0" />'.format(id=id, \
        sx=sx,sy=sy,sz=sz,x=x,y=y,z=z,mass=mass,material=material)

    return s

sx = args.sx
sy = args.sy
sz = args.sz

num_x = args.num_x
num_y = args.num_y
num_z = args.num_z

model_name = args.model_name
mass = args.mass
material = args.material

with open('../urdf/'+model_name+'.xacro','w') as pile_model:
    #begin
    with open('../urdf/includes/pile_model.xacro.begin') as begin:
        pile_model.write(begin.readline().format(model_name=model_name))
    #body
    counter = 0
    for i in range(num_x):
        for j in range(num_y):
            for k in range(num_z):
                pile_model.write(brick_string(counter,mass,material,sx,sy,sz,i*sx,j*sy,1.1*k*sz+sz/2.))
                counter += 1
    #end
    with open('../urdf/includes/pile_model.xacro.end') as end:
        pile_model.write(end.readline())
