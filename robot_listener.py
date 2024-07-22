# -*- coding: utf-8 -*-
"""
Created on Mon Apr  9 11:04:28 2018

@author: Jimmy-Exjobb
"""
import agx
import agxUtil
import agxIO
import agxSDK
import agxCable
import agxPython
import agxCollide
import agxOSG
import numpy as np
import math
import sys
import agxData
import agxRender
import argparse

"""Listener part"""


radius = 0.005
resolution = 70

X = agx.Vec3.X_AXIS()
Y = agx.Vec3.Y_AXIS()
Z = agx.Vec3.Z_AXIS()




setup = {'save_data': True,
         'output_name': 'marker',
         'output_path': 'C:/Test_Save'}

r_cable = {'radius':    0.025,
           'resolution': 50,
           'marknumber': 20}
class SaveListener(agxSDK.StepEventListener):
    ''' Listener that saves everything returned by the foo function '''
    def __init__(self, foo, arg, dt, current_marker, time_interval=4, out_path='output', out_name='out', file_out=None):
        super().__init__(agxSDK.StepEventListener.POST_STEP)
        sim = agxPython.getContext().environment.getSimulation()

        self.foo = foo
        self.arg = arg
        self.time_interval = time_interval
        self.out_path = out_path
        self.out_name = out_name
        self.file_out = file_out
        self.timeStep = dt
        self.out = np.zeros((math.floor(time_interval/self.timeStep), 1+len(self.foo(self.arg))), np.float32)
        self.idx = 0
        self.n = 0
        self.last_write = 0.0
        self.last_save = 0.0
        self.cm = current_marker
        self.name = "_step"

    def post(self, t):
        if (t-self.last_write) >= self.timeStep:
            self.last_write = t
            if (t-self.last_save) >= self.time_interval:
                self.last_save = t
                if self.file_out is not None:
                    print('', file=self.file_out, flush=True)
                    print("not saving")
                else:
                    # Save only the lines where not all the elements in the line are zero.
                    # This is needed since the array self.out can be over allocated if not
                    # dt is smaller than the timestep in the simulation.
                    np.savez('%s/%s%d%s%d.npz' % (self.out_path, self.out_name, self.cm, self.name, self.n), self.foo(self.arg))#[~np.all(self.out == 0, axis=1)])
                    print('Saving', self.out_name, self.n, self.foo(self.arg))
                self.n +=1
                self.last_write = t
                self.out = np.zeros(self.out.shape, np.float32)
                self.idx = 0
                self.out[self.idx,:] = [t] + self.foo(self.arg)
                self.idx += 1
            else:
                self.out[self.idx,:] = [t] + self.foo(self.arg)
                if self.file_out is not None:
                    print(self.out_name, self.out[self.idx,:], file=self.file_out)

"""

functions responsible for creating cables using SpaceClaim generated models.

"""

#==============================================================================
# def read_model_name():
#     """Find the --model command line arguments and return its value."""
#     arguments = application().getArguments()
#     num_arguments = arguments.getNumArguments()
#     flag_index = arguments.find("--model")
#     model_index = flag_index + 1
#     if flag_index == -1 or model_index >= num_arguments:
#         print("Missing '--model <name>.agx' command line argument.")
#         return None
#     return arguments.getArgumentName(model_index)
#==============================================================================


def extract_counter(s, i):
    """Reads the i:th underscore-based column and returns it as an integer."""
    columns = s.split("_")
    if i >= len(columns):
        print("String '" + s + "' doesn't have a " + str(i) + "th column.")
    return int(columns[i])


class FreeNodeInfo:
    def __init__(self, marker):
        self.marker = marker
        self.type = "Free"
        self.position = marker.getPosition()


class BodyFixedNodeInfo:
    def __init__(self, marker, fields):
        self.marker = marker
        self.type = "Attach"
        self.position = marker.getPosition()
        self.attach_name = fields[3]


def is_freenode(fields):
    """Return True if the given marker name is a free node marker."""
    return len(fields) == 3 and \
        fields[0].startswith("Cable") and \
        fields[1].isdigit() and \
        fields[2] == "Free"


def is_bodyfixednode(fields):
    """Return True if the given marker name is a body fixed node marker."""
    return len(fields) > 3 and \
        fields[0].startswith("Cable") and \
        fields[1].isdigit() and \
        fields[2] == "Attach"


def is_marker_name(name):
    """Return true if the given name is a valid marker name."""
    fields = name.split("_")
    return is_freenode(fields) or is_bodyfixednode(fields)


def create_node_info(marker):
    """Parse the given body name and create a node info instance."""
    fields = marker.getName().split("_")

    if is_freenode(fields):
        return FreeNodeInfo(marker)
    elif is_bodyfixednode(fields):
        return BodyFixedNodeInfo(marker, fields)
    else:
        print("Routing body '" + marker.getName() +
              "' is not a valid route specification.")
        return None


def create_node(cable, pos):
    node = agxCable.FreeNode(pos)
    cable.add(node)


def find_closest_segment(cable, pos):
    segment = cable.begin()
    closest_segment = None
    closest_distance = math.inf
    while not segment.isEnd():
        begin_dist = segment.getBeginPosition().distance(pos)
        end_dist = segment.getEndPosition().distance(pos)
        dist = min(begin_dist, end_dist)
        if dist < closest_distance:
            closest_segment = segment
            closest_distance = dist
        segment = segment.next()
    return closest_segment


def create_relative_transform(segment, body):
    """Create a transformation that describe the segment's position relative to the body."""
    cable_frame = segment.getRigidBody().getFrame()
    body_frame = body.getFrame()
    local_frame = agx.Frame()
    local_frame.setParent(body_frame)
    local_frame.setMatrix(cable_frame.getMatrix())  # Will compute the local matrix.
    transform = local_frame.getLocalMatrix()
    return transform


def create_cable(sim, root, name_prefix):
    
    ##need to change this so we don't have to use scene
    
    bodies = sim.getRigidBodies()
    body_names = map(lambda body: body.getName(), bodies)
    marker_names = sorted(filter(lambda name: name.startswith(name_prefix), body_names))
    marker_bodies = [sim.getRigidBody(name) for name in marker_names]
    attachments = []
    cable = agxCable.Cable(radius, agxCable.PathRoute(resolution))
    
    for marker in marker_bodies:
        print("got marker named" + marker.getName())
        node_info = create_node_info(marker)
        if node_info is None:
            print("error")
        if node_info.type == "Free":
            create_node(cable, node_info.position)
        elif node_info.type == "Attach":
            create_node(cable, node_info.position)
            attachments.append({
                                'pos': node_info.position,
                                'body_name': node_info.attach_name
                                })
        else:
            print("Got unknown cable node type")
        sim.remove(marker)
    sim.add(cable)
    renderer = agxOSG.createVisual(cable, root)
    agxOSG.setDiffuseColor(renderer, agxRender.Color(0.8, 0.4, 0.0, 0.0))

    print("Cable got resolution " + str(cable.getResolution()) + " and " +
          str(cable.getNumSegments()) + " segments.")

    for attachment in attachments:
        pos = attachment['pos']
        segment = find_closest_segment(cable, pos)
        body = sim.getRigidBody(attachment['body_name'])
        transform = create_relative_transform(segment, body)
        cable.attach(segment, body, transform)
        cable.setEnableCollisions(body, False)  # May want to disable only some segments.

#==============================================================================
#     cable.getMaterial().setName(name_prefix + "Material")
#==============================================================================

    return cable
    
    

def create_cables(sim, root):
    cables = [
        # List the cables prefixes used when the markers were created.
        create_cable(sim, root, 'Cable1'),
        create_cable(sim, root, 'Cable2'),
        create_cable(sim, root, 'Cable4'),
        create_cable(sim, root, 'Cable3'),
    ]

    for cable in cables:
        cable_properties = cable.getCableProperties()
        cable_properties.setYoungsModulus(1e7, agxCable.BEND)
        
                    
'''
Scene building

'''                    
                    
                    
def buildScene():
    
#==============================================================================
#     ap = argparse.ArgumentParser()
#     ap.add_argument('--bend', type=float, default=5e5, help='Youngs modulus bend')
#     ap.add_argument('--stretch', type=float, default=1e5, help='Youngs modulus stretch')
#     ap.add_argument('--path', default='C:/Test_Save/xddd', help='Path to save data, differs betwen true and temporary positions')
#     args, unknown = ap.parse_known_args()
#     args = vars(args)
#     
#     ym_bend = args['bend']
#     ym_stretch = args['stretch']
#     setup['output_path'] = args['path']
#==============================================================================
    
    # Get
    sim = agxPython.getContext().environment.getSimulation()
    app = agxPython.getContext().environment.getApplication()
    root = agxPython.getContext().environment.getSceneRoot()
    print("\n\n = Scene creation start =\n")
    model_name = "test_agx_1.agx"
    robot = agxSDK.Assembly()
    agxOSG.readFile(model_name, sim, root, robot)
    create_cables(sim, root)
            
    
    
    body1 = agx.RigidBody()
    geom1 = agxCollide.Geometry( agxCollide.Sphere(0.5) )
    body1.add(geom1)
    sim.add(body1)
    agxOSG.createVisual(body1,root)
        
    #r = 0.025
    #res = 50
    rb_list = []
    i = 0
    ct_1DOF= []    

    robot = agxSDK.Assembly()
    agxOSG.readFile("IRB2400.agx", sim, root, robot)
        
        
    geometries = robot.getRigidBodies()
    print(geometries)
    for geometry in geometries:
        rb_list.append(robot.getRigidBody(geometry.getName()))
        print (geometry.getName())
        
    a = robot.getRigidBody("IRB2400_10_m2000_rev3_01-4") ##dolna czesc ramienia
    b = robot.getRigidBody("IRB2400_10_m2000_rev3_01-1") ##podstawa robota
    c = robot.getRigidBody("IRB2400_10_m2000_rev3_01-3") ##
    d = robot.getRigidBody("IRB2400_10_m2000_rev3_01-5") ##szary dlugi metal
    e = robot.getRigidBody("RigidBody14") ## czesc robota na podstawa
    f = robot.getRigidBody("Component5") ## 1/2 koncowek manipulatora
    g = robot.getRigidBody("Component4") ## 2/2 koncowek manipulatora
    h = robot.getRigidBody("RigidBody21") ## gorne ramie robota
    
    cable = agxCable.Cable(r_cable['radius'], r_cable['resolution'])
    properties = cable.getCableProperties()
    properties.setYoungsModulus(1e5, agxCable.STRETCH)
    
    cable.add(agxCable.BodyFixedNode(b, agx.Vec3(0, -0.3, 0)))
    cable.add(agxCable.BodyFixedNode(e, agx.Vec3(0.38, -0.1, 0))) ##done
    cable.add(agxCable.BodyFixedNode(h, agx.Vec3(0,-0.12,0))) ##done
    cable.add(agxCable.BodyFixedNode(h, agx.Vec3(0.6,-0.12,0)))
    sim.add(cable)
    
    
    
    #print(Cable1.getRestLength())
    
    # calculate segment properties, put marks on it etc.
    current_length = 0
    current_marker = 1
    length = cable.getRestLength()
    mark_interval = length/r_cable['marknumber']
    n = cable.getNumSegments()
    segment_length = length/n
    mark_interval = length/r_cable['marknumber']
    print (mark_interval)
    temp_iter = cable.begin()
    while not temp_iter.isEnd():
        rigid = temp_iter.getRigidBody()
        geom = temp_iter.getGeometry()
        segment = agxOSG.createVisual(geom, root)
        if current_length>=mark_interval:
            agxOSG.setTexture(geom, root, 'C:/Users/Jimmy-Exjobb/Desktop/Master Thesis/Video2Simulation/data/textures/2red.png')
            current_length = 0
            
            #saving position from the simulatied rigid bodies to numpy file
            if setup['save_data']:
                def position(arg):
                    position_temp = arg['rigid'].getPosition()
                    velocity_temp = arg['rigid'].getVelocity()
                    print(velocity_temp.x())
                    print(position_temp.x(), position_temp.y(), position_temp.z())
                    return [(position_temp.x(), position_temp.y(), position_temp.z(), velocity_temp.x(), velocity_temp.y(), velocity_temp.z())]
                position_save = SaveListener(position,{'rigid': rigid},0.1, current_marker, time_interval=1, out_path = setup['output_path'], out_name = setup['output_name'])
                sim.add(position_save)
                current_marker = current_marker+1
                
        else:
            current_length += segment_length
        temp_iter.inc()
    
    #print(rb_list)
    #for i in range(len(rb_list)):
        #cable.add(agxCable.BodyFixedNode(rb_list[i], agx.Vec3(0,-1,0)))

    
    #cable.add(agxCable.BodyFixedNode(c, agx.Vec3(1,0,0)))

    #cable.add(agxCable.BodyFixedNode(d, agx.Vec3(0,0,0)))
    
    hinge = robot.getConstraint("Base") ##obracanie w osi z, podstawa robota
    hinge_1 = robot.getConstraint("Lower Joint") 
    hinge_2 = robot.getConstraint("high")
    hinge_3 = robot.getConstraint("Hinge4")
    hinge_4 = robot.getConstraint("Upper Joint")##podnoszenie w osi z wzgledem podstawy robota
    hinge_5 = robot.getConstraint("Hinge8")
    hinge_6 = robot.getConstraint("LeftGrip")
    hinge_7 = robot.getConstraint("RightGrip")
    
    
    hinge_1dof = agx.Constraint1DOF_safeCast(hinge)
    hinge1_1dof = agx.Constraint1DOF_safeCast(hinge_1)
    hinge4_1dof = agx.Constraint1DOF_safeCast(hinge_4)
    
    hinge_1dof.getMotor1D().setEnable(True)
    hinge_1dof.getMotor1D().setSpeed(1)
    
    hinge1_1dof.getMotor1D().setEnable(True)
    hinge1_1dof.getMotor1D().setSpeed(10)
    
    #hinge4_1dof.getMotor1D().setEnable(True)
    #hinge4_1dof.getMotor1D().setSpeed(1)
    
    #print(boolean)
    #hinge.getMotor1D().setEnable(True)
    #hinge.getMotor1D().setSpeed(1)



    #agxOSG.createVisual(body, root) # create graphical representation of the body
    #gxOSG.createVisual(cable,root)    

def main(args):
    # Create an application with graphics etc.
    app = agxOSG.ExampleApplication()

    # Create a command line parser. sys.executable will point to python 
    # executable in this case, because getArgumentName(0) needs to match the
    # C argv[0] which  is the name of the program running
    argParser = agxIO.ArgumentParser([sys.executable] + args)

    app.addScene(argParser.getArgumentName(1), "buildScene", ord('1'))

    # Call the init method of ExampleApplication
    # It will setup the viewer, windows etc.
    if app.init(argParser):
        app.run()
    else:
        print("An error occurred while initializing ExampleApplication.")

# Entry point when this script is loaded with python
if agxPython.getContext() is None:
    init = agx.AutoInit()
    main(sys.argv)