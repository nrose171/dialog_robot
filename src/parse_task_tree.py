#!/usr/bin/env python

import sys
'''
Description: A program to parse the Task Expression Language
Output => A preorder tree traversal and a YAML file
'''

Symbol = str

#Preposition offsets
def get_offsets(word):
  offset_dict = {
    '' : [0, 0, 0],
    'on' : [0.330056,0.354265,0.028489],
    'onto' : [0.292087,0.016732,-0.015673],
    'in+front+of' : [0.718193,-0.298247,0.990027],
    'on_l' : [0.250604,-0.04831,0.004393],
    'on_r' : [0.26528,0.272534,0.011394]  
  }
  return offset_dict[word]

# Tokenize the input stream of characters based on the language
def tokenize(chars):
  return chars.replace('(', ' ( ').replace(')', ' ) ').split()

def read_from_tokens(tokens):
  if len(tokens) == 1:
    raise SyntaxError('Unexpected EOF while reading')
  token = tokens.pop(0)
  if '(' == token:
    L = []
    while tokens[0] != ')':
      L.append(read_from_tokens(tokens))
    tokens.pop(0)
    return L
  elif ')' == token:
    raise SyntaxError('Unexpected [)] symbol')
  else:
    return atom(token)

def atom(token):
  try: return int(token)
  except ValueError:
    try: return float(token)
    except ValueError:
      return Symbol(token)

#   Parse the input character stream and output syntax tree
def parse(program):
  program = "(ROOT" + program + ")"
  return read_from_tokens(tokenize(program))

class Env(dict):
  "An environment: a dict of {'var':val} pairs, with an outer Env."
  def __init__(self, parms=(), args=(), outer=None):
    self.update(zip(parms, args))
    self.outer = outer if outer else {}
  def find(self, var):
    "Find the innermost Env where var appears."
    return self if (var in self) else None


def CreateRootObject(robot_id, node_id, parent='NONE'):
  return TaskObject('ROOT', 4, robot_id, node_id, parent)

def CreateThenObject(robot_id, node_id, parent):
  return TaskObject('THEN', 0, robot_id, node_id, parent)

def CreatePlaceObject(robot_id, node_id, parent):
  return PlaceObject('PLACE', 6, robot_id, node_id, parent)

def CreateAndObject(robot_id, node_id, parent):
  return TaskObject('AND', 2, robot_id, node_id, parent)

def CreateOrObject(robot_id, node_id, parent):
  return TaskObject('OR', 1, robot_id, node_id, parent)


class TaskObject(object):
  def __init__(self, name='', node_type=0, robot_id=0, node_id=0, parent=''):
    self.node_type = node_type
    self.robot_id = robot_id
    self.node_id = node_id
    self.name = '%s_%d_%d_%03d' % (name, node_type, robot_id, node_id)
    self.children = ['NONE']
    self.parent = parent
    self.peers = ['NONE']

  def __call__(self, *args):
    self.children = [child.name for child in args]
    print(self)
    return self

  def __repr__(self):
    string = (
      '%(name)s:\n'
      '  mask:\n'
      '    type: %(node_type)d\n'
      '    robot: %(robot_id)d\n'
      '    node: %(node_id)d\n'
      '  parent: %(parent)s\n'
      '  children: %(children)s\n'
      '  peers: %(peers)s'
      % {
        'name': self.name,
        'node_type': self.node_type,
        'robot_id': self.robot_id,
        'node_id': self.node_id,
        'parent': self.parent,
        'children': self.children,
        'peers': self.peers
      }
    )
    return string

class PlaceObject(TaskObject):
  def __init__(self, name='', node_type=0, robot_id=0, node_id=0, parent=''):
    super(PlaceObject, self).__init__(name, node_type, robot_id, node_id, parent)
    self.place_object = ''
    self.dest_object = ''
    self.off_x = 0.0
    self.off_y = 0.0
    self.off_z = 0.0

  def __call__(self, item, dest = "", prep = ""):
    self.place_object = item
    self.dest_object = dest
    offsets = get_offsets(prep)
    self.off_x = offsets[0]
    self.off_y = offsets[1]
    self.off_z = offsets[2]
    print(self)
    return self

  def __repr__(self):
    parent_str = super(PlaceObject, self).__repr__()
    string = (                             
      '%(parentString)s\n'                                     
      '  object: %(object)s\n'                                 
      '  loc_obj: %(location)s\n'                             
      '  off_x: %(offset_x)3.6f\n'                             
      '  off_y: %(offset_y)3.6f\n'                            
      '  off_z: %(offset_z)3.6f'                               
      % {                                  
        'parentString': parent_str,        
        'object': self.place_object,      
        'location': self.dest_object,      
        'offset_x': self.off_x,
        'offset_y': self.off_y,            
        'offset_z': self.off_z             
      }                                    
    )                                      
    return string

def standard_env():
  env = Env()
  env.update({

    'THEN':    CreateThenObject,
    'PLACE':   CreatePlaceObject,
    'AND':     CreateAndObject,
    'OR':      CreateOrObject,
    'ROOT':    CreateRootObject,
  })
  return env
global_env = standard_env()

#   "Evaluate an expression in an environment."
def eval(x, env=global_env, func_index=[0], parent="'NONE'"):
  if isinstance(x, Symbol):      # x is a string
    if env.find(x):
      value = env.find(x)[x](0, func_index[0], parent)
      func_index[0] += 1
      return value
    return x
  else: # x is a list
    proc = eval(x[0], env, parent=parent)
    args = [eval(arg, env, parent=proc.name) for arg in x[1:]]
#    return proc(*args)
    return proc(*args)

def parse_command(command):
  parse_str = parse(command)
  x = eval(parse_str)

if __name__ == '__main__':

  command = sys.argv[1]
  parse_command (command)
