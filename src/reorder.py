#!/usr/bin/env python

import sys
import re
import collections

def reorder_YAML(inString) :
  NodeDict = dict()
  NodeIndex = dict()
  nodeString = ""
  node = ""

  placeNode = False
  for line in inString.split('\n'): 
    if re.match(r'ROOT*|THEN*|AND*|OR*|PLACE*', line):
      if nodeString != "":
        NodeDict[node] = nodeString
      nodeString =""
      node = re.sub(r':', "", line.strip())
      numb = re.search(r'...$', node).group()
      NodeIndex[numb] = node
    if len(line) != 0:
      if re.match(r'.*PLACE.*:.*', line):
        placeNode = True
      if placeNode == True:  
        if re.match(r'.*children:.*',line):
          continue
        elif re.match(r'.*object:.*',line):
          line_l=line.split(':')
          line = ':'.join(line_l)
        elif re.match(r'.*loc_obj:.*',line):
          line_l=line.split(':')
          line = ':'.join(line_l)
          placeNode = False
      nodeString += '  ' + line + '\n'
  NodeDict[node] = nodeString
  NodeIndex = collections.OrderedDict(sorted(NodeIndex.items())) 
  listString = ("NodeList: [")
  nodeString= "Nodes: \n"
  for key, val in NodeIndex.items():
    nodeString += NodeDict[val]
    listString += "'"+ val +"'" + ", "
  listString = re.sub (r'..$', ' ]\n', listString)

  finalString = listString + nodeString
  listString =re.sub(r'^.*\[', "", listString) 
  listString =re.sub(r'\].*', "", listString)
  listString_l = listString.split()
  firstNode = re.sub(r',', "", listString_l[1].strip())
  firstNode = re.sub(r'\'', "", firstNode)

  return finalString, firstNode