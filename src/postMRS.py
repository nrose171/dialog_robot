import re
  
class CommandTree:
    class Node:
        def __init__(self, ID):
            self.ID = ID;
            self.children = list()
  
        def _addChildren(self, children):
            for c in children:
                children.append(c)
  
def parseMRSintoCommand(MRS):
    relsLines = list()
    takeRels = False
    index = "IO"
    sentence = ""
    MRS = MRS.split('\n')
    for line in MRS:    
        line = line.strip()
        if re.match(r'^SENT*', line):
            line = re.sub(r'^SENT: ', "", line)
            sentence = line.strip()
        elif re.match(r'INDEX*', line):
            line = line.split()
            index = line[1]
        elif re.match(r'RELS*', line):
            line = re.sub(r'RELS: < ', "", line)
            takeRels = True
        elif re.match(r'HCONS*', line):
            takeRels = False
        if takeRels ==True:
            relsLines.append(line)
    rels = simplify(relsLines)
    adjectiveHandling(rels)
    prepositionHandling(rels)
    command = buildTree (rels, index)

    #############TEST###########
    print(command)
    ############################

    return command
  
def simplify (rels):
    rels_dic = dict()
    rels_dic_impl = dict()
    implicit_conj = False
    conjunction = ""
    for r in rels:
        r = r.split()
        ARG_0 = r[5]
        ARGS = getARGS(r)
        location = re.sub(r'>*', "", re.sub(r'.*<', "", r[1]))
        if re.match(r'.*_v_.*', r[1]):
            category = "verb"
            word = re.sub(r'^_', "", re.sub(r'_v_.*', "", r[1]))
            rels_dic[ARG_0] = [category, word, location, ARGS]
        elif re.match(r'.*_n_.*', r[1]):
            category = "noun"
            word = re.sub(r'^_', "", re.sub(r'_n_.*', "", r[1]))
            rels_dic[ARG_0] = [category, word, location, ARGS]
        elif re.match(r'.*_a_.*', r[1]):                            
            category = "adjective"                                  
            word = re.sub(r'^_', "", re.sub(r'_a_.*', "", r[1]))    
            rels_dic[ARG_0] = [category, word, location, ARGS]      
        elif re.match(r'.*_p.*', r[1]):                                         
            category = "preposition"                                
            word = re.sub(r'^_', "", re.sub(r'_p.*', "", r[1]))     
            rels_dic[ARG_0] = [category, word, location, ARGS]
        elif re.match(r'^_and_.*', r[1]):
            conjunction = word
            category = "conj"
            word = "and"
            conjunction = word            
            rels_dic[ARG_0] = [category, word, location, ARGS]    
        elif re.match(r'^_or_.*', r[1]):
            category = "conj"
            word = "or"
            conjunction = word
            rels_dic[ARG_0] = [category, word, location, ARGS]  
        elif re.match(r'^_then_.*', r[1]):
            category = "conj"
            word = "then"
            conjunction = word
            rels_dic[ARG_0] = [category, word, location, ARGS] 
        elif re.match(r'^implicit_conj.*', r[1]):
            implicit_conj = True
            category = "conj"
            word = "implicit"
            rels_dic[ARG_0] = [category, word, location, ARGS]                 
        else:
            category = "other"
    if implicit_conj == True:
        rels_dic = orderImplicitAGS(rels_dic, conjunction)  
    return rels_dic

def orderImplicitAGS(old_rels, conj):
    new_rels = dict()
    arguments = list()
    implicit_keys = list()
    conj_key = ""
    verb_key = ""
    for key in old_rels:
        value = old_rels[key]
        if value[0] != "conj":
            new_rels[key] = old_rels[key]
            if value[0] == "verb":
                verb_key = key 
        else:
            for arg in value[3]:
                if old_rels[arg][0] == "noun":
                    arguments.append(arg)
            if value[1] != "implicit":
                location = value[2]
                conj_key = key
    new_rels[verb_key][3][1] = conj_key            
    new_rels[conj_key] = ['conj', conj, location, arguments]

    return new_rels
  
def getARGS (argList):
    newList = list()
    for index in range(0, len(argList)):        
        if re.match(r'^ARG.*', argList[index].strip()):
            if argList[index].strip() != "ARG0:" :      
                newList.append(argList[index+1])
        elif re.match(r'^L-IND.*', argList[index].strip()):
            newList.append(argList[index+1])
        elif re.match(r'^R-IND.*', argList[index].strip()):
            newList.append(argList[index+1])
    return newList
  
  
def buildTree (rels, index):
    verb = rels[index][1]
#    commandString = "( ROOT " + getTree(verb, rels, rels[index][3][1]) + " )"

    commandString = getTree(verb, rels, rels[index][3][1])
 
    return commandString
  
def getTree (verb, rels, index):
    commandString = ""
    word_type = rels[index][0]
    if word_type == 'noun':
        predicate = getPredicate(verb)
        commandString = "( " + predicate + " " + rels[index][1] + " )"
    elif word_type == 'conj':
        if rels[index][1] == 'or':
            commandString = "( OR " 
        elif rels[index][1] == 'and':
            commandString = "( AND "   
        elif rels[index][1] == 'then':
            commandString = "( THEN "  
        for arg in rels[index][3]:
            commandString += getTree(verb, rels, arg) + " "
        commandString += ")"
    else: # word_type "THEN"
        pass
    return commandString    
  
def getPredicate (verb):
    if verb == "place" or verb == "put":
        return "PLACE"
    else:
        return "NONE"
def adjectiveHandling(rels):
    for adj in rels:
        if rels[adj][0] == "adjective": 
            relation = rels[adj][3][0]
            word = rels[relation][1]
            rels[relation][1] =rels[adj][1] + "_" + word

def prepositionHandling(rels):
    for prep in rels:
        if rels[prep][0] == "preposition":
            subjects = list()
            getSubjects(rels, prep, subjects)
            destination = rels[prep][3][1]
            dest_name = rels[destination][1]
            preposition = rels[prep][1]                  
            for x in subjects:
		if rels[x][1] == "pink_bar":
		    preposition = "on_l"
		elif rels[x][1] == "yellow_bar":
		    preposition = "on_r"
                word = rels[x][1]
                rels[x][1] = word + " " + dest_name + " " + preposition
def getSubjects(rels, prep, subjects):
    for arg in rels[prep][3]:
        if rels[arg][0] == 'noun':
            subjects.append(arg)
        elif rels[arg][0] == 'conj':
            getSubjects(rels, arg, subjects)
