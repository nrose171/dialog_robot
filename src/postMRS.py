import re

#used for creating rels_dic out of MRS notation
#then uses rels_dic to create parenthesized commands
  
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

    #search for 'while', if found, split sentence into two and run both seperately then add while at the end
    #Search for number of verbs. If 2 or more found, split sentence and run Sentences seperately then recombine with conjuntion after.
    commandL = list()
    verbL = list()

    verbL = returnVerbs( relsLines )
    j = len(verbL)  #j = number of verbs

    #Assumes only 1 or 2 verbs for now
    if j >= 2:
        relsLinesList = splitSentenceIntoTwo(relsLines)
        conj = returnConj( relsLinesList )
    else:
        relsLinesList = [relsLines]

    for i in range(j):
        rels = simplify(relsLinesList[i])
        adjectiveHandling(rels)
        prepositionHandling(rels)
        #command = buildTree(rels, verbL[i])
        commandL.append( buildTree(rels, verbL[i]) )
    
    if j >= 2:
        finalCommand = addConjToEnd( commandL, conj )
    else:
        finalCommand = commandL[0]
    
    #############TEST###########
    print(finalCommand)
    ############################

    return finalCommand

#checks for verbs in a sentence and returns them in a list
def returnVerbs( sentence ):

    verbL = list()
    for word in sentence:
        word = word.split()
        if re.match(r'.*_v_.*', word[1]):
            verbL.append( word[5] )

    return verbL

#splits a sentence with 2 verbs and 1 conj into two simple sentences. ASSUMES just 2 verbs and 1 conj.
def splitSentenceIntoTwo( sentence ):

    sent1 = list() 
    sent2 = list()

    #conjHit = False
    conj = None
    for wordL in sentence:
        word = wordL.split()
        if re.match(r'^_and_.*', word[1]) or re.match(r'^_or_.*', word[1]) or re.match(r'^_then_.*', word[1]) or re.match(r'^_while_.*', word[1]):
            conjHit = True
            conj = word[1]
        elif conj is None:
            sent1.append(wordL)
        else:
            sent2.append(wordL)

    sentL = list()    
    sentL.append(sent1)
    sentL.append(sent2)
    sentL.append(conj)
    return sentL

#Takes in a list of MRS sentences and assumes the first element is a conjunction. Returns that conjunction.
def returnConj( sentence_list ):

    conj = sentence_list[2]
    if re.match(r'^_and_.*', conj):
        return 'AND'
    elif re.match(r'^_or_.*', conj):
        return 'OR'
    elif re.match(r'^_then_.*', conj):
        return 'THEN'
    elif re.match(r'^_while_.*', conj):
        return 'WHILE'
    else:
        rospy.logerr( "ERROR: 2 VERBS PASSED, BUT NO CONJUNCTION PASSED" )

#need to add passed conjunction to end of parenthesized command
def addConjToEnd( pCommandL, conj ):

    commandString = "( " + conj + " "
    for i in range(len(pCommandL)):
        commandString += pCommandL[i] + " "
    commandString += ")"
    return commandString

#creates rels dictionary
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
    verb = rels[index][1]	#N: assumes there is 1 verb
    commandString = "( ROOT " + getTree(verb, rels, rels[index][3][1]) + " )"

    commandString = getTree(verb, rels, rels[index][3][1])
 
    return commandString
  
#creates parenthesized commands based off the word with respect to a verb
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
    else:
        pass
    return commandString    
  
def getPredicate (verb):
    if verb == "place" or verb == "placing" or verb == "put" or verb == "putting":
        return "PLACE"
    elif verb == "hold" or verb == "holding":
        return "HOLD"
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
			
			#orange hacking 
            if rels[arg][1] == 'top':
				rels[arg][1] = 'orange_' + rels[arg][1] 
		
            subjects.append(arg)

        elif rels[arg][0] == 'conj':
            getSubjects(rels, arg, subjects)






