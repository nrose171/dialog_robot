import re
import sys

#Used for storing MRS notation of parsed words in a list
def collectingMRS (inputString):
    MRS_list =  list()
    MRS_string =""
    record = False
 
    for line in inputString.split('\n'):
        line = line.strip()
        if re.match(r'^\[ LTOP.*', line):
            record = True
        if record == True:
            MRS_string += '\n' + line
        if re.match(r'^HCONS.*', line):
            record = False
            MRS_list.append(MRS_string)
            MRS_string = ""
 
    return MRS_list
 
#used to select MRS that has the desired params for each word
def selectImperative(MRS):

    for line in MRS.split('\n'):
        if re.match(r'^INDEX.*', line):
            line = line.split(' ')
#'prop-or-ques' keyword returns the correct params of prepositions except when there is a 'while' for some reason
            if line[5] == 'prop-or-ques':
                if searchForCorrectPrepArgs(MRS):   #if preposition has correct arguments, return true
                    return True
                else:
                    return False
            else:
                return False

#makes sure selected MRS can be used
def selectNoAppositions(MRS):
 
    for line in MRS.split('\n'):
        if re.match(r'^\[ appos.*', line):
            return False
    return True
 
#Searches MRS list and checks to see which one should be returned (MAIN FUNCTION)
def preprocessSemantics(inputString):

    output = collectingMRS (inputString)  
 
    for MRS in output:
        imperative = selectImperative(MRS)
        if imperative:
            usuable = selectNoAppositions(MRS)
            if usuable:
                return (MRS)
                break

#searches MRS for its ARG1 and ARG2 labels, if found, will check if they are the correct labels for the preposition. ARG1 should be a noun or a conj, ARG2 should just be a noun.
def searchForCorrectPrepArgs(MRS):

    args = list()
    prepCount = 0
    for line in MRS.split('\n'):
        if len(line) != 0:
            line = line.split(' ')
            if re.match(r'.*_p<.*', line[1]):
                prepCount = prepCount + 1
                for i in range( 2 ):
                    args.append( findPrepArgs(line)[i] )

    if prepCount == 0:
        return True
    else:
        for i in range(prepCount):
            arg1 = searchForLabel(args[2*i], MRS)
    #Don't want 'while' because it will be thrown out eventually
            if re.match(r'.*_n_.*', arg1) or re.match(r'^_and_.*', arg1) or re.match(r'^_or_.*', arg1) or re.match(r'^_then_.*', arg1):
                arg2 = searchForLabel(args[(2*i)+1], MRS)
                if not re.match(r'.*_n_.*', arg2):
                    return False
            else:
                return False
    return True

#finds the two preposition Arguments in the line
def findPrepArgs(line):
    
    args = list()
    argCounter = 0
    counter = 0
    for word in line:
        counter += 1
        if argCounter == 2:
            break
        elif re.match(r'ARG1:', word) or re.match(r'ARG2:', word):
            argCounter += 1
            args.append(line[counter])

    return args


#searches MRS for label, then returns the word associated with that label
def searchForLabel(lbl, MRS):

    for line in MRS.split('\n'):
        if len(line) > 10 and line[0] == '[':
            line = line.split(' ')
            if re.match(lbl, line[5]) and not re.match(r'.*_q.*', line[1]):
                return line[1]

    return 'ERR'
                    






