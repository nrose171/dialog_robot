import re
import sys
 
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
 
def selectImperative(MRS):
 
    for line in MRS.split('\n'):
        if re.match(r'^INDEX.*', line):
            line = line.split(' ')
            if line[5] == 'prop-or-ques':
                return True
            else:
                return False
 
def selectNoAppositions(MRS):
 
    for line in MRS.split('\n'):
        if re.match(r'^\[ appos.*', line):
            return False
    return True
 
def preprocessSemantics(inputString):

    output = collectingMRS (inputString)  
 
    for MRS in output:
        imperative = selectImperative(MRS)
        if imperative:
            usuable = selectNoAppositions(MRS)
            if usuable:
                return (MRS)
                break
