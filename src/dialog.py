#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
import preMRS
import postMRS
import reorder
import dialog_action_client as dc
import recognizer_client 
import nltk
import sys
import os
import re
import time
import datetime

home_dir       = "/home/nataliereu/catkin_ws/src/dialog_robot/"
yaml_dir 	   = home_dir + "yaml/"
recipe_dir     = home_dir + "recipe/"
analyzer_dir   = home_dir + "analyzer/"
recognizer_dir = home_dir + "pocketsphinx/"
scripts_dir    = home_dir + "src/"
tree_dir	   = home_dir + "tree/"


class Dialog:


	def __init__(self, script_path):		
		
		rospy.init_node('dialogLoop')
		rospy.on_shutdown(self.cleanup)

		#recognizer_client.pause()	########################################
		
		self.voice = rospy.get_param("~voice", "voice_kal_diphone")
		self.wavepath = rospy.get_param("~wavepath", script_path + "/../sounds")
		self.soundhandle = SoundClient()
		self.dialog_state = '0'
		self.recipe_file = ""
		self.recipe = ""
		self.recipe_name = ""

		corpus_file  = open(recognizer_dir + "recipe.corpus", 'r')
		self.good_strings = list()
		self.recipe_commands = list()
		for line in corpus_file:
			line = line.strip()
			self.good_strings.append(line)
			if re.match(r'^place.*', line):
				self.recipe_commands.append(line)
		corpus_file.close()

		self.recipes = list()
		recipes_names_file = open(recipe_dir + "recipe.names", 'r')
		for line in recipes_names_file:
			line = line.strip()
			line = re.sub(r'_recipe.txt.*$', "", line)
			self.recipes.append(line)
		recipes_names_file.close()

		rospy.sleep(1)
		self.soundhandle.stopAll()

		rospy.Subscriber('/recognizer/output', String, self.dialogLoop)
		rospy.sleep(1)

		readyString = "Ready..."		
		self.soundhandle.say(readyString, self.voice)
		rospy.loginfo(readyString)

		#recognizer_client.continuing()		#################################
		
		
		rospy.sleep(1)		

	def dialogLoop(self,msg):

		rospy.loginfo(msg.data)

		#recognizer_client.pause()	########################################


		response = self.dialog_manager(msg.data)

		if response == "exiting":
			rospy.sleep(1)
			recognizer_client.stop()
			rospy.signal_shutdown("ROSPy Shutdown")	

		rospy.sleep(1)
		
		#recognizer_client.continuing()	###############################################



	def cleanup(self):

		self.soundhandle.stopAll()
		rospy.loginfo("Shutting down dialog node...")


	def dialog_manager(self, inString):

		inString = inString.strip()

		if self.dialog_state == '40':
			return self.dialog_40(inString)

		elif inString not in self.good_strings:
			outString = "Please repeat your command!"
			self.soundhandle.say(outString, self.voice)		
			return 

		else:

			if inString == "cancel":
				outString = "Good Bye!"
				self.soundhandle.say(outString, self.voice)
				return "exiting"

			else:

				if self.dialog_state == '0':
					return self.dialog_00(inString)

				elif self.dialog_state == '21':
					return self.dialog_21(inString)	

				elif self.dialog_state == '22':
					return self.dialog_22(inString)	

				return


	def dialog_00(self, inString):

		rospy.sleep(1)

		if inString in self.recipe_commands:
			return self.dialog_10(inString)

		elif re.match(r'^make recipe.*', inString):
			return self.dialog_20(inString)

		elif re.match(r'^get recipe.*', inString):
			return self.dialog_30(inString)

		elif re.match(r'^pause.*', inString):
			self.soundhandle.say("Taking a break!", self.voice)

			return self.dialog_40(inString)	

		else:
			outString = "Please say a placement command; make a recipe; or get an existing recipe..."
			self.soundhandle.say(outString, self.voice)
			return


	def dialog_10(self, inString):

		inString_new, leftOrRightBread = self.detectLeftRightBread(inString) # hack 1of2 for left/right bread
		inString_updated = self.detectYou(inString_new)
		inString_final = self.detectBase(inString_updated)
		selectedMRS = self.semanticAnalysis (inString_final)	
		parenthesizedCommand = postMRS.parseMRSintoCommand(selectedMRS)
		parenthesizedCommand = self.resolveLeftRightBread(parenthesizedCommand, leftOrRightBread)  # hack 2of2 for left/right bread

		outString = "OK. " + inString 
		self.soundhandle.say(outString, self.voice)

		self.processCommandToYAML(parenthesizedCommand)

		outString = "What is your next command?" 
		self.soundhandle.say(outString, self.voice)

		return 


	def dialog_20(self, inString):

		self.dialog_state = '21'

		outString = "Ok. " + inString + ". What is the name of the recipe? "
		self.soundhandle.say(outString, self.voice)

		return 


	def dialog_21(self, inString):

		self.dialog_state = '22'

		inString_l = inString.split()
		for word in inString_l:
			self.recipe_name += word.strip()
		
		self.recipe_file = recipe_dir + self.recipe_name + "_recipe.txt"
		filestream = open (self.recipe_file, 'w')
		filestream.close()	

		outString = "Ok. A recipe for " + inString + ". Please start with the instructions: "
		self.soundhandle.say(outString, self.voice)
		
		return


	def dialog_22(self, inString):

		if inString =="store recipe":
			self.finish_recipe()

		elif inString in self.recipe_commands:
			self.addCommandToRecipe(inString);
			self.soundhandle.say("What's the next step?", self.voice)

		elif self.isRecipeOfrecipes(inString):		
			self.recipe	= inString	+ '\n'
			outString = "Ok. " + inString + "; what else?"
			self.soundhandle.say(outString, self.voice)			
		
		else:
			outString = "Please repeat your command"
			self.soundhandle.say(outString, self.voice)			

		return


	def dialog_30(self, inString):

		inString = re.sub(r'get recipe for.', "", inString)
		inString_l = inString.split()
		for word in inString_l:
			self.recipe_name += word.strip()

		self.recipe_file = recipe_dir + self.recipe_name + "_recipe.txt"
		filestream = open(self.recipe_file, 'r')
		for line in filestream:
			self.recipe += line + ' '

		outString = "Performing recipe for " + inString
		self.soundhandle.say(outString, self.voice)

		if self.isRecipeOfrecipes(self.recipe):
			self.recipeOfrecipes()

		else:
			self.recipeOfCommands(True)
	
		outString = "What is your next command?"
		self.soundhandle.say(outString, self.voice)		

		self.dialog_state = '0'
		self.recipe_name = ""
		self.recipe_file = ""
		self.recipe = ""
		return


	def dialog_40(self, inString):

		rospy.loginfo("[PASSIVE LISTENING]: " + inString)

		if inString == 'continue':
			self.dialog_state = '0'
			self.soundhandle.say("Break is over!", self.voice)

		else:			
			rospy.sleep(1)
			self.dialog_state = '40'

		return


	def semanticAnalysis (self, inString):

		ace = analyzer_dir + "ace"
		erg = analyzer_dir + "erg-1214-x86-64-0.9.25.dat"
		
		command = "echo " + inString + " | " + ace + " -g " + erg + " -n 10 -Tf"
		allMRS = os.popen(command).read()
		selectedMRS = preMRS.preprocessSemantics(allMRS)
		
		return selectedMRS


	def outputYAML(self, parenthesizedCommand):

		parse = scripts_dir +  "parse_task_tree.py"
		command = "python " + parse + " \"" + parenthesizedCommand + "\""	
		
		outputData = os.popen(command).read()
		yamlData, firstNode = reorder.reorder_YAML(outputData)
		
		return yamlData, firstNode


	def processCommandToYAML (self, command):

		dataYAML, firstNode = self.outputYAML (command)

		rospy.loginfo("Requesting action to server...")

		###########TESTING#########
		responseServer = dc.dialog_action_client(dataYAML, firstNode) ############################
		#############################		

		if responseServer.strip() == "Done":
			rospy.loginfo("DONE: Action performed by robot")			
			ts = time.time()
  			st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H:%M:%S')
  			outfilename = yaml_dir + "output_" + st + ".yaml"
  			self.outputTreeImage(command, st)
		  	outfile = open(outfilename, 'w')
  			outfile.write (dataYAML)
  			outfile.close()	
  		
  		else:
  			rospy.loginfo("Action could not be performed by the robot...")


	def outputTreeImage(self, inString, timeStamp):

		t = nltk.Tree.fromstring(inString)
		outfilename = tree_dir + "output_" + timeStamp + ".ps"
		nltk.draw.tree.TreeView(t)._cframe.print_to_file(outfilename)


	def	addCommandToRecipe(self, inString):

		self.recipe += inString + '\n'
		filestream = open (self.recipe_file, 'a')
		filestream.write(inString + '\n')
		filestream.close()	
		
		inString_new, leftOrRightBread = self.detectLeftRightBread(inString)  # hack 1of2 for left/right bread
		inString_updated = self.detectYou(inString_new)
		inString_final = self.detectBase(inString_updated)
		selectedMRS = self.semanticAnalysis (inString_final)	
		parenthesizedCommand = postMRS.parseMRSintoCommand(selectedMRS)
		parenthesizedCommand = self.resolveLeftRightBread(parenthesizedCommand, leftOrRightBread) # hack 2of2 for left/right bread
		
		inString = "Ok. " + inString 
		self.soundhandle.say(inString, self.voice)
		
		self.processCommandToYAML(parenthesizedCommand)	
		
		return


	def finish_recipe(self):

		if self.isRecipeOfrecipes(self.recipe):
			self.recipeOfrecipes()	

		else:
			self.recipeOfCommands(False)

		outString = "Ok. The recipe for " + self.recipe_name + " is complete and stored. "
		self.soundhandle.say(outString, self.voice)

		self.dialog_state = '0'
		self.recipe_name = ""	
		self.recipe = ""	


	def isRecipeOfrecipes(self, inString):

		inString_l = inString.split()
		for word in inString_l:
			word = word.strip()
			if word == 'and':
				continue
			if word not in self.recipes:
				return False
		return True


	def recipeOfrecipes(self):

		parenthesizedCommand_ALL = "( AND "

		inString_l = self.recipe.split()
		for word in inString_l:
			word = word.strip()
			if word == 'and':
				continue
			filename = recipe_dir + word + '_recipe.txt'
			fileinput = open(filename, 'r')
			parenthesizedCommand_recipe = ""	
			numberLine = 0
			for line in fileinput:
				line = line.strip()
				if len(line) >3:
					numberLine += 1
					line_new, leftOrRightBread = self.detectLeftRightBread(line) # hack 1of2 for left/right bread
					line_updated = self.detectYou(line_new)							
					line_final = self.detectBase(line_updated)
					selectedMRS = self.semanticAnalysis (line_final)	
					parenthesizedCommand = postMRS.parseMRSintoCommand(selectedMRS)
					parenthesizedCommand = self.resolveLeftRightBread(parenthesizedCommand, leftOrRightBread) # hack 2of2 for left/right bread
					if numberLine ==2:
						parenthesizedCommand_recipe = "( THEN " + parenthesizedCommand_recipe
					parenthesizedCommand_recipe += parenthesizedCommand
			parenthesizedCommand_ALL += parenthesizedCommand_recipe
			if numberLine > 1:
				parenthesizedCommand_ALL += " )"
			fileinput.close()
		parenthesizedCommand_ALL += " )"

		self.processCommandToYAML(parenthesizedCommand_ALL)	

		filestream = open(self.recipe_file, 'w')
		filestream.write(self.recipe)
		filestream.close()

		if self.recipe_name not in self.recipes:
			filestream = open(recipe_dir + "recipe.names", 'a')
			outstring = self.recipe_name + '_recipe.txt'
			filestream.write(outstring  + '\n')
			filestream.close()
			self.recipes.append(self.recipe_name)

		self.recipe_file = ""


	def recipeOfCommands(self, withYAML):
	
		parenthesizedCommand_ALL = ""
		numberLine = 0

		self.recipe = self.recipe.split('\n')
		for line in self.recipe:
			line = line.strip()
			if len(line) > 3:
				numberLine += 1
				line_new, leftOrRightBread = self.detectLeftRightBread(line) # hack 1of2 for left/right bread
				line_updated = self.detectYou(line_new)
				line_final = self.detectBase(line_updated)
				selectedMRS = self.semanticAnalysis (line_final)	
				parenthesizedCommand = postMRS.parseMRSintoCommand(selectedMRS)
				parenthesizedCommand = self.resolveLeftRightBread(parenthesizedCommand, leftOrRightBread) # hack 2of2 for left/right bread			
				if numberLine ==2:
					parenthesizedCommand_ALL = "( THEN " + parenthesizedCommand_ALL
				parenthesizedCommand_ALL += parenthesizedCommand

		if numberLine >1:
			parenthesizedCommand_ALL += " )"

		if withYAML:
			self.processCommandToYAML(parenthesizedCommand_ALL)	

		else:		
			ts = time.time()
  			st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H:%M:%S')
  			self.outputTreeImage(parenthesizedCommand_ALL, st);

	  	if self.recipe_name not in self.recipes:
			filestream = open(recipe_dir + "recipe.names", 'a')
			outstring = re.sub(r'^.*recipe/', "", self.recipe_file)
			filestream.write(outstring  + '\n')
			filestream.close()
			self.recipes.append(self.recipe_name)

		self.recipe_file = ""

	def detectYou(self, inString):
		if re.match(r'.*you.*', inString):
			newString = re.sub(r'you', "the robot", inString)
			return newString
		else:
			return inString
	def detectBase(self, inString):
		if re.match(r'.*base.*', inString):
			newString = re.sub(r'base', "green leg", inString)
			return newString
		else:
			return inString
	def detectLeftRightBread(self, inString):

		if re.match(r'.*right bread.*', inString):
			newString = re.sub(r'right bread', "bread", inString)
			return newString, 'right'

		elif re.match(r'.*left bread.*', inString):
			newString = re.sub(r'left bread', "bread", inString)
			return newString, 'left'

		else:
			return inString, 'none'


	def resolveLeftRightBread(self, command, leftOrRight):

		if leftOrRight =='right':
			newString = re.sub(r'bread', 'Right_Bread', command)
			return newString

		elif leftOrRight == 'left':
			newString = re.sub(r'bread', 'Left_Bread', command)
			return newString

		else:
			return command
	


if __name__=="__main__":

	try:
		Dialog(sys.path[0])
		rospy.spin()

	except rospy.ROSInterruptException:
		rospy.loginfo("Dialog finished.")
