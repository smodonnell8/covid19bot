#! /usr/bin/env python

import argparse
import os
import re

parser = argparse.ArgumentParser(description='Update the switch statements, audio comments, and number of lines')
parser.add_argument('module', metavar='fn', type=str, help='Name of the module js file')
parser.add_argument('--trust', action='store_true', help='replace content in original module file')
args = parser.parse_args()

with open(args.module) as moduleFile:
	modFileContent = moduleFile.read()
	module = modFileContent.split('\n')

with open('updated_' + args.module,'w') as moduleFile:
	# Get Module Number
	currentLine = 0;
	while 'module_num' not in module[currentLine]:
		moduleFile.write(module[currentLine] + '\n')
		currentLine += 1
	moduleNum = int(module[currentLine][module[currentLine].index('=')+1:module[currentLine].index(';')])

	# Get module text
	p = re.compile(r"text_url.*=.*?'(.*?)'")
	path = p.findall(modFileContent)[0]
	with open('../' + path + str(moduleNum) + '.txt') as txtFile:
		speech = txtFile.read().split('\n')

	# Update lines_of_text
	while 'lines_of_text' not in module[currentLine]:
		moduleFile.write(module[currentLine] + '\n')
		currentLine += 1
	module[currentLine] = re.sub(r'=\s*\d+\s*;','= ' + str(len(speech)) + ';',module[currentLine])
	moduleFile.write(module[currentLine] + '\n')
	currentLine += 1

	# Update audio comments and switch statements
	initFnDef = 'async function init() {'
	remainingContent = '\n'.join(module[currentLine:]).split(initFnDef)
	moduleFile.write(remainingContent[0])
	audioComment = re.findall( re.compile(r'\s*(// \d+:.*?)\n\s*?play',re.M) ,remainingContent[1])
	audioInd = 0
	caseInd = 0;
	currentLine += remainingContent[0].count('\n')
	while currentLine<len(module):
		# Update audio comments
		if audioInd<len(audioComment) and audioComment[audioInd] in module[currentLine]:
			moduleFile.write(module[currentLine][:module[currentLine].index('/')])
			moduleFile.write('// ' + str(audioInd) + ': ' + speech[audioInd] + '\n')
			audioInd += 1
		# Update switch statements
		elif '            case ' in module[currentLine]:
			moduleFile.write('            case ' + str(caseInd) + ':\n')
			caseInd += 1
		else:
			moduleFile.write(module[currentLine] + '\n')
		currentLine += 1

if args.trust:
	with open('updated_' + args.module) as moduleFile:
		module = moduleFile.read()
	with open(args.module,'w') as moduleFile:
		moduleFile.write(module[:-1])