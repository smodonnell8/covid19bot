#! /usr/bin/env python

# -*- coding: utf-8 -*-
"""
Created on Wed Oct  4 10:12:19 2017

run this script before running main script teachbot.py
This script creates all necessary mp3s 

@author: jerrying
"""

from __future__ import print_function
import os
import sys
import argparse
import math
from gtts import gTTS
from importlib import reload

parser = argparse.ArgumentParser(description='Synthesize text file into audio files of speech.')
parser.add_argument('text_file', metavar='txt', type=str, help='Name of the text file from which to extract speech snippets')
parser.add_argument('dest_dir', metavar='dir', type=str, help='Directory to add audio files')
args = parser.parse_args()
if args.dest_dir.endswith('/'):
	args.dest_dir = args.dest_dir[:-1]

newpath = args.dest_dir;
if not os.path.exists(newpath): #makes folder for name sounds
    os.makedirs(newpath)

textfile = open(args.text_file)
lines = textfile.read().split('\n')

linepath = args.dest_dir + '/line'
progress = 0
print (str(progress) + '%', end="\r")
sys.stdout.flush()
for x in range(0,len(lines)):
	print(linepath + str(x) + '.mp3')
	tts = gTTS(text = lines[x], lang='en')
	tts.save(linepath + str(x) + '.mp3')
	if math.floor(100*x/len(lines))>progress:
		progress = math.floor(100*x/len(lines))
		print (str(progress) + '%', end="\r")
		sys.stdout.flush()

def response_popper():
    global response_req
    response_req.pop(0)
