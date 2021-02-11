import yaml
import os
import textwrap
import shutil
import argparse

from parser import *

imports = "import m5\nfrom m5.objects import *\nfrom m5.util import *\nimport ConfigParser\nfrom HWAccConfig import *\n\n"

# Parse Arguements
parser = argparse.ArgumentParser(description="SALAM System Builder")

parser.add_argument('-sysName', help="System Name", required=True)
parser.add_argument('-headerName', help="Header Name", required=True)
parser.add_argument('-benchDir', help="Path to Benchmark Directory", required=True)
parser.add_argument('-config', help="Name of Config File", required=True)

args=parser.parse_args()

# Set file information
fileName = args.sysName
# This requires M5_PATH to point to your gem5-SALAM directory
M5_Path = os.getenv('M5_PATH')

workingDirectory = M5_Path + "/"  + args.benchDir

stream = open(workingDirectory + args.config + '.yml', "r")

clusters = []

variables = []

baseAddress = 0x10020000
maxAddress = 0x13ffffff

config = yaml.load_all(stream, Loader=yaml.FullLoader)

# Initial processing
for section in config:
	clusterName = None
	dmas = []
	accs = []
	for k,v in section.items():
		for i in v:
			if "Name" in i:
				clusterName = i['Name']
			if "DMA" in i:
				dmas.append(i)
			if "Accelerator" in i:
				accs.append(i)

	clusters.append(AccCluster(clusterName, dmas, accs, baseAddress, M5_Path))
	baseAddress = clusters[-1].clusterTopAddress + 1

# Write out config file
with open(M5_Path + "/configs/SALAM/" + fileName + ".py", 'w') as f:
	f.write(imports)
	for i in clusters:
		for j in i.genConfig():
			f.write(j + "\n")
		#Add cluster definitions here
		for j in i.dmas:
			for k in j.genConfig():
				f.write("	" + k + "\n")
		for j in i.accs:
			for k in j.genDefinition():
				f.write("	" + k + "\n")
		for j in i.accs:
			for k in j.genConfig():
				f.write("	" + k + "\n")
	f.write("def makeHWAcc(options, system):\n\n")
	for i in clusters:
		f.write("	system." + i.name.lower() + " = AccCluster()" + "\n")
		f.write("	build" + i.name + "(options, system, system." + i.name.lower() + ")\n\n")

begin = None
end = None

# Read in existing header
headerlist = []
for i in clusters:
	try:
		f = open(workingDirectory + i.name  + "_" + args.headerName + ".h", 'r')
		oldHeader = f.readlines()
		for i in range(0,len(oldHeader)):
			if oldHeader[i] == "//BEGIN GENERATED CODE\n":
				begin = i
			elif oldHeader[i] == "//END GENERATED CODE\n" or oldHeader[i] == "//END GENERATED CODE":
				end = i
		del oldHeader[begin:end+1]
		headerlist.append(oldHeader)
	except:
		print("No Header Found")
		emptyList = []
		headerlist.append(emptyList)

# Write out headers
for currentHeader in headerlist:
	for i in clusters:
		with open(workingDirectory + i.name  + "_" + args.headerName + ".h", 'w') as f:
			currentHeader.append("//BEGIN GENERATED CODE\n")
			currentHeader.append("//Cluster: " + j.name.upper() + "\n")
			for j in i.dmas:
				if j.dmaType == "NonCoherent":
					currentHeader.append("//" + j.dmaType + "DMA" + "\n")
					currentHeader.append("#define " + j.name.upper() + "_Flags " + hex(j.address) + "\n")
					currentHeader.append("#define " + j.name.upper() + "_RdAddr " + hex(j.address + 1) + "\n")
					currentHeader.append("#define " + j.name.upper() + "_WrAddr " + hex(j.address + 9) + "\n")
					currentHeader.append("#define " + j.name.upper() + "_CopyLen " + hex(j.address + 17) + "\n")
				elif j.dmaType == "Stream":
					currentHeader.append("//" + j.dmaType + "DMA" + "\n")
					currentHeader.append("#define " + j.name.upper() + "_Flags " + hex(j.address) + "\n")
					currentHeader.append("#define " + j.name.upper() + "_RdAddr " + hex(j.address + 4) + "\n")
					currentHeader.append("#define " + j.name.upper() + "_WrAddr " + hex(j.address + 12) + "\n")
					currentHeader.append("#define " + j.name.upper() + "_RdFrameSize " + hex(j.address + 20) + "\n")
					currentHeader.append("#define " + j.name.upper() + "_NumRdFrames " + hex(j.address + 24) + "\n")
					currentHeader.append("#define " + j.name.upper() + "_RdFrameBufSize " + hex(j.address + 25) + "\n")
					currentHeader.append("#define " + j.name.upper() + "_WrFrameSize " + hex(j.address + 26) + "\n")
					currentHeader.append("#define " + j.name.upper() + "_NumWrFrames " + hex(j.address + 30) + "\n")
					currentHeader.append("#define " + j.name.upper() + "_WrFrameBufSize " + hex(j.address + 31) + "\n")
					currentHeader.append("#define " + j.name.upper() + "_Stream " + hex(j.address + 32) + "\n")
			for j in i.accs:
				currentHeader.append("//Accelerator: " + j.name.upper() + "\n")
				currentHeader.append("#define " + j.name.upper() + " " + hex(j.address) + "\n")
				for k in j.variables:
					currentHeader.append("#define " + k.name + " " + hex(k.address) + "\n")
				for k in j.streamVariables:
					currentHeader.append("#define " + k.name + " " + hex(k.address) + "\n")
			currentHeader.append("//END GENERATED CODE")
			f.writelines(currentHeader)
			currentHeader = []

# print(workingDirectory + fileName + ".h")

shutil.copyfile("fs_template.py", M5_Path + "/configs/SALAM/fs_" + fileName + ".py")

# Generate full system
f = open(M5_Path + "/configs/SALAM/fs_" + fileName + ".py", "r")

fullSystem = f.readlines()

fullSystem[69] = "import " + fileName
fullSystem[239] = "		" + fileName + ".makeHWAcc(options, test_sys)"

f = open(M5_Path + "/configs/SALAM/fs_" + fileName + ".py", "w")

f.writelines(fullSystem)

# print(hex(clusters[-1].clusterTopAddress))

if(clusters[-1].clusterTopAddress>maxAddress):
    print("WARNING: Address range is greater than defined for gem5")