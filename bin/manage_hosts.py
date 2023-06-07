#! /usr/bin/env python3

'''
build_setup:
- Configures the build environment for working with ROSBot and ROS Setup in the AIIL

@author
- Timothy Wiley
'''

import argparse
import os
from pathlib import Path
import re
import shutil
import sys

import utils.config as cfg
from utils.echo import (
    print_error,
    print_status,
    print_warning,
    print_subitem,
)
from utils.grep import (
    grep
)
import utils.shellEscape as shell
import utils.sshscp as ssh


# Global Parameters
fullSetupPath = os.path.abspath(__file__)
binDir = os.path.dirname(fullSetupPath)
configDir = os.path.abspath(binDir + "/../config/")

# Host file
hostsFile       = "/etc/hosts"
hostsFileBak    = hostsFile + ".bak"

# Global setup type parameters
setupRobotName  = None
setupCompName   = None

def aliasLine(alias, ip):
    return ip + "    " + alias

def appendNew(hostFileContents, alias, ip):
    # Search for existing block of ROSbot elements
    strKey = "ROSBot"
    if any(strKey in word for word in hostFileContents):
        # Iterate through insert new line
        added = False
        for index in range(len(hostFileContents)):
            if (not added) and re.search(strKey, hostFileContents[index]):
                # Found block - append new line
                hostFileContents.insert(index+1, aliasLine(alias, ip) + "\n")
                added = True
    else:
        # Append to end
        hostFileContents.append("\n")
        hostFileContents.append("# ROSBot hosts\n")
        hostFileContents.append(aliasLine(alias, ip) + "\n")
        hostFileContents.append("\n")

    return

def loadHostsFile():
    fh = open(hostsFile)
    lines = fh.readlines()
    fh.close()
    return lines

def outputHostsFile(hostFileContents):
    print(''.join(hostFileContents))
    fh = open(hostsFile, "w")
    for line in hostFileContents:
        fh.write(line)
    fh.close()

def processAlias(hostFileContents, alias, ip):
    print_subitem("Processing alias: " + alias + " to ip: " + ip)

    # Check ordering of alias/ip
    if re.search("^[0-9]+[.][0-9]+[.][0-9]+[.][0-9]+$", ip):
        # If present, modify in-place
        # If not present, append new alias to ROSBot configuration location
        if any(alias in word for word in hostFileContents):
            print_status("Modfyig existing alias")
            replaceExisting(hostFileContents, alias, ip)
        else :
            print_status("Adding new alias")
            appendNew(hostFileContents, alias, ip)
    else:
        print_error("IP is the wrong format, no IP address: " + ip)

def replaceExisting(hostFileContents, alias, ip):
    # Iterate through backup file and insert new line
    added = False
    for index in range(len(hostFileContents)):
        if (not added) and re.search(alias, hostFileContents[index]):
            # Found replace entry
            hostFileContents[index] = aliasLine(alias, ip) + "\n"
            added = True
    return

if __name__ == "__main__":
    print_status("Manage /etc/hosts for ROSBot")
    print_warning("To function, this program must be run with administrator privilages")
    print()

    # Parse args for managing the host
    parser = argparse.ArgumentParser(description='Manage /etc/hosts for ROSBot library. Updates the given alias to the given ip in the /etc/hosts file. A backup of the file is made in /etc/hosts.bak')
    parser.add_argument('-r', dest="robot", type=str, help='Robot to setup Build for (conflicts with "computer")')
    parser.add_argument('-c', dest="computer", type=str, help='Computer to setup Build for (conflicts with "robot")')
    parser.add_argument('-g', dest="general", action='store_true', help='Configure a General Computer/Device')

    # parser.add_argument('alias', type=str, help='Alias of the IP host')
    # parser.add_argument('ip', type=str, help='IP address for the alias')
    args = parser.parse_args()
    # alias = args.alias
    # ip = args.ip
    if (args.robot is not None):
        setupRobotName = args.robot
    if (args.computer is not None):
        setupCompName = args.computer
    if args.general:
        setupCompName = 'general'
    print_status("Parameters")

    # Configuration
    configRobots = cfg.loadConfigFile(configDir + "/robots.cfg")
    configComputers = cfg.loadConfigFile(configDir + "/computers.cfg")

    # Create Hosts backup
    print_subitem("Creating backup")
    print_warning("If something goes wrong, replace /etc/hosts with the backup or with the below information. CURRENT STATE OF /etc/hosts")
    print()
    shell.exec("cat " + hostsFile, hideOutput = False)
    shutil.copy(hostsFile, hostsFileBak)

    # Load host file
    hostFileContents = loadHostsFile()

    # Process robot aliases from configuration
    for robot in configRobots.sections():
        if robot != setupRobotName:
            print_subitem("Updating robot: " + robot)
            processAlias(hostFileContents, robot, ssh.getRobotIP(configRobots[robot]["ip"]))

    for comp in configComputers.sections():
        if (comp != setupCompName) and (comp != 'general'):
            print_subitem("Updating computer: " + comp)
            processAlias(hostFileContents, comp, ssh.getRobotIP(configComputers[comp]["ip"]))

    # Output new Host Files
    outputHostsFile(hostFileContents)

    exit()

