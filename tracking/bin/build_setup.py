#! /usr/bin/env python3

'''
build_setup:
- Configures the build environment for working with Panther in the AIIL

@author
- Timothy Wiley
'''

import argparse
import os
import pathlib
import re
import shutil
import stat
import subprocess
import sys

import utils.config as cfg
from utils.echo import (
    print_error,
    print_status,
    print_warning,
    print_progress,
    print_subitem,
    query_user,
    query_yes_no
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
AIIL_PANTHER_CHECKOUT_DIR = os.path.abspath(binDir + "/../")
HUSARION_CHECKOUT_DIR = os.path.abspath(AIIL_PANTHER_CHECKOUT_DIR + "/../" + cfg.husarion_workspace)

# Global setup type parameters
setupRobot      = False
setupRobotName  = None
setupComp       = False
setupCompName   = None
setupDocker     = False

def installSoftware(software, ros=False, rosversion='none'):
    # Load this here to avoid script delays
    print_status("Installing Software")
    from utils.aptinstall import (aptinstall, aptcheck, aptinstallcmdline)

    toInstall = []

    # Load Software to install
    counter = 0
    for pkg in software:
        progress = round(counter / len(software) * 100)

        if software.getboolean(pkg):
            # If ROS package - prepend with ros package name
            if ros:
                pkg = "ros-" + rosversion + "-" + pkg

            print_progress("Installing " + pkg, progress)
            installed = aptcheck(pkg)
            if installed:
                print_progress("Package " + pkg + " installed. Skipping", progress)
            else :
                print_progress("Marking " + pkg + " for installation", progress)
                toInstall.append(pkg)
        else :
            print_progress("Skipping package - configuration file disables install of " + pkg, progress)
        counter += 1
    print_progress("Installation checked", 100)

    if len(toInstall) > 0:
        print_warning("Installation will most likely request admin privillages to complete installation")
        print_status("Commencing Installation")
        print_subitem("Packages: " + " ".join(toInstall))
        aptinstallcmdline(toInstall)
    else :
        print_subitem("No packages to install")


def replaceAuthKeys():
    localAuthFile = ssh.authKeyFile()
    target = ssh.sshDirectory
    shutil.copy(localAuthFile, target)

def setupBash():
    print_status("Configuring Bash Environment")
    rbbBashFile = os.path.abspath(AIIL_PANTHER_CHECKOUT_DIR + "/.bashrc")
    bashrcFile = os.path.expanduser("~/.bashrc")
    
    # Create redbackbots source file
    templateBashrc = os.path.abspath(AIIL_PANTHER_CHECKOUT_DIR + "/config/bashrc")
    shutil.copy(templateBashrc, rbbBashFile)
    print_subitem("Copying " + templateBashrc + " to " + rbbBashFile)

    # Set Husarion workspace based on device type
    husDir = HUSARION_CHECKOUT_DIR
    if setupRobot:
        husDir = os.path.expanduser("~/" + cfg.husarion_workspace)
    
    # Append additional dynamic elements
    print_subitem("Updating " + rbbBashFile)
    bashFile = open(rbbBashFile, "a+")
    bashFile.write("\n")
    bashFile.write("# Husarion Panther Environment Settings\n")
    bashFile.write("export AIIL_PANTHER_CHECKOUT_DIR=" + AIIL_PANTHER_CHECKOUT_DIR + "\n")
    bashFile.write("export HUSARION_CHECKOUT_DIR=" + husDir + "\n")
    bashFile.write("export PATH=\"$AIIL_PANTHER_CHECKOUT_DIR/bin:$PATH\"\n")
    bashFile.write("\n")
    bashFile.write("# Source our workspace\n")
    bashFile.write("source $AIIL_PANTHER_CHECKOUT_DIR/noetic_workspace/devel/setup.bash\n")
    bashFile.close()

    # Query if user wishes to automatically source panther
    query = False
    if setupRobot:
        query = True
    elif setupComp:
        query = query_yes_no("Configure root ~/.bashrc to automatically source ROSbot workspace?")

    if query:
        # Search for ssh config block
        found = grep(bashrcFile, "Husarion Panther")

        # If not found, append redbackbots to root bashrc
        if not found:
            print_subitem("Updating ~/.bashrc file")
            bashFile = open(bashrcFile, "a+")
            bashFile.write("\n")
            bashFile.write("# Husarion Panther Bashrc Source\n")
            bashFile.write("source " + rbbBashFile + "\n")
            bashFile.close()
    else :
        print_warning("Automatic source NOT enabled - will require manual source on every use")


    print_warning(".bashrc configuration has changed." +
                  "source the new bashrc file (using below) and re-run the setup\n" + 
                  "source " + rbbBashFile)
    print_warning("HUSARION_CHECKOUT_DIR is set to: '" + HUSARION_CHECKOUT_DIR + "'\n. If this is not correct. Then change before relaunch")
    exit()

def setupBuildHusarion(configRobots, configComputers, configSoftware):
    # Create and configure CMake
    print_status("Configure & Build Husarion Workspace")

    # Setup Husarion Repos
    setupHusarionRepos(configRobots, configComputers, configSoftware)

    # Execute standalone catkin_make script for husarion_ws
    print_status("Building Husarion Workspace")
    binDir = cfg.binDirectory()
    script = binDir + "/catkin/catkin_make_husarion"
    shell.exec(script, hideOutput=False)

def setupBuildRosbot(configRobots, configComputers):
    print_subitem("Configure & Build AIIL Panther Workspace")

    # Setup and build go together.
    # If no devel, then run setup version to overlay on husarion workspace
    # If existing devel, then run normal build
    # Both are executed through standalone scripts

    # Check for exiting devel
    noetic_workspace = cfg.rosbotNoeticWorkspace()
    develDir = noetic_workspace + "/devel"
    command = ""
    if not os.path.exists(develDir):
        # Run initialisation script
        command = cfg.catkin_init_aiil()
    else :
        # Run compilation script
        command = cfg.catkin_make_aiil()
    
    # Execute script
    shell.exec(command, hideOutput=False)

def setupGit():
    print_status("Checking your Git Configuration")
    gitParams = ["user.name", "user.email", "pull.rebase"]
    for param in gitParams:
        value = shell.capture(["git config " + param], emptyFail=True)
        print("git config " + param + " = " + value)

    query = query_yes_no("Are these details correct?")
    if not query:
        print_status("Updating Git details")
        for param in gitParams:
            query = query_yes_no("Update " + param + "?")
            if query:
                value = query_user("New value for " + param + ": ")
                command = "git config --global " + param + " " + value
                print_status("Executing: " + command)
                shell.exec(command, hideOutput=False)
    else:
        print("Skipping git configuration - details are correct")

def setupHusarionRepos(configRobots, configComputers, configSoftware):
    print_status("Setting up Husarion ROS Repositories")

    # Repos Config
    configRepos = cfg.loadConfigFile(configDir + "/repos.cfg")

    # Check for Husarion Repository
    develDir = HUSARION_CHECKOUT_DIR + "/devel"
    if not os.path.exists(develDir):
        # Initialise Husarion Directory
        rosversion = 'noetic'
        if setupRobot:
            rosversion = configRobots[setupRobotName]['rosversion']
        elif setupComp:
            rosversion = configComputers[setupCompName]['rosversion']
        
        # Ensure directory exists
        if not os.path.exists(HUSARION_CHECKOUT_DIR):
            os.makedirs(HUSARION_CHECKOUT_DIR)

        # Run initialisation script
        command = cfg.catkin_init_husarion() + " " + rosversion
        shell.exec(command, hideOutput=False)

    # Iterate through each repo
    for repo in configRepos.sections():
        if configRepos[repo]['type'] == cfg.husarion_workspace:
            print_subitem("Repo: " + repo)

            # Check if exists
            wsDir = HUSARION_CHECKOUT_DIR + "/src/" + repo
            dirExists = os.path.exists(wsDir)

            # Check if git version
            gitDir = wsDir + "/.git"
            gitExists = os.path.exists(gitDir)
            print_subitem("\twsDir: " + wsDir)
            print_subitem("\tgitDir: " + gitDir)

            # Clone/Update
            if gitExists:
                # Update
                print_subitem("Updating Repo: " + repo)
                os.chdir(wsDir)
                command = "git pull"
                shell.exec(command, hideOutput=False)
                os.chdir(AIIL_PANTHER_CHECKOUT_DIR)

            else :
                print_subitem("Cloning Repo: " + repo)
                if dirExists:
                    print_subitem("\tMoving old repo out and replacing with cloned repo")
                    # Move out-of-way
                    saveDir = HUSARION_CHECKOUT_DIR + "/src/" + "orig_image/."
                    if not os.path.exists(saveDir):
                        os.makedirs(saveDir)
                    shutil.move(wsDir, saveDir)
                    # Ensure catkin_ignore set
                    shell.exec("touch " + saveDir + "/CATKIN_IGNORE")

                # Clone
                os.chdir(HUSARION_CHECKOUT_DIR + "/src")
                command = "git clone " + configRepos[repo]['giturl'] + " " + repo
                shell.exec(command, hideOutput=False)
                os.chdir(AIIL_PANTHER_CHECKOUT_DIR)
                
                # Change Branch if necessary
                if configRepos.has_option(repo, 'branch'):
                    print_subitem("Switching to branch: " + configRepos[repo]['branch'])
                    os.chdir(HUSARION_CHECKOUT_DIR + "/src/" + repo)
                    command = "git checkout --track origin/" + configRepos[repo]['branch']
                    shell.exec(command, hideOutput=False)
                    os.chdir(AIIL_PANTHER_CHECKOUT_DIR)

def setupHostsAliases(configRobots, configComputers):
    print_status("Configuring /etc/hosts with manage_hosts.py tool")
    print_warning("Manage Hosts program must be run as an administrator. You may ba asked for an admin password")
    manageProgram = cfg.binDirectory() + "/manage_hosts.py"
    commandBase = "sudo " + manageProgram

    if setupRobot:
        command = commandBase + " -r " + setupRobotName
        shell.exec(command, hideOutput=False)
    elif setupComp:
        command = commandBase + " -c " + setupCompName
        shell.exec(command, hideOutput=False)
    elif setupDocker:
        command = commandBase + " -g"
        shell.exec(command, hideOutput=False)

def setupSSHConfig(configRobots, configComputers):
    print_status("Setting up SSH Config")
    sshDir = ssh.sshDirectory
    sshConfig = ssh.sshConfigFile
    print_subitem("SSH Dir: " + sshDir)
    if not os.path.exists(sshDir):
        print_subitem("Making Root (~/.ssh) SSH Directory")
        os.makedirs(sshDir)
        os.chmod(sshDir, 0o700)
    if not os.path.exists(sshConfig):
        print_subitem("Creating Root (~/.ssh) SSH Config File")
        pathlib.Path(sshConfig).touch()
        os.chmod(sshConfig, 0o700)

    # Create the ssh config file in local config folder
    localSSHFile = cfg.sshLocalConfig()
    sshLocalFile = open(localSSHFile, "w+")
    sshLocalFile.write("# Automatically generated ssh config from build_setup.py.\n")
    sshLocalFile.write("# DO NOT MODIFY DIRECTLY\n\n")
    for robot in configRobots.sections():
        print_subitem("\tAdding " + robot)
        setupSSHConfigAdd(sshLocalFile, robot, configRobots[robot]["ip"])
    for comp in configComputers.sections():
        if comp != 'general':
            print_subitem("\tAdding " + comp)
            setupSSHConfigAdd(sshLocalFile, comp, configComputers[comp]["ip"])
    sshLocalFile.close()

    # Copy Local SSH config to destination - and set permissions
    print_subitem("Copying Local SSH to ~/.ssh")
    shutil.copy(localSSHFile, ssh.sshConfigDestination)
    #os.chmod(ssh.sshConfigDestination, 0o600)
    os.chmod(ssh.sshConfigDestination, stat.S_IRUSR | stat.S_IWUSR)

    # Search for ssh config block
    found = grep(sshConfig, "Husarion Panther")
    
    # If not found, append new block
    robots = cfg.getKeys(configRobots)
    if not found:
        print_subitem("Adding Husarion Panther robots & computers to SSH Config (~/.ssh/config)")

        # create backup file - as Include must go at the TOP of the .ssh/config
        sshConfigBak = sshConfig + ".bak"
        shutil.copy(sshConfig, sshConfigBak)

        sshFile = open(sshConfig, "w")
        sshFile.write("\n")
        sshFile.write("# Husarion Panther SSH Config\n")
        sshFile.write("Include " + ssh.sshConfigDestination + "\n")
        sshFile.write("\n")
        sshBak = open(sshConfigBak)
        for line in sshBak:
            sshFile.write(line)
        sshFile.close()

def setupSSHConfigAdd(sshFile, name, ip):
    sshFile.write("Host " + name + "\n")
    sshFile.write("    Hostname " + ssh.getRobotIP(ip) + "\n")
    sshFile.write("    CheckHostIP no\n")
    sshFile.write("    User husarion\n")
    sshFile.write("    StrictHostKeyChecking no\n")
    sshFile.write("\n")

def setupSSHKeys():
    print_status("Setting up SSH Keys")
    pubkeyFile = ssh.pubicKeyFile
    authkeyFile = ssh.authKeyFile()
    print_subitem("pub key file: " + pubkeyFile)
    print_subitem("authorized_keys file: " + authkeyFile)

    # Check SSH Args
    check_command = 'ssh-keygen -l -f ' + pubkeyFile
    retCode = shell.exec(check_command, True)
    if not retCode:
        print_subitem("No id_rsa.pub file - generating")
        print_error("NOT IMPLEMENTED")
    else:
        print_subitem("Using existing id_rsa.pub file")
    
    # Locate pub 
    if retCode:
        checkAuth = ssh.checkAuthKey()
        if not checkAuth:
            print_status("Adding key to authorized keys")
            command = "cat " + pubkeyFile + " >> " + authkeyFile
            shell.exec(command)
        else:
            print_subitem("Key already in authorized keys")

def _main_setup():
    # Check for env configuration, and configure bash
    tmpEnv = cfg.getEnvParameter("AIIL_PANTHER_CHECKOUT_DIR", check=True)
    bashLoaded = tmpEnv != ""

    # Setup bash script
    if not bashLoaded:
        print_warning("bashrc has not been configured. Either reload or configure bash")
        query = query_yes_no("Configure Bash? (Includes ROS envrionment parameters")
        if query:
            setupBash()
        else:
            print_error("Cannot continue without bashrc being configured")
        exit()
    print()

    # Check for environment variables existing
    AIIL_PANTHER_CHECKOUT_DIR = cfg.getEnvParameter("AIIL_PANTHER_CHECKOUT_DIR")
    HUSARION_CHECKOUT_DIR = cfg.getEnvParameter("HUSARION_CHECKOUT_DIR")

    # Load configs
    config = cfg.loadConfigFile(configDir + "/software.cfg")
    configSoftware = config['Software']
    configROSSoftware = config['Ros']
    configRobots = cfg.loadConfigFile(configDir + "/robots.cfg")
    configComputers = cfg.loadConfigFile(configDir + "/computers.cfg")
    print_subitem("Configured Robots:")
    [print_subitem("\t* " + robot) for robot in cfg.getKeys(configRobots)]
    print_subitem("Configured Computers:")
    [print_subitem("\t* " + comp) for comp in cfg.getKeys(configComputers)]
    print()
   
    # Check robot/computer exists
    if setupRobot:
        if not configRobots.has_section(setupRobotName):
            print_error("No Configuration parameters for robot: " + setupRobotName)
            exit()
    if setupComp:
        if not configComputers.has_section(setupCompName):
            print_error("No Configuration parameters for robot: " + setupCompName)
            exit()

    # Install software
    query = query_yes_no("Install General Software?")
    if query:
        installSoftware(configSoftware)
    print()

    query = query_yes_no("Install ROS Specific Additional Packages?")
    if query:
        rosversion='none'
        if setupRobot:
            rosversion=configRobots[setupRobotName]["rosversion"]
        else :
            rosversion=configComputers[setupCompName]["rosversion"]
        installSoftware(configROSSoftware, ros=True, rosversion=rosversion)
    print()

    # Setup Git
    if setupComp:
        query = query_yes_no("(Computer only) Configure Git properties?")
        if query:
            setupGit()
        print()

    # Setup SSH Config
    query = query_yes_no("Setup ~/.ssh/config file?")
    if query:
        setupSSHConfig(configRobots, configComputers)
    print()

    # Setup /etc/hosts Aliases
    query = query_yes_no("Setup /etc/hosts alias?")
    if query:
        setupHostsAliases(configRobots, configComputers)
    print()

    # Setup SSH Keys
    if setupComp:
        query = query_yes_no("(Computer only) Setup SSH Keys?")
        if query:
            setupSSHKeys()
        print()

    # Repalce authorised keys (on robot only)
    if setupRobot:
        query = query_yes_no("(Robot only) Replace authorised SSH keys?")
        if query:
            replaceAuthKeys()
        print()

    # Configure Husarion Git Repositories
    query = query_yes_no("Configure & Build Repositories?")
    if query:
        setupBuildHusarion(configRobots, configComputers, config)
        setupBuildRosbot(configRobots, configComputers)
    print()

def _main_setupDocker():
    print_status("Setup for Docker environment")

    # Check for environment variables existing, otherwise set manually
    tmpEnv = cfg.getEnvParameter("AIIL_PANTHER_CHECKOUT_DIR", check=True)
    if tmpEnv != "":
        AIIL_PANTHER_CHECKOUT_DIR = tmpEnv
    
    tmpEnv = cfg.getEnvParameter("HUSARION_CHECKOUT_DIR", check=True)
    if tmpEnv != "":
        HUSARION_CHECKOUT_DIR = tmpEnv

    # Load configs
    configRobots = cfg.loadConfigFile(configDir + "/robots.cfg")
    configComputers = cfg.loadConfigFile(configDir + "/computers.cfg")

    # Setup SSH COnfig
    setupSSHConfig(configRobots, configComputers)

    # Setup hosts file
    #setupHostsAliases(configRobots, configComputers)

# Main entry point
if __name__ == "__main__":
    print_status("Commencing Build Setup")
    print_warning("If you are unsure about any answer, always select yes (Y)")
   
    # Process args
    parser = argparse.ArgumentParser(description='Setup the build for a robot/computer')
    parser.add_argument('-r', dest="robot", type=str, help='Robot to setup Build for (conflicts with "computer")')
    parser.add_argument('-c', dest="computer", type=str, help='Computer to setup Build for (conflicts with "robot")')
    parser.add_argument('-g', dest="general", action='store_true', help='Configure a General Computer/Device')
    parser.add_argument('-d', dest="docker", action='store_true', help='Configure a Docker environment')
    args = parser.parse_args()
    setupCount = 0
    if (args.robot is not None):
        setupRobot = True
        setupRobotName = args.robot
        setupCount += 1
    if (args.computer is not None):
        setupComp = True
        setupCompName = args.computer
        setupCount += 1
    if args.general:
        setupComp = True
        setupCompName = 'general'
        setupCount += 1
    if args.docker:
        setupDocker = True
        setupCount += 1
    if (not setupRobot) and (not setupComp) and (not setupDocker):
        print_error("No selection of Robot, Computer or Docker")
        parser.print_help()
        exit()
    if setupCount > 1:
        print_error("Cannot setup for multiple forms, choose only one")
        parser.print_help()
        exit()

    if setupRobot:
        print_status("Running Setup for ROBOT: " + setupRobotName)
    elif setupComp:
        print_status("Running Setup for COMPUTER " + setupCompName)


    # Loading paths
    print_subitem("AIIL_PANTHER_CHECKOUT_DIR = " + AIIL_PANTHER_CHECKOUT_DIR)
    print_subitem("HUSARION_CHECKOUT_DIR = " + HUSARION_CHECKOUT_DIR)
    print_subitem("Bin Directory = " + binDir)
    print_subitem("Config Directory = " + configDir)
    print()

    # Use setup based on robot/comp or docker
    if setupComp or setupRobot:
        _main_setup()
    elif setupDocker:
        _main_setupDocker()

    print_status("Build Setup Complete")

