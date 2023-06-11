#! /usr/bin/env python3

'''
Wrappers for SSH and SCP useage
- Work with SSH config files
- Run SSH and SCP commands
'''

import os

import utils.config as cfg
import utils.echo as echo
import utils.grep as grep
import utils.shellEscape as shell

'''
Static names/variables
'''
pubicKeyFile = os.path.expanduser('~/.ssh/id_rsa.pub')
rosbot_default_subnet = "192.168.100"
sshDirectory = os.path.expanduser("~/.ssh")
sshConfigFile = sshDirectory + "/config"
sshConfigDestination = sshDirectory + "/ssh_config_rosbot"

'''
Long form commands
'''
DELETE_COMMAND = "rm -rf "
RSYNC_COMMAND = "rsync --archive --compress --partial --progress "
RSYNC_CHMOD = " --chmod "
RSYNC_EXCLUDE = (" --exclude .git " + 
                " --exclude .bashrc" +
                " --exclude devel" +
                " --exclude install" +
                " --exclude build" +
                " --exclude log" +
                " --exclude .catkin_workspace" +
                " --exclude *.swp" +
                " --exclude bin/utils/__pycache__ ")
RSYNC_SUDO = " --rsync-path=\"sudo rsync\" "
SUDO = "sudo "

'''
Authorisation key file
'''
def authKeyFile():
    return cfg.configDirectory() + "/authorized_keys"

'''
Check is the users public key in in the authorized key file
'''
def checkAuthKey():
    check_command = 'ssh-keygen -l -f ' + pubicKeyFile
    capture = shell.capture(check_command)
    keyid = capture.split(' ')[2]
    return grep.grep(authKeyFile(), keyid)

'''
Return the full IP for a robot using subnet and robot ip
'''
def getRobotIP(ip):
    return rosbot_default_subnet + "." + ip

'''
Class for managing SSH and SCP commands
'''
class SshScp:
    def __init__(self, debug=False):
        self.debug = debug
        self.hostname = ''
        self.hideOutput = False

    def setHostname(self, robot):
        self.hostname = "husarion@" + robot

    def setHideOutput(self, hide):
        self.hideOutput = hide
    
    def deleteFiles(self, dest, sudo=False):
        fullCommand = DELETE_COMMAND
        prepend = ""
        if sudo:
            prepend += SUDO
        fullCommand = prepend + fullCommand + dest
        echo.print_subitem(fullCommand)
        
        return self.sshCommand(fullCommand)

    def rsyncFiles(self, src, dest, sudo=False, chmod=None):
        fullCommand = RSYNC_COMMAND
        append = " "
        if sudo:
            append += RSYNC_SUDO
        if chmod is not None:
            append += RSYNC_CHMOD + chmod + " "
        append += RSYNC_EXCLUDE
        fullCommand = fullCommand + append + src + " " + self.hostname + ":" + dest
        echo.print_subitem(fullCommand)
        
        success = True
        if not self.debug:
            success = shell.exec(fullCommand, self.hideOutput)
        return success

    def scpFile(self, src, dest):
        fullCommand = "scp " + src + " " + self.hostname + ":" + dest
        echo.print_subitem(fullCommand)
        
        success = True
        if not self.debug:
            success = shell.exec(fullCommand, self.hideOutput)
        return success

    def sshCommand(self, command):
        fullCommand = "ssh " + self.hostname + " \"" + command + "\""
        echo.print_subitem(fullCommand)
        
        success = True
        if not self.debug:
            success = shell.exec(fullCommand, self.hideOutput)
        return success
    
    def makePath(self, path):
        command = "mkdir -p " + path
        return self.sshCommand(command)
