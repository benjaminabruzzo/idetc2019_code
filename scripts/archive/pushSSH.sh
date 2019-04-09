#!/bin/bash
# call using: sh pushSSH.sh neptune
scp ~/.ssh/id_rsa.pub tbrl@192.168.100.25:~/$1.pub
