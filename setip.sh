#!/bin/bash

echo "Assuming connected to the router";
echo "Choose which drone is connected (1/2):";
read -r drone

case $drone in
    1)  telnet 192.168.4.1 23 <<EOF
+++AT MODE 2
+++AT MODE 3
+++AT STA ariitk ariitk2022
+++AT SETIP 192.168.0.10
^]
EOF
    ;;
    2)  telnet 192.168.4.1 23 <<EOF
+++AT MODE 2
+++AT MODE 3
+++AT STA ariitk ariitk2022
+++AT SETIP 192.168.0.20
^]
EOF
    ;;
    *) echo "Wrong choice";
esac