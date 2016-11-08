# zapping-ants
Jetson ant zapping: Find ants with a camera and zap them with a laser

Operational code to recognize and track ants and move a laser around

    images:                 MNIST training images extracted from movies

    movies:
    find_ants               Find ants in a movie and record positions in a file
    snap                    Convert shots of ants in a movie to .png images
    make_list.sh            Make input lists for MNIST

    python:
    eibot.py                Opens and controls an EIBOT board over a USB link
    clicker.py              Polls a shared mem location for motor and laser commands

    scripts:                Scripts to start things in various ways

    units:
    units                   Opencv based utility to recognize and track ants.r
                            Sends cmds to clicker.py
    xytest                  Simple utility used to test the setup
