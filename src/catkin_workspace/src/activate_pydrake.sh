#!/bin/sh
export PYTHONPATH=~/git/manipulation:${PYTHONPATH}
export PYTHONPATH=/opt/drake/lib/python3.8/site-packages:${PYTHONPATH}
jupyter notebook
