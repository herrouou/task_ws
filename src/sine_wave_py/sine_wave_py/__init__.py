# -*- coding: utf-8 -*-
# Package initialization
# This file makes the tests directory a Python package
import os
import sys

# Add the parent directory to sys.path
sys.path.insert(0, os.path.abspath(os.path.dirname(os.path.dirname(__file__))))
