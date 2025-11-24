#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Sofía
"""

import sys
import os

# Añadir la carpeta 'lib' al path
project_root = os.path.dirname(os.path.abspath(__file__))
lib_path = os.path.join(project_root, 'lib')
sys.path.insert(0, lib_path)

# Asegurar que 'lib' esté en el path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(project_root, 'lib'))

from lib.TejoApplication import TejoApplication

def main():
    print("🚀 Iniciando Tejo v0.1 (PyBullet + Ogre + VTK)...")
    app = TejoApplication()
    app.go()

if __name__ == "__main__":
    main()