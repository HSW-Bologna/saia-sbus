import os

LIBRARY = './sbus'
sources = {}

Import('sbus_env')
objects = sbus_env.Object(Glob(os.path.join(LIBRARY, '*.c')))
Return('objects')