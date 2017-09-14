import os
import ycm_core

flags = [
    '-Wall',
    '-Wextra',
    '-Weffc++',
    '-std=c++11',
    '-xc++',
    '-isystem/usr/include/',
    '-I/opt/data/git/fordyca/include'
    '-I/opt/data/git/fordyca/src/tests/include'
    '-I/home/jharwell/git/rcppsw/include'
    '-I/home/jharwell/git/rcsw/include'
    '-isystem/usr/lib/x86_64-linux-gnu/qt5/mkspecs/linux-g++-64'
    '-isystem/usr/include/x86_64-linux-gnu/qt5/QtCore'
    '-isystem/usr/include/x86_64-linux-gnu/qt5/QtGui'
    '-isystem/usr/include/x86_64-linux-gnu/qt5/QtWidgets'
    '-isystem/usr/include/x86_64-linux-gnu/qt5'
    '-isystem/usr/include/lua5.2'
]

SOURCE_EXTENSIONS = ['.cpp', '.hpp']


def FlagsForFile(filename, **kwargs):
    return {
        'flags': flags,
        'do_cache': True
    }
