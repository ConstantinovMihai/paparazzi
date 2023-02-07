# Copyright (C) 2008-2022 The Paparazzi Team
# released under GNU GPLv2 or later. See COPYING file.
import os
import subprocess
from PyQt5.QtWidgets import *
from typing import NamedTuple
from PyQt5.QtCore import QSettings


class GConfEntry(NamedTuple):
    name: str
    value: str
    application: str


PAPARAZZI_SRC = os.getenv("PAPARAZZI_HOME")
PAPARAZZI_HOME = os.getenv("PAPARAZZI_HOME", PAPARAZZI_SRC)
CONF_DIR = os.path.join(PAPARAZZI_HOME, "conf/")


def remove_prefix(string: str, prefix: str, /) -> str:
    if string.startswith(prefix):
        return string[len(prefix):]
    else:
        return string[:]


def remove_suffix(s: str, suffix: str, /) -> str:
    if s.endswith(suffix):
        return s[:-len(suffix)]
    else:
        return s


# TODO: make it work with shell program such as vim.
def edit_file(file_path, prefix=CONF_DIR):
    path = prefix + file_path
    editor = get_settings().value("text_editor", "", str)
    if editor == "":
        editor = "gedit"
    try:
        subprocess.Popen([editor, path])
    except Exception as e:
        print(e)


def make_line(parent: QWidget = None, vertical=False) -> QWidget:
    line = QFrame(parent)
    if vertical:
        line.setFrameShape(QFrame.VLine)
    else:
        line.setFrameShape(QFrame.HLine)
    line.setFrameShadow(QFrame.Sunken)
    return line


def get_version() -> str:
    run_version_exe = os.path.join(PAPARAZZI_HOME, "paparazzi_version")
    proc = subprocess.run(run_version_exe, cwd=PAPARAZZI_HOME, capture_output=True)
    return proc.stdout.decode().strip()


def get_build_version() -> str:
    bv_path = os.path.join(PAPARAZZI_HOME, "var", "build_version.txt")
    with open(bv_path, 'r') as f:
        version = f.readline().strip()
    return version


def open_terminal(wd, command=None):
    cmd = ""
    if command is not None:
        cmd = " -- {}".format(command)
    terminal_emulator = get_settings().value("terminal_emulator", "", str)
    if terminal_emulator == "":
        terminal_emulator = "gnome-terminal"
    os.system("{} --working-directory {}{}".format(terminal_emulator, wd, cmd))


def get_settings() -> QSettings:
    return QSettings(os.path.join(CONF_DIR, "pprz_center_settings.ini"), QSettings.IniFormat)


ABOUT_TEXT = \
    """
    <h1>Paparazzi Center</h1>
    <p>The Paparazzi Center is the home application for Paparazzi UAV.</br>
    Learn more about Paparazzi:
    <ul>
      <li>Documentation:<br/>
        <a href="https://paparazzi-uav.readthedocs.io">https://paparazzi-uav.readthedocs.io</a>
      </li>
      <li>Wiki:<br/>
        <a href="https://wiki.paparazziuav.org">https://wiki.paparazziuav.org</a>
      </li>
      <li>Github:<br/>
        <a href="https://github.com/paparazzi/paparazzi/">https://github.com/paparazzi/paparazzi/</a>
      </li>
      <li>Gitter community chat:<br/>
        <a href="https://gitter.im/paparazzi/discuss">https://gitter.im/paparazzi/discuss</a>
      </li>
      <li>Autogenerated developer doc:<br/>
        <a href="https://docs.paparazziuav.org/">https://docs.paparazziuav.org/</a>
      </li>
    </ul> 
    </p>
    
    <h2>Licence information:</h2>
    <p>
    <strong>Copyright (C) 2008-2022 The Paparazzi Team</strong><br/>
    Paparazzi Center is part of the Paparazzi, released under GPLv2 or any later version.
    See the file <a href="{}/COPYING">COPYING</a>,
    or the licence information at <a href="http://www.gnu.org/licenses/">http://www.gnu.org/licenses/</a>.
    </p>
    <p>
    This software uses:
    <ul>
        <li><a href="https://www.qt.io/">Qt</a> released under GNU LGPL</li>
        <li><a href="https://www.riverbankcomputing.com/software/pyqt/">PyQt5</a> released under GNU GPLv3</li>
    </ul> 
    </p>

    """.format(PAPARAZZI_HOME)
