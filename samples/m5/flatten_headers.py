#!/usr/bin/env python3

import re
import os

INCLUDE_DIR = "../../core/include"

OUTPUT_HPP = "ShapoNES/shapones_core.h"

RE_INCLUDE = re.compile(r'#include\s+"(.+)"')
RE_PRAGMA_GCC_OPTIMIZE = re.compile(r'#pragma\s+GCC\s+optimize\s*\(\s*".*"\s*\)')

included_files = set()

def include_file(inc_path):
    path = os.path.join(INCLUDE_DIR, inc_path)
    
    if path in included_files:
        return
    included_files.add(path)

    with open(path, "r") as f:
        lines = f.readlines()
        for line in lines:
            m_include = RE_INCLUDE.search(line)
            m_pragma_opt = RE_PRAGMA_GCC_OPTIMIZE.search(line)
            if m_include:
                out_f.write(f"// {line}\n")
                inc_path = m_include.group(1)
                line = include_file(inc_path)
            elif m_pragma_opt:
                out_f.write(f"// {line}\n")
            else:
                out_f.write(line)

with open(OUTPUT_HPP, "w") as out_f:
    out_f.write("#ifndef SHAPONES_CORE_H\n")
    out_f.write("#define SHAPONES_CORE_H\n")
    out_f.write("\n")
    
    out_f.write("#define SHAPONES_MENU_LARGE_FONT (1)\n\n")
    out_f.write("#define SHAPONES_ENABLE_LOG (1)\n\n")
    
    out_f.write("#pragma GCC optimize (\"O2\")\n\n")
    
    include_file("shapones/shapones.hpp")

    out_f.write("#ifdef SHAPONES_IMPLEMENTATION\n")

    include_file("../src/common.cpp")
    include_file("../src/apu.cpp")
    include_file("../src/cpu.cpp")
    include_file("../src/fs.cpp")
    include_file("../src/input.cpp")
    include_file("../src/interrupt.cpp")
    include_file("../src/mapper.cpp")
    include_file("../src/memory.cpp")
    include_file("../src/menu.cpp")
    include_file("../src/ppu.cpp")
    include_file("../src/state.cpp")
    include_file("../src/shapones.cpp")

    out_f.write("#endif\n")

    out_f.write("\n")
    out_f.write("#endif\n")
