#!/usr/bin/env python3
"""
Recursively check DLL dependencies for a Windows PE file (.pyd/.dll).

Usage:
  python check_deps.py path\to\module.pyd

If `pefile` is not installed, install with: python -m pip install pefile

This script searches each directory in the current PATH for required DLLs and
reports which direct or transitive imports are missing.
"""
import os
import sys
from collections import deque

try:
    import pefile
except Exception as e:
    print('Missing dependency: pefile. Install with: python -m pip install pefile')
    raise


def find_dll_on_path(dll_name):
    # Case-insensitive match on PATH entries
    paths = os.environ.get('PATH', '').split(os.pathsep)
    dll_lower = dll_name.lower()
    for p in paths:
        try:
            entries = os.listdir(p)
        except Exception:
            continue
        for f in entries:
            if f.lower() == dll_lower:
                return os.path.join(p, f)
    return None


def get_imports(pe_path):
    try:
        pe = pefile.PE(pe_path)
    except Exception as e:
        print(f'Error reading PE {pe_path}: {e}')
        return []
    imports = []
    if hasattr(pe, 'DIRECTORY_ENTRY_IMPORT'):
        for entry in pe.DIRECTORY_ENTRY_IMPORT:
            try:
                imports.append(entry.dll.decode('utf-8'))
            except Exception:
                pass
    return imports


def check_recursive(root):
    root = os.path.abspath(root)
    q = deque([root])
    seen = set()
    missing = set()
    resolved = {}

    while q:
        p = q.popleft()
        if p in seen:
            continue
        seen.add(p)
        imports = get_imports(p)
        for dll in imports:
            if dll.lower() in ('kernel32.dll','advapi32.dll'):
                # common system libs - skip checking locations
                continue
            found = find_dll_on_path(dll)
            if found:
                resolved[dll] = found
                if found not in seen:
                    q.append(found)
            else:
                missing.add(dll)

    return resolved, missing


def main():
    if len(sys.argv) < 2:
        print('Usage: python check_deps.py path\\to\\module.pyd')
        return 1
    target = sys.argv[1]
    if not os.path.exists(target):
        print('File not found:', target)
        return 2
    print('Checking:', target)
    resolved, missing = check_recursive(target)
    print('\nResolved (sample):')
    for k, v in list(resolved.items())[:20]:
        print('  ', k, '->', v)
    if missing:
        print('\nMissing DLLs:')
        for m in sorted(missing):
            print('  ', m)
        print('\nRecommendation: add directories that contain the missing DLLs to your PATH, or copy the missing DLLs into the Release folder for testing.')
        return 3
    else:
        print('\nAll imports resolved (direct and transitive) on current PATH.')
        return 0


if __name__ == '__main__':
    sys.exit(main())
