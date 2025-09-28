import sys
import traceback
import os

# Base (known) build directories - keep these as explicit candidates
candidate_dirs = [
    r"C:\_LOCAL\GitHub\PC_WIN\tests\pcl_hybrid\build_py39\Release",
    r"C:\_LOCAL\GitHub\PC_WIN\tests\pcl_hybrid\build_py39\Debug",
    r"C:\_LOCAL\GitHub\PC_WIN\tests\pcl_hybrid\build\Debug",
]

# Ensure vcpkg runtime DLLs are on PATH so dependent DLLs can be resolved when loading the .pyd
vcpkg_bin = r"C:\vcpkg\installed\x64-windows\bin"
if os.path.isdir(vcpkg_bin):
    os.environ['PATH'] = vcpkg_bin + os.pathsep + os.environ.get('PATH', '')


def discover_interpreter_paths():
    """Discover additional candidate paths from the running interpreter (helps when running inside Rhino)."""
    added = []
    try:
        import site
    except Exception:
        site = None

    try:
        import sysconfig
    except Exception:
        sysconfig = None

    # Basic interpreter info
    print('Interpreter executable:', sys.executable)
    print('Interpreter version:', sys.version.replace('\n', ' '))
    print('sys.prefix:', sys.prefix)
    print('sys.exec_prefix:', sys.exec_prefix)

    # Candidate: site.getsitepackages()
    if site is not None:
        try:
            for p in getattr(site, 'getsitepackages', lambda: [])():
                if p and os.path.isdir(p) and p not in candidate_dirs:
                    candidate_dirs.append(p)
                    added.append(p)
        except Exception:
            pass

    # Candidate: user site-packages
    if site is not None:
        try:
            up = site.getusersitepackages()
            if up and os.path.isdir(up) and up not in candidate_dirs:
                candidate_dirs.append(up)
                added.append(up)
        except Exception:
            pass

    # Candidate: sysconfig purelib / platlib
    if sysconfig is not None:
        try:
            paths = sysconfig.get_paths()
            for key in ('purelib', 'platlib'):
                p = paths.get(key)
                if p and os.path.isdir(p) and p not in candidate_dirs:
                    candidate_dirs.append(p)
                    added.append(p)
        except Exception:
            pass

    # Candidate: typical location under the python executable folder: Lib\site-packages
    try:
        exe_dir = os.path.dirname(sys.executable) or None
        if exe_dir:
            guess = os.path.join(exe_dir, 'Lib', 'site-packages')
            if os.path.isdir(guess) and guess not in candidate_dirs:
                candidate_dirs.append(guess)
                added.append(guess)
    except Exception:
        pass

    # Print discovered additions
    if added:
        print('Added candidate dirs from interpreter discovery:')
        for a in added:
            print('  -', a)
    else:
        print('No additional interpreter candidate dirs discovered')


def find_pyd_files(candidates):
    import glob
    pyds = []
    for d in candidates:
        try:
            pyds.extend(glob.glob(os.path.join(d, 'pcl_hybrid_py*.pyd')))
        except Exception:
            pass
    # fallback: search repo tree
    if not pyds:
        base = os.path.dirname(__file__)
        pyds = glob.glob(os.path.join(base, '**', 'pcl_hybrid_py*.pyd'), recursive=True)
    return pyds


# Discover paths from the current interpreter (this will print Rhino paths when run inside Rhino)
discover_interpreter_paths()

# Ensure each candidate dir is on sys.path (front) so imports prefer local builds
for d in list(candidate_dirs):
    try:
        if os.path.isdir(d) and d not in sys.path:
            sys.path.insert(0, d)
    except Exception:
        pass


# Try a normal import first
print('\nAttempting a normal import of pcl_hybrid_py...')
try:
    import pcl_hybrid_py
    ver = getattr(pcl_hybrid_py, '__version__', getattr(pcl_hybrid_py, 'version', 'no version'))
    print('Imported pcl_hybrid_py OK:', ver)
except Exception:
    print('Normal import failed; attempting to locate .pyd files and load them individually...')
    traceback.print_exc()

    # Prepare guarded importlib/imp utilities
    try:
        import importlib.util as _importlib_util
    except Exception:
        _importlib_util = None
    try:
        import importlib.machinery as _importlib_machinery
    except Exception:
        _importlib_machinery = None

    pyd_paths = find_pyd_files(candidate_dirs)

    if not pyd_paths:
        print('No pcl_hybrid_py*.pyd files found in candidate directories or repo tree')
    else:
        print('Found', len(pyd_paths), 'candidate .pyd files:')
        for p in pyd_paths:
            print('  -', p)

        for pyd in pyd_paths:
            print('\nTrying to load:', pyd)
            try:
                if _importlib_machinery is not None and _importlib_util is not None:
                    loader = _importlib_machinery.ExtensionFileLoader('pcl_hybrid_py', pyd)
                    spec = _importlib_util.spec_from_loader(loader.name, loader)
                    mod = _importlib_util.module_from_spec(spec)
                    loader.exec_module(mod)
                    sys.modules['pcl_hybrid_py'] = mod
                    print('Loaded via importlib from file:', pyd)
                    break
                else:
                    try:
                        import imp
                        mod = imp.load_dynamic('pcl_hybrid_py', pyd)
                        sys.modules['pcl_hybrid_py'] = mod
                        print('Loaded via imp.load_dynamic from file:', pyd)
                        break
                    except Exception:
                        # last resort: add dir to sys.path and import normally
                        ddir = os.path.dirname(pyd)
                        if ddir and ddir not in sys.path:
                            sys.path.insert(0, ddir)
                        try:
                            mod = __import__('pcl_hybrid_py')
                            print('Imported after adding dir to sys.path:', pyd)
                            break
                        except Exception:
                            raise
            except Exception:
                print('Failed to load', pyd)
                traceback.print_exc()
