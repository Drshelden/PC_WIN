import os
import sys

print('PYTHON:', sys.executable)
print('Python version:', sys.version)
print('sys.path (head):', sys.path[:5])
print('PATH has build folder?', os.pathsep.join(sys.path).find('build_py39\Release')!=-1)

# Ensure build folder is first on sys.path
build_release = os.path.abspath(os.path.join(os.path.dirname(__file__), 'build_py39', 'Release'))
if build_release not in sys.path:
    sys.path.insert(0, build_release)

print('sys.path after insert:', sys.path[:5])
print('os.environ["PATH"] head:', os.environ['PATH'].split(os.pathsep)[:5])

try:
    import pcl_hybrid_py
    print('Imported pcl_hybrid_py OK')
    # quick smoke test if available
    if hasattr(pcl_hybrid_py, 'importPoints'):
        print('pcl_hybrid_py.importPoints available')
except Exception as e:
    print('Import failed:', e)
    raise
