import sys, importlib, os
bd = r"c:\_LOCAL\GitHub\PC_WIN\tests\pcl_hybrid\build\Debug"
os.environ["PATH"] = bd + os.pathsep + os.environ.get("PATH", "")
sys.path.insert(0, bd)
m = importlib.import_module("pcl_hybrid_py")
print("import OK; symbols =", [a for a in dir(m) if not a.startswith('__')])
