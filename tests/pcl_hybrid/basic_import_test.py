import sys, os
sys.path.insert(0, r'C:\_LOCAL\GitHub\PC_WIN\tests\pcl_hybrid\build_py39\Debug')
sys.path.insert(0, r'C:\_LOCAL\GitHub\PC_WIN\tests\pcl_hybrid')
os.environ['PATH'] = r'C:\_LOCAL\GitHub\PC_WIN\tests\pcl_hybrid\build_py39\Debug' + os.pathsep + os.environ['PATH']
print('sys.path:', sys.path)
print('PATH:', os.environ['PATH'])
import pcl_hybrid_py
import main
