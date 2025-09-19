import pefile, os
p = r"C:\_LOCAL\GitHub\PC_WIN\tests\pcl_hybrid\build_py39\Release\pcl_hybrid_py.cp39-win_amd64.pyd"
pe = pefile.PE(p)
imports = set()
if hasattr(pe, 'DIRECTORY_ENTRY_IMPORT'):
    for entry in pe.DIRECTORY_ENTRY_IMPORT:
        imports.add(entry.dll.decode('utf-8'))
print('Direct imports:', sorted(imports))
