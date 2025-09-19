# from repo root or change path as needed
$src = 'C:\vcpkg\installed\x64-windows\bin'
$dst = 'C:\_LOCAL\GitHub\PC_WIN\tests\pcl_hybrid\build_py39\Release'
Get-ChildItem -Path $src -Filter '*.dll' | ForEach-Object {
    Copy-Item -Path $_.FullName -Destination $dst -Force
}
# then run the test harness
Set-Location 'C:\_LOCAL\GitHub\PC_WIN\tests\pcl_hybrid'
python runtests.py
