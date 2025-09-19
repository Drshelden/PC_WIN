@echo off
setlocal
set DLLS=pcl_segmentationd.dll python39.dll pcl_featuresd.dll pcl_iod.dll pcl_searchd.dll pcl_kdtreed.dll pcl_commond.dll KERNEL32.dll ADVAPI32.dll MSVCP140D.dll VCOMP140D.DLL VCRUNTIME140D.dll VCRUNTIME140_1D.dll ucrtbased.dll
for %%D in (%DLLS%) do (
    where %%D >nul 2>&1
    if errorlevel 1 (
        echo NOT FOUND: %%D
    ) else (
        echo FOUND: %%D
    )
)
endlocal
