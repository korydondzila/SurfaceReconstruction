@echo off
setlocal enabledelayedexpansion

for /F "tokens=*" %%A in (knot.pcd) do (
    set line=%%A
    echo(!line:~0,-1!>>newknot.pcd
)