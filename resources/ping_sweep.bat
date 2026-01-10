@echo off
setlocal

for /L %%i in (202,1,254) do (
    start /b ping -n 1 -w 50 10.24.29.%%i | find "Reply"
)

endlocal
