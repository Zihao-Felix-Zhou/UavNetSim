@echo off
cd /d "%~dp0.."
start "" /b ".venv\Scripts\python.exe" -m uvicorn api.app:app --host 127.0.0.1 --port 8000 > server.out.log 2> server.err.log
