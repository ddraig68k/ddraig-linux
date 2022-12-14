@echo // Copyright (C) 1996-2005 Markus F.X.J. Oberhumer
@echo //
@echo //   DOS 16-bit
@echo //   Microsoft QuickC
@echo //
@call b\prepare.bat
@if "%BECHO%"=="n" echo off


set CC=qcl -nologo -AL
set CF=-O -Gf -W3 %CFI%
set LF=%BLIB% -Fm

@REM %CC% %CF% -c src\*.c
for %%f in (src\*.c) do %CC% %CF% -c %%f
@if errorlevel 1 goto error
lib /nologo %BLIB% @b\dos16\bc.rsp;
@if errorlevel 1 goto error

%CC% %CF% examples\dict.c %LF%
@if errorlevel 1 goto error
%CC% %CF% examples\lzopack.c %LF%
@if errorlevel 1 goto error
%CC% %CF% examples\precomp.c %LF%
@if errorlevel 1 goto error
%CC% %CF% examples\precomp2.c %LF%
@if errorlevel 1 goto error
%CC% %CF% examples\simple.c %LF%
@if errorlevel 1 goto error

%CC% %CF% lzotest\lzotest.c %LF%
@if errorlevel 1 goto error


@call b\done.bat
@goto end
:error
@echo ERROR during build!
:end
@call b\unset.bat
