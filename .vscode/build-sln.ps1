param(
	[switch]
	$release
)

& "C:/Program Files/Microsoft Visual Studio/2022/Community/Common7/Tools/Launch-VsDevShell.ps1" -SkipAutomaticLocation

if($release){
	msbuild -warnaserror -p:Configuration=Release
}else{
	msbuild -warnaserror -p:Configuration=Debug
}

exit $LASTEXITCODE
