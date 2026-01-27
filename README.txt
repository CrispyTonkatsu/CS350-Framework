/* Start Header ------------------------------------------------------
Copyright (C) 2026 DigiPen Institute of Technology.
File Name: README.txt
Purpose: Information of the project
Language: ASCII English
Platform: Windows MSVC version: 18.0.5.56406
Project: e.donosomansilla_CS350_1
Author: Edgar Jose Donoso Mansilla, e.donosomansilla, id: 0066578
Creation date: 26-Jan-2026
End Header -------------------------------------------------------*/

The program is compiled using MSVC via the .sln file at the root of the project. 

NOTE:
When compiling on Release mode, the project had been giving me the following error related to the version that I have installed:

"C:\Users\edgui\OneDrive\Documents\DigiPen\CS350\Framework\CS350Framework.sln"  
(default target) (1) ->
"C:\Users\edgui\OneDrive\Documents\DigiPen\CS350\Framework\CS350Framework\CS350 
Framework.vcxproj" (default target) (2) ->
(Link target) ->
  LINK : fatal error C1900: Il mismatch between 'P1' version '20240319' and 'P2 
' version '20230904' [C:\Users\edgui\OneDrive\Documents\DigiPen\CS350\Framework 
\CS350Framework\CS350Framework.vcxproj]
  LINK : fatal error LNK1257: code generation failed [C:\Users\edgui\OneDrive\D 
ocuments\DigiPen\CS350\Framework\CS350Framework\CS350Framework.vcxproj]

To deal with it I re-targeted the toolset versions it should use. In 
