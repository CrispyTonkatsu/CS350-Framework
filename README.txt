/* Start Header ------------------------------------------------------
Copyright (C) 2026 DigiPen Institute of Technology.
File Name: README.txt
Purpose: Information of the project
Language: ASCII English
Platform: Windows MSVC version: 18.0.5.56406
Project: e.donosomansilla_CS350_5
Author: Edgar Jose Donoso Mansilla, e.donosomansilla, id: 0066578
Creation date: 16-March-2026
End Header -------------------------------------------------------*/

The program is compiled using MSVC via the .sln file at the root of the project.

Given the math in the slides is not specific enough, I find myself unable to do a 1:1 match of all decimal values as:
- floating point number operations are not commutative
- they are not the same across platforms

I believe for future instances of this class, it should use fixed point numbers to ensure that the 
same operations produce the same results across machines.