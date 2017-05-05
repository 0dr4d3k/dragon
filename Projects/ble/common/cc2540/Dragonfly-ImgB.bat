
::@echo off
::chdir %1\..\..\common\cc2540
::start cc254x_ubl_pp.js %2 %3 %4 %5

::cc254x_ubl_pp.js "ProdUBL" "%1\CC2541DF-OAD-ImgA\Exe\dragonfly.sim"
cd c:\ruben\dragonfly_work\Projects\ble\common\cc2540\
cmd /C cc254x_ubl_pp.js "ProdUBL" "c:\ruben\dragonfly_work\Projects\ble\dragonfly\CC2541DF\CC2541DF-DragonFly-OAD-ImgB\Exe\DragonFly.sim" 
