/* 
*  For nandflash
*/
mmc init 
fatload mmc 0:1 80000000 MLO 
nandecc hw 
nand erase 0 80000 
nand write 80000000 0 80000


mmc init 
fatload mmc 0:1 80000000 u-boot.bin 
nandecc sw 
nand erase 80000 160000 
nand write 80000000 80000 160000



mmc init 
fatload mmc 0:1 80000000 uImage 
nandecc sw 
nand erase 280000 400000 
nand write 80000000 280000 400000



mmc init 
fatload mmc 0:1 84000000 ubi.img 
nandecc sw
nand erase 680000 10000000
nand write 84000000 680000 4600000

/* 
*  For 4.3 inch touchscreen display
*/
setenv dvimode
setenv defaultdisplay lcd043


/* 
*  For 7 inch touchscreen display
*/
setenv dvimode
setenv defaultdisplay lcd070


/* 
*  For EVM3530-B3
*/
setenv boardmodel EVM35X-B3-1880-LUNC0

/* 
*  For SBC3530-B1
*/
setenv boardmodel SBC35X-B1-1880-LUAC0