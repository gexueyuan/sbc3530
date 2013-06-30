EXTRA_CFLAGS += -Idrivers/net/wireless/wilink/stad/Export_Inc \
	-Idrivers/net/wireless/wilink/utils \
	-Idrivers/net/wireless/wilink/Test \
	-Idrivers/net/wireless/wilink/platforms/hw/host_platform_omap3530/linux \
	-Idrivers/net/wireless/wilink/platforms/os/common/inc \
	-Idrivers/net/wireless/wilink/platforms/os/linux/src \
	-Idrivers/net/wireless/wilink/platforms/os/linux/inc \
	-Idrivers/net/wireless/wilink/platforms/hw/host_platform_omp3530 \
	-Idrivers/net/wireless/wilink/external_drivers/omap3530/Linux/sdio \
	-Idrivers/net/wireless/wilink/external_apps/Linux/bmtrace/Export_Inc \
	-Idrivers/net/wireless/wilink/TWD \
	-Idrivers/net/wireless/wilink/TWD/Ctrl \
	-Idrivers/net/wireless/wilink/TWD/Data_Service/Export_Inc \
	-Idrivers/net/wireless/wilink/TWD/FW_Transfer/Export_Inc \
	-Idrivers/net/wireless/wilink/TWD/FW_Transfer \
	-Idrivers/net/wireless/wilink/TWD/Ctrl/Export_Inc \
	-Idrivers/net/wireless/wilink/TWD/MacServices/Export_Inc \
	-Idrivers/net/wireless/wilink/TWD/FirmwareApi \
	-Idrivers/net/wireless/wilink/TWD/TwIf/ \
	-Idrivers/net/wireless/wilink/TWD/MacServices \
	-Idrivers/net/wireless/wilink/TWD/TWDriver \
	-Idrivers/net/wireless/wilink/Txn \
	-Idrivers/net/wireless/wilink/stad/src/AirLink_Managment \
	-Idrivers/net/wireless/wilink/stad/src/Application \
	-Idrivers/net/wireless/wilink/stad/src/XCC \
	-Idrivers/net/wireless/wilink/stad/src/Connection_Managment \
	-Idrivers/net/wireless/wilink/stad/src/Ctrl_Interface \
	-Idrivers/net/wireless/wilink/stad/src/Data_link \
	-Idrivers/net/wireless/wilink/stad/src/Connection_Managment \
	-Idrivers/net/wireless/wilink/stad/src/Sta_Management \
	-Idrivers/net/wireless/wilink/stad/src/Connection_Managment 
EXTRA_CFLAGS += -D HOST_PLATFORM_OMAP3530 -D __BYTE_ORDER_LITTLE_ENDIAN -D TIWLAN_BMTRACE -D HOST_COMPILE -D FW_RUNNING_AS_STA -D TNETW1273 -D __LINUX__ 
obj-m	= libuadrv.o
libuadrv-objs = \
	../../src/osRgstry.o \
	../../../../../stad/src/Ctrl_Interface/CmdHndlr.o
