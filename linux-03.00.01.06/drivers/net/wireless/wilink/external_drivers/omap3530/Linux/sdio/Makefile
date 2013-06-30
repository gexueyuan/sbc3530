EXTRA_CFLAGS += -Idrivers/net/wireless/wilink/Txn \
	-Idrivers/net/wireless/wilink/external_drivers/omap3530/Linux/sdio \
	-Idrivers/net/wireless/wilink/platforms/hw/host_platform_omap3530/linux \
	-Idrivers/net/wireless/wilink/utils
EXTRA_CFLAGS += -DOMAP3_EVM -DCONFIG_SDIO_ADDRESS_MAPPING_BY_APPLICATION 
obj-m	:= sdio.o
obj-m	+= testsdio.o
testsdio-objs := testdrv.o ../../../../platforms/hw/host_platform_omap3530/linux/SdioAdapter.o
sdio-objs := SdioDrv.o
