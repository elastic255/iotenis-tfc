deps_config := \
	/home/fabio/esp/esp-idf/components/app_trace/Kconfig \
	/home/fabio/esp/esp-idf/components/aws_iot/Kconfig \
	/home/fabio/esp/esp-idf/components/bt/Kconfig \
	/home/fabio/esp/esp-idf/components/esp32/Kconfig \
	/home/fabio/esp/esp-idf/components/ethernet/Kconfig \
	/home/fabio/esp/esp-idf/components/fatfs/Kconfig \
	/home/fabio/esp/esp-idf/components/freertos/Kconfig \
	/home/fabio/esp/esp-idf/components/heap/Kconfig \
	/home/fabio/esp/esp-idf/components/libsodium/Kconfig \
	/home/fabio/esp/esp-idf/components/log/Kconfig \
	/home/fabio/esp/esp-idf/components/lwip/Kconfig \
	/home/fabio/Documentos/TFC/IoTenis/firmware/espFirmware/main/Kconfig \
	/home/fabio/esp/esp-idf/components/mbedtls/Kconfig \
	/home/fabio/esp/esp-idf/components/openssl/Kconfig \
	/home/fabio/esp/esp-idf/components/pthread/Kconfig \
	/home/fabio/esp/esp-idf/components/spi_flash/Kconfig \
	/home/fabio/esp/esp-idf/components/spiffs/Kconfig \
	/home/fabio/esp/esp-idf/components/tcpip_adapter/Kconfig \
	/home/fabio/esp/esp-idf/components/wear_levelling/Kconfig \
	/home/fabio/esp/esp-idf/components/bootloader/Kconfig.projbuild \
	/home/fabio/esp/esp-idf/components/esptool_py/Kconfig.projbuild \
	/home/fabio/esp/esp-idf/components/partition_table/Kconfig.projbuild \
	/home/fabio/esp/esp-idf/Kconfig

include/config/auto.conf: \
	$(deps_config)


$(deps_config): ;
