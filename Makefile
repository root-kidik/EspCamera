.PHONY: set-target
set-target:
	idf.py set-target esp32

.PHONY: clear
clear:
	rm -rf build dependencies.lock sdkconfig sdkconfig.old managed_components