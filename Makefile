BUILD_DIR ?= build
CMAKE ?= cmake

$(BUILD_DIR)/join-point-clouds:
	rm -rf build
	$(CMAKE) --version
	$(CMAKE) -G Ninja -B $(BUILD_DIR) .
	$(CMAKE) --build $(BUILD_DIR) --target join-point-clouds

appimage: $(BUILD_DIR)/join-point-clouds
	appimage-builder --recipe packaging/appimage-builder.yml
	chmod +x viam-camera-join-point-clouds-latest-aarch64.AppImage

clean:
	rm -rf build *.AppImage

setup:
	sudo apt-get update
	sudo apt-get install -qqy \
		libpcl-dev \
		libmpich-dev \
		libflann-dev \
		cmake
