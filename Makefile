BUILD_DIR ?= build
CMAKE ?= cmake

$(BUILD_DIR)/join-point-clouds:
	rm -rf build
	$(CMAKE) --version
	$(CMAKE) -G Ninja -B $(BUILD_DIR) .
	$(CMAKE) --build $(BUILD_DIR) --target join-point-clouds

# Docker
BUILD_CMD = docker buildx build --pull $(BUILD_PUSH) --force-rm --no-cache --build-arg MAIN_TAG=$(MAIN_TAG) --build-arg BASE_TAG=$(BUILD_TAG) --platform linux/$(BUILD_TAG) -f $(BUILD_FILE) -t '$(MAIN_TAG):$(BUILD_TAG)' .
BUILD_PUSH = --load
BUILD_FILE = ./etc/Dockerfile

# CI target that automates pushes, avoid for local
docker: MAIN_TAG = ghcr.io/viam-modules/join-point-clouds
docker: BUILD_TAG = arm64
docker: BUILD_PUSH = --push
docker:
	$(BUILD_CMD)

appimage: $(BUILD_DIR)/join-point-clouds
	appimage-builder --recipe packaging/appimage-builder.yml
	chmod +x viam-camera-join-point-clouds-latest-aarch64.AppImage

module.tar.gz: appimage
	mv viam-camera-join-point-clouds-latest-aarch64.AppImage viam-jpc.AppImage
	tar czf module.tar.gz viam-jpc.AppImage
	rm viam-jpc.AppImage

clean:
	rm -rf build *.AppImage

setup:
	sudo apt-get update
	sudo apt-get install -qqy \
		libpcl-dev \
		libmpich-dev \
		libflann-dev \
		cmake
