IMAGE_NAME = ardupilot_waf
VOLUME = -v $(realpath build):/ardupilot/build

build_image:
	mkdir -p build
	docker build -f Dockerfile -t $(IMAGE_NAME) ../..

run: build_image
	docker run --rm $(VOLUME) -it $(IMAGE_NAME) /ardupilot/Tools/docker/build.sh

shell: build_image
	docker run --rm $(VOLUME) -it $(IMAGE_NAME) /bin/bash

# TODO: allow partial builds for more rapid development
