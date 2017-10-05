COMPILER = g++
FLAGS = -std=c++14 -O3 -Wall -Werror -Wextra -Wpedantic

SO_DEPS = $(shell pkg-config --libs --cflags libSimpleAmqpClient msgpack librabbitmq opencv theoradec theoraenc)
SO_DEPS += -lboost_program_options -lboost_system -lboost_filesystem -lpthread -larmadillo -lyaml-cpp -Iinclude/

MAINTAINER = mendonca
SERVICE = robot-controller
VERSION = 1.2
LOCAL_REGISTRY = git.is:5000

all: $(SERVICE) robot-status test

clean:
	rm -f $(SERVICE) robot-status test

robot-status: robot-status.cpp
	$(COMPILER) $^ -o $@ $(FLAGS) $(SO_DEPS) 

test: test.cpp
	$(COMPILER) $^ -o $@ $(FLAGS) $(SO_DEPS) 

$(SERVICE): $(SERVICE).cpp
	$(COMPILER) $^ -o $@ $(FLAGS) $(SO_DEPS) 

docker: $(SERVICE)
	rm -rf libs/
	mkdir libs/
	lddcp $(SERVICE) libs/
	docker build -t $(MAINTAINER)/$(SERVICE):$(VERSION) .
	rm -rf libs/

push_local: docker
	docker tag $(MAINTAINER)/$(SERVICE):$(VERSION) $(LOCAL_REGISTRY)/$(SERVICE):$(VERSION) 
	docker push $(LOCAL_REGISTRY)/$(SERVICE):$(VERSION)

push_cloud: docker
	docker push $(MAINTAINER)/$(SERVICE):$(VERSION)