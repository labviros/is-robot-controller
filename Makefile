CXX = clang++
CXXFLAGS += -std=c++14 -Wall -Werror
DEBUGFLAGS = -g -fsanitize=address -fno-omit-frame-pointer
RELEASEFLAGS = -O2
BUILDFLAGS = $(RELEASEFLAGS)
LDFLAGS += -L/usr/local/lib -I/usr/local/include \
			`pkg-config --libs protobuf librabbitmq libSimpleAmqpClient`\
			-lpthread -lboost_program_options -lboost_system -lboost_filesystem\
			-lismsgs -larmadillo -lyaml-cpp\
			-Wl,--no-as-needed -Wl,--as-needed -ldl
PROTOC = protoc

LOCAL_PROTOS_PATH = ./msgs/
vpath %.proto $(LOCAL_PROTOS_PATH)

MAINTAINER = mendonca
SERVICE = service
VERSION = 1
LOCAL_REGISTRY = git.is:5000

all: $(SERVICE) test

clean:
	rm -f *.o *.pb.cc *.pb.h service test

service: service.o
	$(CXX) $^ $(LDFLAGS) $(BUILDFLAGS) -o $@

test: test.o
	$(CXX) $^ $(LDFLAGS) $(BUILDFLAGS) -o $@

.PRECIOUS: %.pb.cc
%.pb.cc: %.proto
	$(PROTOC) -I $(LOCAL_PROTOS_PATH) --cpp_out=. $<

docker:
	rm -rf libs/
	mkdir libs/
	lddcp service libs/
	docker build -t $(MAINTAINER)/$(SERVICE):$(VERSION) .
	rm -rf libs/

push_local: docker
	docker tag $(MAINTAINER)/$(SERVICE):$(VERSION) $(LOCAL_REGISTRY)/$(SERVICE):$(VERSION)
	docker push $(LOCAL_REGISTRY)/$(SERVICE):$(VERSION)

push_cloud: docker
	docker push $(MAINTAINER)/$(SERVICE):$(VERSION)