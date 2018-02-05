CXX = clang++
CXXFLAGS += -std=c++14
LDFLAGS += -L/usr/local/lib -I/usr/local/include \
			-lpthread -lprotobuf -lrabbitmq -lSimpleAmqpClient \
			-lboost_program_options -lboost_system -lboost_filesystem -larmadillo \
			-lismsgs -lopentracing -lzipkin -lzipkin_opentracing
PROTOC = protoc
LOCAL_PROTOS_PATH = ./msgs/

vpath %.proto $(LOCAL_PROTOS_PATH)

MAINTAINER = viros
SERVICE = robot-controller
VERSION = 1
LOCAL_REGISTRY = ninja.local:5000

all: debug

debug: CXXFLAGS += -g 
debug: LDFLAGS += -fsanitize=address -fno-omit-frame-pointer
debug: $(SERVICE)

release: CXXFLAGS += -Wall -Werror -O2
release: $(SERVICE)

$(SERVICE): robot-parameters.pb.o $(SERVICE).o
	$(CXX) $^ $(LDFLAGS) -o $@

.PRECIOUS: %.pb.cc
%.pb.cc: %.proto
	$(PROTOC) -I $(LOCAL_PROTOS_PATH) --cpp_out=. $<

clean:
	rm -f *.o *.pb.cc *.pb.h $(SERVICE)

docker: 
	docker build -t $(MAINTAINER)/$(SERVICE):$(VERSION) --build-arg=SERVICE=$(SERVICE) .

push_local: docker
	docker tag $(MAINTAINER)/$(SERVICE):$(VERSION) $(LOCAL_REGISTRY)/$(SERVICE):$(VERSION)
	docker push $(LOCAL_REGISTRY)/$(SERVICE):$(VERSION)

push_cloud: docker
	docker push $(MAINTAINER)/$(SERVICE):$(VERSION)