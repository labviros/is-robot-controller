FROM git.is:5000/is-cpp:1

RUN git clone https://github.com/jbeder/yaml-cpp && \
    cd yaml-cpp && mkdir build && cd build && \
    cmake .. && make install && \
    cd ../../ && rm -rf yaml-cpp

ARG SERVICE=local
COPY . ${SERVICE}
RUN cd ${SERVICE} && \
    make clean && \
    make release && \
    mkdir lib && \
    for lib in `ldd service | awk 'BEGIN{ORS=" "}$1~/^\//{print $1}$3~/^\//{print $3}' | sed 's/,$/\n/'`; do cp $lib lib/; done 

FROM ubuntu:16.04
ARG SERVICE=local
COPY --from=0 ${SERVICE}/service .
COPY --from=0 ${SERVICE}/parameters.yaml .
COPY --from=0 ${SERVICE}/lib /usr/local/lib/
RUN ldconfig
CMD ["./service"]