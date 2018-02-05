# Build container
FROM viros/is-cpp:1
ARG SERVICE=local
WORKDIR /opt
COPY . .
RUN make release                                                \
 && mkdir deploy                                                \
 && mv ${SERVICE} deploy/service                                \
 && libs=`ldd deploy/service                                    \
    | awk 'BEGIN{ORS=" "}$1~/^\//{print $1}$3~/^\//{print $3}'  \
    | sed 's/,$/\n/'`                                           \
 && for lib in $libs;                                           \
    do                                                          \  
      dir="deploy`dirname $lib`";                               \
      mkdir -v -p  $dir;                                        \
      cp --verbose $lib $dir;                                   \
    done

# Deployment container
FROM scratch
ENV LD_LIBRARY_PATH=/usr/local/lib
COPY --from=0 /opt/deploy/ /
CMD ["/service"]