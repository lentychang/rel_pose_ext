ARG DOCKER_BASE_IMAGE=ros-docker-base
FROM $DOCKER_BASE_IMAGE

ARG ROS_DISTRO=kinetic

RUN cd /root && \
    wget https://download.savannah.gnu.org/releases/freetype/freetype-2.9.tar.bz2 && \
    tar -xvf freetype-2.9.tar.bz2 && \
    cd freetype-2.9 && mkdir build && cd build && \
    cmake .. && \
    make && \
    make install

RUN cd /root && \
    wget http://prdownloads.sourceforge.net/swig/swig-3.0.12.tar.gz && \
    tar -xvf swig-3.0.12.tar.gz && \
    cd swig-3.0.12 && \
    apt install -y build-essential libpcre3-dev libpcre3 && \
    cd /root/swig-3.0.12 && \
    sh ./configure && \
    make && \
    make install 

RUN cd /root && \
    git clone git://github.com/tpaviot/oce.git && \
    cd oce && mkdir build && cd build && \ 
#    cmake .. && \
#    make -j2 && \
#    make install/strip 

    cd /root && \
    git clone git://github.com/tpaviot/pythonocc-core.git && \
    cd pythonocc-core && \
    mkdir build && \
    cd build 
 #   apt-get update && \

