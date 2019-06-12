# BISMO Dockerfile
FROM hseeberger/scala-sbt
WORKDIR /app
RUN sed -Ei 's/^# deb-src /deb-src /' /etc/apt/sources.list
RUN apt-get -y update && apt-get -y upgrade
RUN apt-get install -y  tzdata
RUN echo "Ireland/Dublin" >> /etc/timezone
RUN dpkg-reconfigure --frontend noninteractive tzdata
RUN apt-get install -y zsh git verilator make build-essential
RUN git clone --recurse-submodules https://github.com/EECS-NTNU/bismo
